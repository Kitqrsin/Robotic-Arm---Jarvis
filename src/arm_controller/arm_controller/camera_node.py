#!/usr/bin/env python3
"""
Camera Node - Captures frames from Raspberry Pi Camera V2
Publishes raw images to /camera/image_raw for downstream processing.

Supports (tried in order):
  1. picamera2 (native libcamera, best on Pi OS)
  2. libcamera pipeline via rpicam-vid → OpenCV (for Docker / Pi 5)
  3. OpenCV VideoCapture with V4L2 device scan
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np
import time
import subprocess
import shutil
import os

# Try picamera2 first (native Pi Camera support), fall back to OpenCV
PICAMERA2_AVAILABLE = False
OPENCV_AVAILABLE = False

try:
    from picamera2 import Picamera2
    # Also verify libcamera is actually usable (not just pip-installed)
    import libcamera  # noqa: F401
    PICAMERA2_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    pass

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    pass


class CameraNode(Node):
    """ROS2 node that captures frames from Pi Camera V2 and publishes them."""

    def __init__(self):
        super().__init__('camera_node')

        # --- Parameters ---
        self.declare_parameter('frame_rate', 15.0)       # FPS
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('device_id', -1)          # -1 = auto-scan
        self.declare_parameter('camera_backend', 'auto')  # 'picamera2', 'libcamera', 'opencv', 'auto'

        self.fps = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.device_id = self.get_parameter('device_id').value
        self.backend = self.get_parameter('camera_backend').value

        # --- Publishers ---
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # --- Camera setup ---
        self.camera = None          # picamera2 instance
        self.cv_cap = None          # OpenCV VideoCapture
        self.active_backend = None  # name of working backend
        self._init_camera()

        # --- Capture timer ---
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture_and_publish)
        self.frame_count = 0

        self.get_logger().info(
            f'Camera node started — {self.width}x{self.height} @ {self.fps} FPS'
            f' [backend: {self.active_backend or "NONE"}]'
        )

    # ------------------------------------------------------------------ #
    #  Camera initialisation                                               #
    # ------------------------------------------------------------------ #
    def _init_camera(self):
        """Try to initialise the camera with the best available backend."""
        backend = self.backend

        # --- 1. picamera2 (native libcamera) ---
        if backend in ('auto', 'picamera2') and PICAMERA2_AVAILABLE:
            if self._try_picamera2():
                return

        # --- 2. libcamera via rpicam-vid pipeline (Pi 5 in Docker) ---
        if backend in ('auto', 'libcamera') and OPENCV_AVAILABLE:
            if self._try_libcamera_pipe():
                return

        # --- 3. OpenCV direct V4L2 ---
        if backend in ('auto', 'opencv') and OPENCV_AVAILABLE:
            if self._try_opencv():
                return

        self.get_logger().error(
            'No camera backend available!\n'
            '  Checklist:\n'
            '  1. Is the camera detected?  Run: rpicam-hello --list-cameras\n'
            '  2. Pi 5 needs a 22-to-15 pin adapter cable for Pi Camera V2\n'
            '  3. Docker needs: privileged=true, /dev:/dev volume mount\n'
            '  4. Install opencv-python: pip3 install opencv-python-headless'
        )

    def _try_picamera2(self):
        """Attempt native picamera2 backend."""
        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={'size': (self.width, self.height), 'format': 'RGB888'}
            )
            self.camera.configure(config)
            self.camera.start()
            time.sleep(1.0)  # let auto-exposure settle
            self.active_backend = 'picamera2'
            self.get_logger().info('Using picamera2 backend (native libcamera)')
            return True
        except Exception as e:
            self.get_logger().warn(f'picamera2 failed: {e}')
            self.camera = None
            return False

    def _try_libcamera_pipe(self):
        """
        Use rpicam-vid (Pi 5) or libcamera-vid to stream to stdout,
        then read frames with OpenCV.  Works inside Docker with /dev mounted.
        """
        # Find the right command
        vid_cmd = None
        for cmd in ('rpicam-vid', 'libcamera-vid'):
            if shutil.which(cmd):
                vid_cmd = cmd
                break

        if vid_cmd is None:
            self.get_logger().warn(
                'libcamera pipeline: rpicam-vid / libcamera-vid not found'
            )
            return False

        # Check if any camera is available
        hello_cmd = vid_cmd.replace('-vid', '-hello')
        try:
            result = subprocess.run(
                [hello_cmd, '--list-cameras'],
                capture_output=True, text=True, timeout=5
            )
            if 'No cameras available' in result.stderr or 'No cameras available' in result.stdout:
                self.get_logger().warn('libcamera: no cameras detected by hardware')
                return False
        except Exception:
            pass  # continue trying anyway

        # Build GStreamer-style pipeline for OpenCV
        # rpicam-vid outputs h264 by default; we use --codec mjpeg for easier decode
        pipeline = (
            f'{vid_cmd} --width {self.width} --height {self.height} '
            f'--framerate {int(self.fps)} --codec mjpeg --nopreview '
            f'-t 0 -o - 2>/dev/null'
        )

        try:
            # Use OpenCV with a pipe:// or file descriptor approach
            # Actually, use subprocess + manual JPEG decode for reliability
            self._libcam_proc = subprocess.Popen(
                pipeline, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                bufsize=self.width * self.height * 3
            )
            # Try to read one test frame
            time.sleep(1.0)
            if self._libcam_proc.poll() is not None:
                self.get_logger().warn(
                    f'libcamera pipeline exited immediately (code {self._libcam_proc.returncode})'
                )
                return False

            self.active_backend = 'libcamera-pipe'
            self.get_logger().info(f'Using libcamera pipeline backend ({vid_cmd})')
            self._mjpeg_buffer = b''
            return True
        except Exception as e:
            self.get_logger().warn(f'libcamera pipeline failed: {e}')
            return False

    def _try_opencv(self):
        """Scan V4L2 devices for a working camera."""
        # On Pi 5, video devices start at /dev/video19+
        # On Pi 4, they start at /dev/video0
        # Try explicit device_id first, then scan
        if self.device_id >= 0:
            candidates = [self.device_id]
        else:
            # Build list from what actually exists in /dev
            candidates = []
            for i in range(36):
                if os.path.exists(f'/dev/video{i}'):
                    candidates.append(i)

        for dev_id in candidates:
            try:
                cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cv_cap = cap
                        self.active_backend = f'opencv (device {dev_id})'
                        self.get_logger().info(
                            f'Using OpenCV backend (/dev/video{dev_id})'
                        )
                        return True
                cap.release()
            except Exception:
                continue

        self.get_logger().warn(
            f'OpenCV: no working camera found (scanned {len(candidates)} devices)'
        )
        return False

    # ------------------------------------------------------------------ #
    #  Frame capture + publish                                             #
    # ------------------------------------------------------------------ #
    def capture_and_publish(self):
        """Grab one frame and publish it as sensor_msgs/Image."""
        frame = None

        # --- picamera2 ---
        if self.camera is not None:
            try:
                frame = self.camera.capture_array()  # RGB numpy array
            except Exception as e:
                self.get_logger().error(f'picamera2 capture error: {e}')
                return

        # --- libcamera pipe (MJPEG) ---
        elif hasattr(self, '_libcam_proc') and self._libcam_proc is not None:
            frame = self._read_mjpeg_frame()

        # --- OpenCV V4L2 ---
        elif self.cv_cap is not None:
            ret, bgr = self.cv_cap.read()
            if not ret:
                self.get_logger().warn('OpenCV frame grab failed')
                return
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        else:
            return  # no camera

        if frame is None:
            return

        # --- Build ROS Image message ---
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3  # 3 bytes per pixel (RGB)
        msg.data = frame.tobytes()

        self.image_pub.publish(msg)

        # --- Camera info (minimal, update after calibration) ---
        info_msg = CameraInfo()
        info_msg.header = msg.header
        info_msg.width = frame.shape[1]
        info_msg.height = frame.shape[0]
        self.info_pub.publish(info_msg)

        self.frame_count += 1
        if self.frame_count % (int(self.fps) * 10) == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def _read_mjpeg_frame(self):
        """Read one JPEG frame from the rpicam-vid MJPEG stream."""
        try:
            # Read chunks until we find SOI (0xFFD8) and EOI (0xFFD9)
            chunk = self._libcam_proc.stdout.read(4096)
            if not chunk:
                return None
            self._mjpeg_buffer += chunk

            # Look for JPEG boundaries
            while True:
                soi = self._mjpeg_buffer.find(b'\xff\xd8')
                if soi == -1:
                    # No start marker — keep only last 2 bytes
                    self._mjpeg_buffer = self._mjpeg_buffer[-2:]
                    return None

                eoi = self._mjpeg_buffer.find(b'\xff\xd9', soi + 2)
                if eoi == -1:
                    # Have start but no end yet — read more
                    more = self._libcam_proc.stdout.read(
                        max(4096, self.width * self.height)
                    )
                    if not more:
                        return None
                    self._mjpeg_buffer += more
                    continue

                # Extract complete JPEG
                jpeg_data = self._mjpeg_buffer[soi:eoi + 2]
                self._mjpeg_buffer = self._mjpeg_buffer[eoi + 2:]

                # Decode JPEG → numpy array
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8),
                    cv2.IMREAD_COLOR
                )
                if frame is not None:
                    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return None

        except Exception as e:
            self.get_logger().warn(f'MJPEG read error: {e}')
            return None

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #
    def destroy_node(self):
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
        if self.cv_cap is not None:
            self.cv_cap.release()
        if hasattr(self, '_libcam_proc') and self._libcam_proc is not None:
            try:
                self._libcam_proc.terminate()
                self._libcam_proc.wait(timeout=3)
            except Exception:
                self._libcam_proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
