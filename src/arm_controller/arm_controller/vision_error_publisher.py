#!/usr/bin/env python3
"""
Vision Error Publisher — Eye-in-Hand Visual Servoing

Subscribes to /camera/image_raw (from camera_node), detects faces using
OpenCV Haar cascades (lightweight — no GPU needed on Pi 5), and publishes
a normalised error vector on /vision/error as geometry_msgs/Vector3:

    x : horizontal error  −1.0 (face left)  →  +1.0 (face right)
    y : vertical   error  −1.0 (face bottom) →  +1.0 (face top)
    z : depth/size error   (target_area − current_area)
        positive  → face is too far  (smaller than desired)
        negative  → face is too close (larger than desired)

When no face is detected the node publishes (0, 0, 0) so the arm holds
its current position.

Usage:
    ros2 run arm_controller vision_error_publisher
    ros2 run arm_controller vision_error_publisher --ros-args \
        -p target_area_frac:=0.08 -p cascade:=haarcascade_frontalface_default.xml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import Vector3
import numpy as np
import time
import os

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class VisionErrorPublisher(Node):
    """Detect faces in camera frames and publish normalised error vector."""

    def __init__(self):
        super().__init__('vision_error_publisher')

        # ---- Parameters ----
        self.declare_parameter('target_area_frac', 0.08)
        self.declare_parameter('cascade', 'haarcascade_frontalface_default.xml')
        self.declare_parameter('min_face_size', 40)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('min_neighbours', 5)
        self.declare_parameter('detect_interval', 2)       # process every Nth frame
        self.declare_parameter('publish_rate', 15.0)        # Hz for zero-vector hold

        self.target_area_frac = self.get_parameter('target_area_frac').value
        self.min_face_size = int(self.get_parameter('min_face_size').value)
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_neighbours = int(self.get_parameter('min_neighbours').value)
        self.detect_interval = max(1, int(self.get_parameter('detect_interval').value))

        # ---- Load Haar cascade ----
        self.face_cascade = None
        if CV2_AVAILABLE:
            cascade_name = self.get_parameter('cascade').value
            # Try the parameter path first, then fall back to OpenCV data dir
            if os.path.isfile(cascade_name):
                cascade_path = cascade_name
            else:
                cv2_data = getattr(cv2, 'data', None)
                data_dir = cv2_data.haarcascades if cv2_data else ''
                cascade_path = os.path.join(data_dir, cascade_name)

            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                self.get_logger().error(
                    f'Failed to load cascade from {cascade_path}. '
                    'Face detection will not work.'
                )
                self.face_cascade = None
            else:
                self.get_logger().info(f'Loaded face cascade: {cascade_path}')
        else:
            self.get_logger().error(
                'OpenCV (cv2) not available — vision_error_publisher cannot run.'
            )

        # ---- Publisher ----
        self.error_pub = self.create_publisher(Vector3, '/vision/error', 10)

        # ---- Subscriber to camera frames ----
        self.image_sub = self.create_subscription(
            ROSImage, '/camera/image_raw', self._image_callback, 10
        )

        # ---- State ----
        self._frame_count = 0
        self._last_error = Vector3()     # last published error (hold on skip)
        self._last_detect_time = 0.0
        self._face_lost_frames = 0       # consecutive frames without a face

        self.get_logger().info(
            f'VisionErrorPublisher ready — '
            f'target_area_frac={self.target_area_frac:.2f}, '
            f'detect_interval={self.detect_interval}'
        )

    # ------------------------------------------------------------------ #
    #  Image callback                                                      #
    # ------------------------------------------------------------------ #
    def _image_callback(self, msg: ROSImage):
        """Receive a ROS Image, detect faces, publish error vector."""
        if self.face_cascade is None:
            return

        self._frame_count += 1
        if self._frame_count % self.detect_interval != 0:
            # Re-publish the previous error so the controller keeps moving
            self.error_pub.publish(self._last_error)
            return

        # ---- Decode ROS Image to numpy (BGR or RGB) ----
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ('rgb8', 'bgr8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if msg.encoding == 'rgb8':
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
            else:
                self.get_logger().warn(
                    f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0
                )
                return
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}', throttle_duration_sec=5.0)
            return

        # ---- Detect faces ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbours,
            minSize=(self.min_face_size, self.min_face_size),
        )

        error = Vector3()

        if len(faces) > 0:
            self._face_lost_frames = 0

            # Pick the largest face (by area)
            areas = [fw * fh for (_, _, fw, fh) in faces]
            idx = int(np.argmax(areas))
            fx, fy, fw, fh = faces[idx]

            # Centre of detected face
            cx = fx + fw / 2.0
            cy = fy + fh / 2.0

            # Normalised error: [-1, +1]
            # x: positive = face is to the RIGHT of frame centre
            error.x = (cx - w / 2.0) / (w / 2.0)
            # y: positive = face is ABOVE centre (image y increases downward,
            #    so we invert so that +y = up in the robot's world)
            error.y = -((cy - h / 2.0) / (h / 2.0))

            # z: depth error based on face area fraction
            current_area_frac = (fw * fh) / (w * h)
            error.z = float(self.target_area_frac - current_area_frac)

            self.get_logger().debug(
                f'Face @ ({cx:.0f},{cy:.0f}) size={fw}x{fh} '
                f'err=({error.x:.2f}, {error.y:.2f}, {error.z:.3f})'
            )
        else:
            self._face_lost_frames += 1
            # After a short grace period, zero out the error so the arm
            # stops nudging (holds position).
            if self._face_lost_frames > 5:
                error.x = 0.0
                error.y = 0.0
                error.z = 0.0
            else:
                # Keep the last error briefly (momentum) to smooth jitter
                error = self._last_error

        self._last_error = error
        self.error_pub.publish(error)


def main(args=None):
    rclpy.init(args=args)
    node = VisionErrorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
