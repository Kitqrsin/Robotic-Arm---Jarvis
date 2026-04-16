#!/usr/bin/env python3
"""
Face-Tracking PID Turret Node
==============================
Standalone ROS2 node that captures 320×240 frames from a V4L2 camera,
detects faces with OpenCV Haar cascades, and drives the arm's base (pan)
and shoulder (tilt) joints via smooth PID-controlled position updates.

Published topics
----------------
/joint_commands    Float32MultiArray   6-element array [base, shoulder,
                                       elbow, wrist, wrist_rotate, gripper]
                                       matching servo_node's expected format.
/face_track/pid_output  geometry_msgs/Point  Debug: (pan_deg, tilt_deg,
                                              face_area_fraction).

Architecture (3 threads)
------------------------
1. Camera thread (daemon)   – tight V4L2 read loop at 320×240.
2. ROS2 main thread         – rclpy.spin() + 30 Hz timer for detect → PID → publish.
3. Tkinter GUI thread (daemon) – 6 sliders for live PID gain tuning.

No raw images are published over ROS2.

Usage
-----
    ros2 run arm_controller face_track_pid
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point

# ---------------------------------------------------------------------------
#  PID Controller
# ---------------------------------------------------------------------------

class PIDController:
    """Discrete PID with anti-windup and output clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -5.0, output_max: float = 5.0,
                 integral_max: float = 200.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def compute(self, error: float, dt: float) -> float:
        """Return clamped PID output given *error* and time-step *dt*."""
        if dt <= 0.0:
            return 0.0

        # Proportional
        p = self.kp * error

        # Integral (with anti-windup clamp)
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        i = self.ki * self._integral

        # Derivative (skip on first call — no previous error)
        if self._first:
            d = 0.0
            self._first = False
        else:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p + i + d
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        """Zero out integral and derivative memory."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


# ---------------------------------------------------------------------------
#  Face-Tracking ROS2 Node
# ---------------------------------------------------------------------------

class FaceTrackPIDNode(Node):
    """ROS2 node: camera → Haar face detect → PID → /joint_commands."""

    # Frame size
    FRAME_W = 320
    FRAME_H = 240
    CENTRE_X = FRAME_W / 2.0   # 160
    CENTRE_Y = FRAME_H / 2.0   # 120

    # Joint indices (must match servo_node convention)
    J_BASE = 0          # pan   (0-270°)
    J_SHOULDER = 1      # tilt  (0-180°, hardware-inverted by servo_node)
    J_ELBOW = 2
    J_WRIST = 3
    J_WRIST_ROT = 4
    J_GRIPPER = 5
    NUM_JOINTS = 6

    # Position limits (GUI-space degrees)
    BASE_MIN, BASE_MAX = 0.0, 270.0
    SHOULDER_MIN, SHOULDER_MAX = 0.0, 180.0

    # Default "park" angles for joints not under PID control
    DEFAULT_ANGLES = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]

    def __init__(self):
        super().__init__('face_track_pid')

        # ---- ROS2 parameters (overridable at launch) ----
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('loop_hz', 30.0)
        self.declare_parameter('kp_pan', 0.035)
        self.declare_parameter('ki_pan', 0.002)
        self.declare_parameter('kd_pan', 0.008)
        self.declare_parameter('kp_tilt', 0.035)
        self.declare_parameter('ki_tilt', 0.002)
        self.declare_parameter('kd_tilt', 0.008)
        self.declare_parameter('max_step_deg', 5.0)
        self.declare_parameter('lost_face_frames', 15)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('servo_speed', 80.0)

        cam_idx    = self.get_parameter('camera_index').value
        loop_hz    = self.get_parameter('loop_hz').value
        kp_pan     = self.get_parameter('kp_pan').value
        ki_pan     = self.get_parameter('ki_pan').value
        kd_pan     = self.get_parameter('kd_pan').value
        kp_tilt    = self.get_parameter('kp_tilt').value
        ki_tilt    = self.get_parameter('ki_tilt').value
        kd_tilt    = self.get_parameter('kd_tilt').value
        max_step   = self.get_parameter('max_step_deg').value
        self._lost_threshold = self.get_parameter('lost_face_frames').value
        self._show_gui = self.get_parameter('show_gui').value
        servo_speed = self.get_parameter('servo_speed').value

        # ---- PID controllers ----
        self.pid_pan  = PIDController(kp_pan,  ki_pan,  kd_pan,
                                      -max_step, max_step)
        self.pid_tilt = PIDController(kp_tilt, ki_tilt, kd_tilt,
                                      -max_step, max_step)

        # ---- Publishers ----
        self.joint_pub = self.create_publisher(
            Float32MultiArray, '/joint_commands', 10)
        self.debug_pub = self.create_publisher(
            Point, '/face_track/pid_output', 10)
        self.speed_pub = self.create_publisher(
            Float32, '/servo_speed', 10)

        # Publish desired servo speed once at startup
        speed_msg = Float32()
        speed_msg.data = float(servo_speed)
        self.speed_pub.publish(speed_msg)

        # ---- State ----
        self._positions = list(self.DEFAULT_ANGLES)   # current commanded angles
        self._lost_count = 0
        self._last_tick = time.monotonic()

        # ---- Haar cascade ----
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            self.get_logger().error(
                f'Failed to load Haar cascade from {cascade_path}')

        # ---- Camera (threaded) ----
        self._frame = None
        self._frame_lock = threading.Lock()
        self._cam_thread = threading.Thread(
            target=self._camera_loop, args=(cam_idx,), daemon=True)
        self._cam_thread.start()

        # ---- Tkinter tuning GUI (threaded) ----
        if self._show_gui:
            self._gui_thread = threading.Thread(
                target=self._tkinter_loop, daemon=True)
            self._gui_thread.start()

        # ---- Main processing timer ----
        period = 1.0 / loop_hz
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'FaceTrackPID started — cam={cam_idx}, {loop_hz} Hz, '
            f'PID pan({kp_pan},{ki_pan},{kd_pan}) '
            f'tilt({kp_tilt},{ki_tilt},{kd_tilt})')

    # ------------------------------------------------------------------
    #  Camera thread
    # ------------------------------------------------------------------
    def _camera_loop(self, cam_idx: int):
        """Tight V4L2 capture loop — runs in a daemon thread."""
        cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            # Fallback: try without explicit V4L2 backend
            cap = cv2.VideoCapture(cam_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimise latency

        if not cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera index {cam_idx}')
            return

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Camera opened: requested {self.FRAME_W}x{self.FRAME_H}, '
            f'actual {actual_w}x{actual_h}')

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            # Store only the latest frame (discard stale ones)
            with self._frame_lock:
                self._frame = frame

        cap.release()

    # ------------------------------------------------------------------
    #  Main 30 Hz tick
    # ------------------------------------------------------------------
    def _tick(self):
        """Grab latest frame → detect face → PID → publish."""
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        # Grab latest frame
        with self._frame_lock:
            frame = self._frame
            self._frame = None   # consume it

        if frame is None:
            return

        # ---- Face detection ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE,
        )

        if len(faces) == 0:
            self._lost_count += 1
            if self._lost_count >= self._lost_threshold:
                self.pid_pan.reset()
                self.pid_tilt.reset()
            # Hold position — publish current angles unchanged
            self._publish_joints()
            self._publish_debug(0.0, 0.0, 0.0)
            return

        self._lost_count = 0

        # Pick the largest face
        areas = [w * h for (_, _, w, h) in faces]
        idx = int(np.argmax(areas))
        (fx, fy, fw, fh) = faces[idx]

        face_cx = fx + fw / 2.0
        face_cy = fy + fh / 2.0

        # Error in pixels (positive = face is right / below centre)
        err_x = face_cx - self.CENTRE_X
        err_y = face_cy - self.CENTRE_Y

        # ---- PID compute ----
        pan_delta  = self.pid_pan.compute(err_x, dt)    # degrees/tick
        tilt_delta = self.pid_tilt.compute(err_y, dt)

        # Eye-in-hand convention:
        #   face to the RIGHT (err_x > 0) → DECREASE base to swing camera right
        #   face BELOW centre (err_y > 0) → DECREASE shoulder to tilt camera down
        new_base = self._positions[self.J_BASE] - pan_delta
        new_shoulder = self._positions[self.J_SHOULDER] - tilt_delta

        # Clamp to joint limits
        new_base = max(self.BASE_MIN, min(self.BASE_MAX, new_base))
        new_shoulder = max(self.SHOULDER_MIN,
                           min(self.SHOULDER_MAX, new_shoulder))

        self._positions[self.J_BASE] = new_base
        self._positions[self.J_SHOULDER] = new_shoulder

        # ---- Publish ----
        self._publish_joints()

        face_area_frac = (fw * fh) / (self.FRAME_W * self.FRAME_H)
        self._publish_debug(pan_delta, tilt_delta, face_area_frac)

    # ------------------------------------------------------------------
    #  ROS2 publishing helpers
    # ------------------------------------------------------------------
    def _publish_joints(self):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in self._positions]
        self.joint_pub.publish(msg)

    def _publish_debug(self, pan: float, tilt: float, area: float):
        msg = Point()
        msg.x = pan
        msg.y = tilt
        msg.z = area
        self.debug_pub.publish(msg)

    # ------------------------------------------------------------------
    #  Tkinter tuning GUI (runs in its own thread)
    # ------------------------------------------------------------------
    def _tkinter_loop(self):
        """Blocking Tkinter mainloop — must run in a daemon thread."""
        try:
            import tkinter as tk
        except ImportError:
            self.get_logger().warn(
                'python3-tk not available — PID tuning GUI disabled')
            return

        root = tk.Tk()
        root.title('PID Face-Track Tuning')
        root.configure(bg='#2b2b2b')
        root.geometry('380x520')

        style_kw = dict(bg='#3b3b3b', fg='white',
                        highlightbackground='#3b3b3b',
                        troughcolor='#555555')

        # ---- Helper to create a labelled slider ----
        def make_slider(parent, label_text, from_, to_, resolution,
                        initial, row):
            lbl = tk.Label(parent, text=label_text, font=('Arial', 10),
                           bg='#3b3b3b', fg='white', anchor='w', width=12)
            lbl.grid(row=row, column=0, padx=5, pady=4, sticky='w')

            val_lbl = tk.Label(parent, text=f'{initial:.4f}',
                               font=('Arial', 10, 'bold'),
                               bg='#3b3b3b', fg='#00ffff', width=8)
            val_lbl.grid(row=row, column=2, padx=5)

            slider = tk.Scale(
                parent, from_=from_, to=to_, orient=tk.HORIZONTAL,
                resolution=resolution, showvalue=False,
                length=160, **style_kw,
                command=lambda v, vl=val_lbl: vl.config(text=f'{float(v):.4f}'))
            slider.set(initial)
            slider.grid(row=row, column=1, padx=5, pady=4)
            return slider

        # ---- Pan PID group ----
        pan_frame = tk.LabelFrame(root, text='Pan (Base) PID',
                                  font=('Arial', 11, 'bold'),
                                  bg='#3b3b3b', fg='#00ff00', bd=2)
        pan_frame.pack(fill=tk.X, padx=10, pady=5)

        sl_kp_pan  = make_slider(pan_frame, 'Kp', 0.0, 0.15, 0.001,
                                 self.pid_pan.kp, 0)
        sl_ki_pan  = make_slider(pan_frame, 'Ki', 0.0, 0.02, 0.0005,
                                 self.pid_pan.ki, 1)
        sl_kd_pan  = make_slider(pan_frame, 'Kd', 0.0, 0.05, 0.001,
                                 self.pid_pan.kd, 2)

        # ---- Tilt PID group ----
        tilt_frame = tk.LabelFrame(root, text='Tilt (Shoulder) PID',
                                   font=('Arial', 11, 'bold'),
                                   bg='#3b3b3b', fg='#00ccff', bd=2)
        tilt_frame.pack(fill=tk.X, padx=10, pady=5)

        sl_kp_tilt = make_slider(tilt_frame, 'Kp', 0.0, 0.15, 0.001,
                                 self.pid_tilt.kp, 0)
        sl_ki_tilt = make_slider(tilt_frame, 'Ki', 0.0, 0.02, 0.0005,
                                 self.pid_tilt.ki, 1)
        sl_kd_tilt = make_slider(tilt_frame, 'Kd', 0.0, 0.05, 0.001,
                                 self.pid_tilt.kd, 2)

        # ---- Max step slider ----
        step_frame = tk.LabelFrame(root, text='Limits',
                                   font=('Arial', 11, 'bold'),
                                   bg='#3b3b3b', fg='#ffaa00', bd=2)
        step_frame.pack(fill=tk.X, padx=10, pady=5)

        sl_max_step = make_slider(step_frame, 'Max °/tick', 1.0, 15.0,
                                  0.5, self.pid_pan.output_max, 0)

        # ---- Status label ----
        self._gui_status_var = tk.StringVar(value='Waiting for camera...')
        status_lbl = tk.Label(root, textvariable=self._gui_status_var,
                              font=('Arial', 9), bg='#2b2b2b',
                              fg='#aaaaaa', anchor='w')
        status_lbl.pack(fill=tk.X, padx=10, pady=(8, 4))

        # ---- Periodic gain sync (every 100 ms) ----
        def sync_gains():
            self.pid_pan.kp  = float(sl_kp_pan.get())
            self.pid_pan.ki  = float(sl_ki_pan.get())
            self.pid_pan.kd  = float(sl_kd_pan.get())
            self.pid_tilt.kp = float(sl_kp_tilt.get())
            self.pid_tilt.ki = float(sl_ki_tilt.get())
            self.pid_tilt.kd = float(sl_kd_tilt.get())

            max_s = float(sl_max_step.get())
            self.pid_pan.output_min  = -max_s
            self.pid_pan.output_max  =  max_s
            self.pid_tilt.output_min = -max_s
            self.pid_tilt.output_max =  max_s

            # Update status text
            base = self._positions[self.J_BASE]
            sh   = self._positions[self.J_SHOULDER]
            lost = self._lost_count
            self._gui_status_var.set(
                f'Base={base:.1f}°  Shoulder={sh:.1f}°  '
                f'Lost={lost}')

            root.after(100, sync_gains)

        root.after(100, sync_gains)

        # ---- Run ----
        try:
            root.mainloop()
        except Exception:
            pass  # window closed — daemon thread will exit


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down face_track_pid...')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
