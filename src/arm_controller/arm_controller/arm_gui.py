#!/usr/bin/env python3
"""
Jarvis Robot Arm Control GUI
Simple Tkinter-based control interface for testing ROS2 integration
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math
import subprocess
import shutil
import os
import time
import struct
import fcntl

# Detect Docker environment
IN_DOCKER = os.path.exists('/.dockerenv') or os.path.exists('/run/.containerenv')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image as ROSImage
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

import numpy as np

# Try to import Pillow for camera display
try:
    from PIL import Image as PILImage, ImageTk, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Try to import OpenCV for direct camera fallback
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# Import IK solvers and 3D visualization
from arm_controller.ik_solver import ArmIKSolver
from arm_controller.arm_3d_visualization import Arm3DVisualization

# Import in-process object detector for camera overlay
from arm_controller.gui_object_detector import GUIObjectDetector

# Try to import MoveIt IK client (optional)
try:
    from arm_controller.moveit_ik_client import MoveItIKClient
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    MoveItIKClient = None


class ArmControlGUI(Node):
    def __init__(self, root):
        super().__init__('arm_control_gui')
        
        self.root = root
        self.root.title("Jarvis Robot Arm Controller")
        self.root.geometry("1600x1050")
        self.root.configure(bg='#2b2b2b')
        
        # Joint names matching your OnShape URDF
        self.joint_names = [
            'base',           # Base rotation (J1)
            'shoulder',       # Shoulder pitch (J2)
            'elbow',          # Elbow pitch (J3)
            'wrist',          # Wrist pitch (J4)
            'wrist_rotate',   # Wrist roll (J5)
            'gripper'         # Gripper (J6) - internal name for hardware
        ]
        
        # Joint names for MoveIt planning (5 DOF arm - excludes gripper)
        self.moveit_joint_names = [
            'base',
            'shoulder', 
            'elbow',
            'wrist',
            'wrist_rotate'
        ]
        
        # Joint names for RViz visualization (includes gripper_base for visualization)
        # The URDF has a 'gripper_base' joint that controls gripper fingers
        self.rviz_joint_names = [
            'base',
            'shoulder',
            'elbow',
            'wrist',
            'wrist_rotate',
            'gripper_base'   # URDF gripper joint name
        ]
        
        # Current joint positions (in degrees)
        self.current_positions = [90.0] * 6
        self.target_positions = [90.0] * 6
        
        # Initialize IK solvers
        # Custom geometric solver as fallback
        self.custom_ik_solver = ArmIKSolver()
        # MoveIt IK client (will be initialized after node setup)
        self.moveit_ik = None
        self.use_moveit = True  # Prefer MoveIt when available
        
        # 3D visualization (will be set up in setup_gui)
        self.viz_3d = None
        
        # Camera feed state
        self.camera_enabled = tk.BooleanVar(value=True)
        self.camera_label = None          # Tkinter Label widget for video feed
        self.camera_photo = None          # Current PhotoImage (must keep reference)
        self.latest_camera_frame = None   # Latest numpy RGB frame (set from any source)
        self._camera_lock = threading.Lock()
        self._direct_cam_cap = None       # OpenCV VideoCapture (direct fallback)
        self._direct_cam_proc = None      # rpicam-vid subprocess (direct fallback)
        self._direct_cam_backend = None   # 'ros2', 'libcamera', 'opencv', or None
        self._camera_frame_count = 0
        self._camera_snapshot_count = 0
        self.camera_status_label = None
        self.camera_fps_label = None
        self._cam_fps_time = time.time()
        self._cam_fps_count = 0

        # Object detection overlay (face detection via Haar cascades)
        self.detect_enabled = tk.BooleanVar(value=False)
        self.object_detector = GUIObjectDetector(
            detect_every_n=3,
            min_face_size=50,
            scale_factor=1.15,
            min_neighbours=5,
            logger=self.get_logger(),
        )
        self.detect_info_label = None

        # Face-following mode
        self.follow_enabled = tk.BooleanVar(value=False)
        self._follow_last_time = 0.0      # rate-limit follow commands
        self._follow_interval = 0.12      # seconds between follow updates (~8 Hz)
        self._follow_lost_count = 0       # frames without a face
        
        # Auto-follow mode: arm tracks 3D target in real-time
        self.auto_follow_active = tk.BooleanVar(value=False)

        # ---- Handshake Mode state ----
        self.handshake_enabled = tk.BooleanVar(value=False)
        self._handshake_in_progress = False
        self._handshake_detect_count = 0
        self._handshake_detect_threshold = 10  # consecutive frames before triggering
        self._handshake_cancelled = False       # set True to abort mid-sequence

        # ---- Bartender Mode state ----
        self.bartender_enabled = tk.BooleanVar(value=False)
        self._bartender_in_progress = False
        self._bartender_cancelled = False
        self._bartender_detect_count = 0
        self._bartender_detect_threshold = 8   # consecutive frames with a cup
        self._bartender_cup = None              # latest cup detection dict

        # ---- Eye-in-Hand Visual Servoing state ----
        self.visual_servo_enabled = tk.BooleanVar(value=False)
        self._vs_gain_xy = 0.0008      # metres per unit error per tick (XY)
        self._vs_gain_z  = 0.02        # metres per unit error per tick (depth)
        self._vs_max_step = 0.003      # max displacement per tick (metres) — velocity cap
        self._vs_interval = 0.10       # seconds between servoing ticks
        self._vs_last_time = 0.0
        self._vs_last_error = Vector3() # latest error from /vision/error
        self._vs_error_fresh = False    # True once a new error arrives
        # Flying-target position (metres). Initialised from current FK on
        # first activation so the arm starts from where it already is.
        self._vs_target = None          # (x, y, z) or None = uninitialised
        
        # Real-time mode flag and update control
        self.realtime_mode = tk.BooleanVar(value=False)
        self.update_pending = False
        self.last_update_time = 0
        self.update_interval = 0.1  # 100ms between updates
        self.cartesian_update_pending = False
        
        # ROS2 Publishers and Subscribers
        self.joint_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )
        
        # Also publish to joint_states for visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for target position marker in RViz (orange)
        self.marker_pub = self.create_publisher(
            Marker,
            '/target_position_marker',
            10
        )
        
        # Publisher for gripper position marker in RViz (green)
        self.gripper_marker_pub = self.create_publisher(
            Marker,
            '/gripper_position_marker',
            10
        )
        
        # Publisher for servo speed control
        self.speed_pub = self.create_publisher(
            Float32,
            '/servo_speed',
            10
        )
        
        # Publisher for emergency stop (controls PCA9685 OE pin)
        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        # Subscriber for camera feed from camera_node
        self.camera_sub = self.create_subscription(
            ROSImage,
            '/camera/image_raw',
            self.camera_image_callback,
            10
        )
        self._ros2_camera_active = False
        self._ros2_camera_last_time = 0.0

        # Subscriber for visual-servoing error vector
        self.vision_error_sub = self.create_subscription(
            Vector3,
            '/vision/error',
            self._vision_error_callback,
            10
        )
        
        # Track emergency stop state
        self.estop_active = False
        
        # Initialize MoveIt IK client (if available)
        if MOVEIT_AVAILABLE:
            self.moveit_ik = MoveItIKClient(self)
            self.get_logger().info("MoveIt IK client initialized")
        else:
            self.moveit_ik = None
            self.get_logger().warn("MoveIt not available - using custom geometric IK only")
        
        # GUI Elements
        self.setup_gui()
        
        # Start ROS2 spinning in separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Update GUI periodically
        self.update_gui()
    
    def setup_gui(self):
        """Setup all GUI elements"""
        
        # Title
        title_frame = tk.Frame(self.root, bg='#2b2b2b')
        title_frame.pack(pady=10)
        
        title_label = tk.Label(
            title_frame,
            text="🤖 Jarvis Robot Arm Controller",
            font=('Arial', 20, 'bold'),
            bg='#2b2b2b',
            fg='#00ff00'
        )
        title_label.pack()
        
        # Connection status
        self.status_label = tk.Label(
            title_frame,
            text="● ROS2 Connected",
            font=('Arial', 10),
            bg='#2b2b2b',
            fg='#00ff00'
        )
        self.status_label.pack()
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Left panel - Joint Control
        left_panel = tk.LabelFrame(
            main_frame,
            text="Joint Angle Control (Degrees)",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        self.joint_sliders = []
        self.joint_labels = []
        
        for i, joint_name in enumerate(self.joint_names):
            joint_frame = tk.Frame(left_panel, bg='#3b3b3b')
            joint_frame.pack(fill=tk.X, padx=10, pady=8)
            
            # Joint name and current value
            name_label = tk.Label(
                joint_frame,
                text=f"J{i+1}: {joint_name.split('_')[-1]}",
                font=('Arial', 10),
                bg='#3b3b3b',
                fg='white',
                width=15,
                anchor='w'
            )
            name_label.pack(side=tk.LEFT)
            
            # Current value display
            value_label = tk.Label(
                joint_frame,
                text="90.0°",
                font=('Arial', 10, 'bold'),
                bg='#3b3b3b',
                fg='#00ffff',
                width=8
            )
            value_label.pack(side=tk.LEFT, padx=5)
            self.joint_labels.append(value_label)
            
            # Base (J1) is limited to 245° for safety (mechanical max ~270°)
            slider_max = 270 if i == 0 else 180
            
            # Slider
            slider = tk.Scale(
                joint_frame,
                from_=0,
                to=slider_max,
                orient=tk.HORIZONTAL,
                resolution=1,
                bg='#3b3b3b',
                fg='white',
                highlightbackground='#3b3b3b',
                troughcolor='#555555',
                command=lambda val, idx=i: self.on_slider_change(idx, val)
            )
            slider.set(90)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.joint_sliders.append(slider)
        
        # Right panel - Quick Actions & Cartesian Control
        right_panel = tk.Frame(main_frame, bg='#2b2b2b')
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Real-time Mode Toggle
        realtime_frame = tk.Frame(right_panel, bg='#3b3b3b', bd=2, relief=tk.RIDGE)
        realtime_frame.pack(fill=tk.X, pady=5)
        
        realtime_check = tk.Checkbutton(
            realtime_frame,
            text="⚡ Real-time Mode (Auto-Update)",
            variable=self.realtime_mode,
            font=('Arial', 11, 'bold'),
            bg='#3b3b3b',
            fg='#ffaa00',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            activeforeground='#ffcc00'
        )
        realtime_check.pack(pady=8, padx=10)
        
        # Speed Control
        speed_frame = tk.LabelFrame(
            right_panel,
            text="Movement Speed",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        speed_frame.pack(fill=tk.X, pady=5)
        
        speed_inner = tk.Frame(speed_frame, bg='#3b3b3b')
        speed_inner.pack(fill=tk.X, padx=10, pady=5)
        
        slow_label = tk.Label(
            speed_inner,
            text="🐢 Slow",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#aaaaaa'
        )
        slow_label.pack(side=tk.LEFT)
        
        self.speed_value = tk.IntVar(value=50)
        self.speed_slider = tk.Scale(
            speed_inner,
            from_=1,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.speed_value,
            bg='#3b3b3b',
            fg='white',
            highlightbackground='#3b3b3b',
            troughcolor='#555555',
            command=self.on_speed_change
        )
        self.speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        fast_label = tk.Label(
            speed_inner,
            text="🚀 Fast",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#aaaaaa'
        )
        fast_label.pack(side=tk.LEFT)
        
        self.speed_label = tk.Label(
            speed_frame,
            text="Speed: 50%",
            font=('Arial', 10, 'bold'),
            bg='#3b3b3b',
            fg='#00ffff'
        )
        self.speed_label.pack(pady=(0, 5))
        
        # Preset Positions
        preset_frame = tk.LabelFrame(
            right_panel,
            text="Preset Positions",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        preset_frame.pack(fill=tk.X, pady=(0, 10))
        
        presets = [
            ("Home", [90, 90, 90, 90, 90, 90]),
            ("Zero", [0, 0, 0, 0, 0, 0]),
            ("Stretch", [90, 45, 45, 90, 90, 90]),
            ("Rest", [90, 135, 135, 90, 90, 90]),
        ]
        
        for name, angles in presets:
            btn = tk.Button(
                preset_frame,
                text=name,
                font=('Arial', 10),
                bg='#4a4a4a',
                fg='white',
                activebackground='#5a5a5a',
                command=lambda a=angles: self.apply_preset(a)
            )
            btn.pack(side=tk.LEFT, padx=5, pady=5, expand=True, fill=tk.X)
        
        # Cartesian Control (IK)
        cartesian_frame = tk.LabelFrame(
            right_panel,
            text="Cartesian Control (IK) - Millimeters",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        cartesian_frame.pack(fill=tk.X, pady=10)
        
        self.cartesian_entries = {}
        self.cartesian_sliders = {}
        self.cartesian_labels = {}
        
        # Define ranges for each axis (in millimeters)
        # Extended ranges — base servo supports 270° sweep
        cartesian_ranges = {
            'X': (-600, 600),
            'Y': (-600, 600),
            'Z': (0, 600)
        }
        
        for axis in ['X', 'Y', 'Z']:
            axis_frame = tk.Frame(cartesian_frame, bg='#3b3b3b')
            axis_frame.pack(fill=tk.X, padx=10, pady=5)
            
            label = tk.Label(
                axis_frame,
                text=f"{axis}:",
                font=('Arial', 10, 'bold'),
                bg='#3b3b3b',
                fg='white',
                width=3
            )
            label.pack(side=tk.LEFT)
            
            # Value display
            value_label = tk.Label(
                axis_frame,
                text="0 mm" if axis != 'Z' else "200 mm",
                font=('Arial', 9, 'bold'),
                bg='#3b3b3b',
                fg='#00ffff',
                width=8
            )
            value_label.pack(side=tk.LEFT, padx=3)
            self.cartesian_labels[axis] = value_label
            
            # Slider
            min_val, max_val = cartesian_ranges[axis]
            slider = tk.Scale(
                axis_frame,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                resolution=1,
                bg='#3b3b3b',
                fg='white',
                highlightbackground='#3b3b3b',
                troughcolor='#555555',
                showvalue=0,
                command=lambda val, ax=axis: self.on_cartesian_slider_change(ax, val)
            )
            slider.set(0 if axis != 'Z' else 200)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)
            self.cartesian_sliders[axis] = slider
            
            # Text entry
            entry = tk.Entry(
                axis_frame,
                font=('Arial', 9),
                bg='#555555',
                fg='white',
                insertbackground='white',
                width=7
            )
            entry.insert(0, "0" if axis != 'Z' else "200")
            entry.bind('<Return>', lambda e, ax=axis: self.on_cartesian_entry_change(ax))
            entry.bind('<FocusOut>', lambda e, ax=axis: self.on_cartesian_entry_change(ax))
            entry.pack(side=tk.LEFT, padx=3)
            self.cartesian_entries[axis] = entry
        
        ik_btn = tk.Button(
            cartesian_frame,
            text="Move to Position (IK)",
            font=('Arial', 10),
            bg='#006600',
            fg='white',
            activebackground='#008800',
            command=self.apply_cartesian,
            state=tk.NORMAL  # Now enabled with IK solver
        )
        ik_btn.pack(pady=10, padx=10)
        
        # 3D Visualization Panel with draggable IK target
        viz_panel = tk.Frame(main_frame, bg='#2b2b2b')
        viz_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        self.viz_3d = Arm3DVisualization(
            viz_panel,
            self.custom_ik_solver,  # Use custom solver for visualization FK
            on_target_move_callback=self.on_3d_target_move
        )
        
        # Sync initial cartesian values to 3D viz
        self.viz_3d.set_target(0.0, 0.0, 0.2)
        
        # ---- Start / Stop Cartesian Auto-Follow Button ----
        self.start_stop_btn = tk.Button(
            right_panel,
            text="▶  Start",
            font=('Arial', 13, 'bold'),
            bg='#006600',
            fg='white',
            activebackground='#008800',
            command=self.toggle_auto_follow,
            height=1
        )
        self.start_stop_btn.pack(fill=tk.X, pady=5)
        
        # ---- Camera Feed Panel ----
        self._setup_camera_panel(right_panel)
        
        # Control Buttons
        button_frame = tk.LabelFrame(
            right_panel,
            text="Actions",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        button_frame.pack(fill=tk.X, pady=10)
        
        send_btn = tk.Button(
            button_frame,
            text="📤 Send to Robot",
            font=('Arial', 11, 'bold'),
            bg='#0066cc',
            fg='white',
            activebackground='#0088ee',
            command=self.send_command,
            height=2
        )
        send_btn.pack(pady=5, padx=10, fill=tk.X)
        
        stop_btn = tk.Button(
            button_frame,
            text="⛔ EMERGENCY STOP",
            font=('Arial', 11, 'bold'),
            bg='#cc0000',
            fg='white',
            activebackground='#ff0000',
            command=self.emergency_stop,
            height=2
        )
        stop_btn.pack(pady=5, padx=10, fill=tk.X)
        
        # Info panel at bottom
        info_frame = tk.Frame(self.root, bg='#2b2b2b')
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=20, pady=10)
        
        self.info_label = tk.Label(
            info_frame,
            text="Ready. Adjust joint angles and click 'Send to Robot'",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#aaaaaa',
            anchor='w',
            padx=10,
            pady=5
        )
        self.info_label.pack(fill=tk.X)
    
    # ------------------------------------------------------------------ #
    #  Camera Feed                                                         #
    # ------------------------------------------------------------------ #
    def _setup_camera_panel(self, parent):
        """Create the camera feed panel with controls."""
        cam_frame = tk.LabelFrame(
            parent,
            text="📷 Camera Feed",
            font=('Arial', 12, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        cam_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        # -- Controls row --
        ctrl_row = tk.Frame(cam_frame, bg='#3b3b3b')
        ctrl_row.pack(fill=tk.X, padx=5, pady=(5, 2))

        toggle_btn = tk.Checkbutton(
            ctrl_row,
            text="Enable",
            variable=self.camera_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#00ff00',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_camera_toggle
        )
        toggle_btn.pack(side=tk.LEFT)

        snap_btn = tk.Button(
            ctrl_row,
            text="📸 Snapshot",
            font=('Arial', 9),
            bg='#4a4a4a',
            fg='white',
            activebackground='#5a5a5a',
            command=self._camera_snapshot
        )
        snap_btn.pack(side=tk.LEFT, padx=5)

        detect_btn = tk.Checkbutton(
            ctrl_row,
            text="🔍 Detect",
            variable=self.detect_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#ff9900',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_detect_toggle
        )
        detect_btn.pack(side=tk.LEFT, padx=5)

        self.follow_btn = tk.Checkbutton(
            ctrl_row,
            text="� Follow",
            variable=self.follow_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#00ccff',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_follow_toggle
        )
        self.follow_btn.pack(side=tk.LEFT, padx=5)

        self.servo_btn = tk.Checkbutton(
            ctrl_row,
            text="👁 Servo",
            variable=self.visual_servo_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#ff66ff',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_visual_servo_toggle
        )
        self.servo_btn.pack(side=tk.LEFT, padx=5)

        self.handshake_btn = tk.Checkbutton(
            ctrl_row,
            text="\U0001F91D Handshake",
            variable=self.handshake_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#ffcc00',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_handshake_toggle
        )
        self.handshake_btn.pack(side=tk.LEFT, padx=5)

        self.bartender_btn = tk.Checkbutton(
            ctrl_row,
            text="\U0001F943 Bartender",
            variable=self.bartender_enabled,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#32ff7e',
            selectcolor='#555555',
            activebackground='#3b3b3b',
            command=self._on_bartender_toggle
        )
        self.bartender_btn.pack(side=tk.LEFT, padx=5)

        self.camera_fps_label = tk.Label(
            ctrl_row,
            text="FPS: --",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#aaaaaa'
        )
        self.camera_fps_label.pack(side=tk.RIGHT, padx=5)

        self.camera_status_label = tk.Label(
            ctrl_row,
            text="⏳ Connecting...",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#ffaa00'
        )
        self.camera_status_label.pack(side=tk.RIGHT, padx=5)

        # -- Video display (wider 640x480 pixel area) --
        cam_display = tk.Frame(cam_frame, bg='#1a1a1a', width=640, height=480)
        cam_display.pack(padx=5, pady=5)
        cam_display.pack_propagate(False)

        self.camera_label = tk.Label(
            cam_display,
            bg='#1a1a1a',
            text="No Camera Feed",
            fg='#555555',
            font=('Arial', 12)
        )
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        # Detection info label (shows detected objects)
        self.detect_info_label = tk.Label(
            cam_frame,
            text="",
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#ff9900',
            anchor='w',
            wraplength=620
        )
        self.detect_info_label.pack(fill=tk.X, padx=5, pady=(0, 3))

        if not PIL_AVAILABLE:
            self.get_logger().warn(
                'Pillow (PIL) not installed — camera images disabled. '
                'Install with: pip3 install Pillow'
            )
            self.camera_label.configure(
                text="Install Pillow:\npip3 install Pillow",
                fg='#ff6666',
                font=('Arial', 10, 'bold')
            )
            if self.camera_status_label:
                self.camera_status_label.config(
                    text="⚠ No Pillow", fg='#ff6666'
                )
            return

        # Show an initial placeholder
        self._show_camera_placeholder("Waiting for camera...")

        # Start the camera refresh loop (runs on Tkinter main thread)
        self._start_camera_refresh()

        # Kick off the direct-camera fallback probe in a background thread
        threading.Thread(target=self._init_direct_camera, daemon=True).start()

    # ---- ROS2 camera topic callback (runs in ROS spin thread) ----
    def camera_image_callback(self, msg: ROSImage):
        """Receive frames from /camera/image_raw published by camera_node."""
        if not self.camera_enabled.get():
            return
        # Skip heavy ROS2 raw-image decoding when the more efficient
        # shared-file backend is already feeding frames.
        if self._direct_cam_backend == 'shared-file':
            return
        # Skip heavy ROS2 raw-image decoding when the more efficient
        # shared-file backend is already feeding frames.
        if self._direct_cam_backend == 'shared-file':
            return
        try:
            # Convert ROS Image to numpy array
            if msg.encoding in ('rgb8', 'RGB8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
            elif msg.encoding in ('bgr8', 'BGR8'):
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                frame = raw[:, :, ::-1]  # BGR → RGB
            elif msg.encoding in ('mono8',):
                gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width
                )
                frame = np.stack([gray] * 3, axis=-1)
            else:
                self.get_logger().warn(
                    f'Unsupported image encoding: {msg.encoding}', throttle_duration_sec=5.0
                )
                return

            with self._camera_lock:
                self.latest_camera_frame = frame
                self._ros2_camera_active = True
                self._ros2_camera_last_time = time.time()
        except Exception as e:
            self.get_logger().error(f'Camera frame decode error: {e}')

    # ---- Direct camera fallback (like app.py) ----
    # Camera frame file written by host-side camera_stream.sh
    CAMERA_FRAME_FILE = '/workspace/.camera_frame.jpg'

    @staticmethod
    def _is_v4l2_capture_device(dev_path):
        """Check if a /dev/videoN supports VIDEO_CAPTURE via ioctl."""
        VIDIOC_QUERYCAP = 0x80685600
        V4L2_CAP_VIDEO_CAPTURE = 0x00000001
        try:
            fd = os.open(dev_path, os.O_RDWR | os.O_NONBLOCK)
            try:
                buf = bytearray(104)
                fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
                caps = struct.unpack_from('<I', buf, 84)[0]
                return bool(caps & V4L2_CAP_VIDEO_CAPTURE)
            finally:
                os.close(fd)
        except Exception:
            return False

    def _init_direct_camera(self):
        """Try to open a camera directly. Priority:
        1. Shared JPEG file (/workspace/.camera_frame.jpg from camera_stream.sh)
        2. Local rpicam-vid pipe (if binary available)
        3. OpenCV V4L2 (with smart device detection)
        Inside Docker on Pi 5, skip V4L2 (won't work) and use shared file.
        """
        # --- 1. Shared JPEG file (most efficient: ~30 KB JPEG vs 921 KB
        #        raw RGB through ROS2).  Check immediately, don't wait. ---
        if os.path.isfile(self.CAMERA_FRAME_FILE):
            self._direct_cam_backend = 'shared-file'
            self.get_logger().info(
                f'[CAM-GUI] Using shared camera frame file: {self.CAMERA_FRAME_FILE}'
            )
            threading.Thread(target=self._shared_file_loop, daemon=True).start()
            return

        # Wait a moment to see if ROS2 camera topic is active
        time.sleep(2.0)
        if self._ros2_camera_active:
            self.get_logger().info('[CAM-GUI] Using ROS2 /camera/image_raw topic')
            return

        self.get_logger().info('[CAM-GUI] No ROS2 camera topic yet — trying direct capture...')

        # Inside Docker on Pi 5, V4L2 raw capture doesn't work — skip to waiting
        if IN_DOCKER:
            self.get_logger().warn(
                '[CAM-GUI] Inside Docker — V4L2 raw capture not available on Pi 5.\n'
                '  Waiting for shared frame file...\n'
                '  Run on HOST: ./camera_stream.sh start'
            )
            self._direct_cam_backend = 'waiting'
            threading.Thread(target=self._wait_for_shared_file, daemon=True).start()
            return

        # --- 2. Try local rpicam-vid pipe (host only) ---
        vid_cmd = shutil.which('rpicam-vid') or shutil.which('libcamera-vid')
        if vid_cmd:
            try:
                pipeline = [
                    vid_cmd,
                    '--width', '640', '--height', '480',
                    '--framerate', '10',
                    '--codec', 'mjpeg',
                    '--nopreview',
                    '-t', '0',
                    '-o', '-',
                ]
                self._direct_cam_proc = subprocess.Popen(
                    pipeline,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=320 * 240 * 3
                )
                time.sleep(0.8)
                if self._direct_cam_proc.poll() is None:
                    self._direct_cam_backend = 'libcamera'
                    self.get_logger().info(f'[CAM-GUI] Direct camera: libcamera ({vid_cmd})')
                    threading.Thread(target=self._direct_libcam_loop, daemon=True).start()
                    return
                else:
                    self._direct_cam_proc = None
            except Exception as e:
                self.get_logger().warn(f'[CAM-GUI] libcamera pipe failed: {e}')
                self._direct_cam_proc = None

        # --- 3. Try OpenCV V4L2 (host only) ---
        if CV2_AVAILABLE:
            capture_devs = []
            for dev_id in range(40):
                dev = f'/dev/video{dev_id}'
                if os.path.exists(dev) and self._is_v4l2_capture_device(dev):
                    capture_devs.append(dev_id)
            self.get_logger().info(
                f'[CAM-GUI] V4L2 capture devices: '
                f'{", ".join(f"/dev/video{d}" for d in capture_devs) or "none"}'
            )
            for dev_id in capture_devs:
                try:
                    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            self._direct_cam_cap = cap
                            self._direct_cam_backend = 'opencv'
                            self.get_logger().info(
                                f'[CAM-GUI] Direct camera: OpenCV /dev/video{dev_id}'
                            )
                            threading.Thread(target=self._direct_opencv_loop, daemon=True).start()
                            return
                    cap.release()
                except Exception:
                    continue

        # --- 4. Wait for shared file to appear ---
        self.get_logger().warn(
            '[CAM-GUI] No camera found yet. Waiting for shared frame file...\n'
            '  Run on HOST: ./camera_stream.sh start'
        )
        self._direct_cam_backend = 'waiting'
        threading.Thread(target=self._wait_for_shared_file, daemon=True).start()

    def _wait_for_shared_file(self):
        """Poll for the shared frame file to appear (camera_stream.sh started later)."""
        while not self._ros2_camera_active:
            if os.path.isfile(self.CAMERA_FRAME_FILE):
                self._direct_cam_backend = 'shared-file'
                self.get_logger().info(
                    f'[CAM-GUI] Shared camera frame file appeared: {self.CAMERA_FRAME_FILE}'
                )
                self._shared_file_loop()
                return
            time.sleep(2.0)

    def _shared_file_loop(self):
        """Background thread: read the latest JPEG frame from the shared file."""
        last_mtime = 0.0
        while True:
            if not self.camera_enabled.get() or self._ros2_camera_active:
                time.sleep(0.5)
                continue
            try:
                if not os.path.isfile(self.CAMERA_FRAME_FILE):
                    time.sleep(0.5)
                    continue
                mtime = os.path.getmtime(self.CAMERA_FRAME_FILE)
                if mtime == last_mtime:
                    time.sleep(0.015)  # ~66 FPS polling
                    continue
                last_mtime = mtime
                with open(self.CAMERA_FRAME_FILE, 'rb') as f:
                    jpeg_data = f.read()
                if len(jpeg_data) < 100:
                    continue
                # Decode JPEG
                if CV2_AVAILABLE:
                    frame = cv2.imdecode(
                        np.frombuffer(jpeg_data, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        with self._camera_lock:
                            self.latest_camera_frame = rgb
                else:
                    import io
                    pil_img = PILImage.open(io.BytesIO(jpeg_data))
                    with self._camera_lock:
                        self.latest_camera_frame = np.array(pil_img)
            except Exception:
                time.sleep(0.1)

    def _direct_libcam_loop(self):
        """Background thread: read MJPEG frames from rpicam-vid pipe."""
        buf = b''
        while self._direct_cam_proc and self._direct_cam_proc.poll() is None:
            if not self.camera_enabled.get() or self._ros2_camera_active:
                time.sleep(0.5)
                continue
            try:
                chunk = self._direct_cam_proc.stdout.read(4096)
                if not chunk:
                    break
                buf += chunk
                soi = buf.find(b'\xff\xd8')
                if soi == -1:
                    buf = buf[-2:]
                    continue
                eoi = buf.find(b'\xff\xd9', soi + 2)
                if eoi == -1:
                    if len(buf) > 2 * 1024 * 1024:
                        buf = buf[-4096:]
                    continue
                jpeg_data = buf[soi:eoi + 2]
                buf = buf[eoi + 2:]
                if CV2_AVAILABLE:
                    frame = cv2.imdecode(
                        np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        with self._camera_lock:
                            self.latest_camera_frame = rgb
                else:
                    # Decode with PIL
                    import io
                    pil_img = PILImage.open(io.BytesIO(jpeg_data))
                    with self._camera_lock:
                        self.latest_camera_frame = np.array(pil_img)
            except Exception:
                break

    def _direct_opencv_loop(self):
        """Background thread: read frames from OpenCV V4L2 device."""
        while self._direct_cam_cap and self._direct_cam_cap.isOpened():
            if not self.camera_enabled.get() or self._ros2_camera_active:
                time.sleep(0.5)
                continue
            try:
                ret, bgr = self._direct_cam_cap.read()
                if ret and bgr is not None:
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    with self._camera_lock:
                        self.latest_camera_frame = rgb
            except Exception:
                break

    # ---- Tkinter display refresh (runs on main thread via .after) ----
    def _start_camera_refresh(self):
        """Kick off the periodic Tkinter camera frame update."""
        self._refresh_camera_frame()

    def _refresh_camera_frame(self):
        """Pull the latest frame and display it in the Tkinter Label."""
        if self.camera_label is None or not PIL_AVAILABLE:
            return

        try:
            if self.camera_enabled.get():
                frame = None
                with self._camera_lock:
                    if self.latest_camera_frame is not None:
                        frame = self.latest_camera_frame.copy()

                if frame is not None:
                    # Run object detection overlay (if enabled)
                    frame = self.object_detector.process_frame(frame)

                    # Person-following: move arm to track detected person
                    if self.follow_enabled.get():
                        self._follow_person(frame.shape)

                    # Handshake mode: detect person and trigger sequence
                    if self.handshake_enabled.get() and not self._handshake_in_progress:
                        dets = self.object_detector.detections
                        faces = [d for d in dets if d['class_name'] == 'face']
                        if faces:
                            self._handshake_detect_count += 1
                            if self._handshake_detect_count >= self._handshake_detect_threshold:
                                # Pick the largest face and compute base correction
                                face = max(faces, key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']))
                                h, w = frame.shape[:2]
                                face_cx = (face['x1'] + face['x2']) / 2.0
                                offset_x = (face_cx - w / 2.0) / (w / 2.0)  # -1..+1
                                base_now = float(self.joint_sliders[0].get())
                                self._handshake_base = max(0, min(270, base_now - offset_x * 12.0))
                                self._start_handshake()
                                self._handshake_detect_count = 0
                        else:
                            self._handshake_detect_count = 0

                    # Bartender mode: detect cup and trigger pour sequence
                    if self.bartender_enabled.get() and not self._bartender_in_progress:
                        dets = self.object_detector.detections
                        self._bartender_check_cup(dets)

                    # Update detection info label
                    if self.detect_enabled.get() and self.detect_info_label:
                        # Only update info label if follow mode isn't managing it
                        if not self.follow_enabled.get():
                            dets = self.object_detector.detections
                            if dets:
                                names = set(d['class_name'] for d in dets)
                                self.detect_info_label.config(
                                    text=f"\U0001F50D {len(dets)} detected: {', '.join(names)}"
                                )
                            else:
                                mode = self.object_detector.mode
                                label = 'No cups' if mode == 'cup' else 'No faces'
                                self.detect_info_label.config(text=f"\U0001F50D {label} detected")

                    # Convert to display image efficiently
                    # Frames are typically already 640x480 — skip
                    # expensive PIL resize when no scaling is needed.
                    h, w = frame.shape[:2]
                    if w > 640 or h > 480:
                        # Need to downscale — use cv2 (C-native, ~10x faster
                        # than PIL LANCZOS) when available.
                        if CV2_AVAILABLE:
                            scale = min(640 / w, 480 / h)
                            frame = cv2.resize(
                                frame,
                                (int(w * scale), int(h * scale)),
                                interpolation=cv2.INTER_LINEAR,
                            )
                        else:
                            pil_img = PILImage.fromarray(frame)
                            pil_img.thumbnail((640, 480), PILImage.BILINEAR)
                            self.camera_photo = ImageTk.PhotoImage(image=pil_img)
                            self.camera_label.configure(image=self.camera_photo, text='')
                            # (skip redundant conversion below)
                            frame = None

                    if frame is not None:
                        pil_img = PILImage.fromarray(frame)
                        self.camera_photo = ImageTk.PhotoImage(image=pil_img)
                        self.camera_label.configure(image=self.camera_photo, text='')

                    # FPS counter
                    self._cam_fps_count += 1
                    now = time.time()
                    elapsed = now - self._cam_fps_time
                    if elapsed >= 1.0:
                        fps = self._cam_fps_count / elapsed
                        if self.camera_fps_label:
                            self.camera_fps_label.config(text=f"FPS: {fps:.1f}")
                        self._cam_fps_count = 0
                        self._cam_fps_time = now

                    # Update status
                    if self._ros2_camera_active:
                        src = "ROS2 Topic"
                    elif self._direct_cam_backend:
                        src = f"Direct ({self._direct_cam_backend})"
                    else:
                        src = "Unknown"
                    if self.camera_status_label:
                        self.camera_status_label.config(
                            text=f"● {src}", fg='#00ff00'
                        )
                else:
                    # No frame yet
                    self._show_camera_placeholder("Waiting for camera...")
            else:
                self._show_camera_placeholder("Camera disabled")
                if self.camera_status_label:
                    self.camera_status_label.config(text="⏸ Disabled", fg='#888888')
        except Exception as e:
            self.get_logger().error(f'Camera display error: {e}')

        # Schedule next refresh (~30 FPS for the display)
        self.root.after(16, self._refresh_camera_frame)

    def _show_camera_placeholder(self, text="No Camera Feed"):
        """Show a dark placeholder image with text."""
        if self.camera_label is None or not PIL_AVAILABLE:
            return
        try:
            img = PILImage.new('RGB', (640, 480), color=(26, 26, 26))
            draw = ImageDraw.Draw(img)
            # Simple centered text
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
            except Exception:
                font = ImageFont.load_default()
            bbox = draw.textbbox((0, 0), text, font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            draw.text(((640 - tw) // 2, (480 - th) // 2), text,
                      fill=(100, 100, 100), font=font)
            self.camera_photo = ImageTk.PhotoImage(image=img)
            self.camera_label.configure(image=self.camera_photo, text='')
        except Exception:
            self.camera_label.configure(image='', text=text)

    def _on_camera_toggle(self):
        """Handle camera enable/disable toggle."""
        if self.camera_enabled.get():
            self.get_logger().info('[CAM-GUI] Camera enabled')
            # If no backend was found, retry
            if not self._ros2_camera_active and not self._direct_cam_backend:
                threading.Thread(target=self._init_direct_camera, daemon=True).start()
        else:
            self.get_logger().info('[CAM-GUI] Camera disabled')

    def _on_detect_toggle(self):
        """Handle object detection enable/disable toggle."""
        on = self.detect_enabled.get()
        self.object_detector.toggle(on)
        if on:
            self.get_logger().info('[CAM-GUI] Object detection enabled')
        else:
            self.get_logger().info('[CAM-GUI] Object detection disabled')
            if self.detect_info_label:
                self.detect_info_label.config(text="")

    # ------------------------------------------------------------------ #
    #  Person-Following Mode                                               #
    # ------------------------------------------------------------------ #
    def _on_follow_toggle(self):
        """Handle Follow checkbox toggle."""
        on = self.follow_enabled.get()
        if on:
            # Ensure detector is in face mode
            self.object_detector.set_mode('face')
            # Auto-enable detection when Follow is turned on
            if not self.detect_enabled.get():
                self.detect_enabled.set(True)
                self.object_detector.toggle(True)
            self._follow_lost_count = 0
            self.get_logger().info('[FOLLOW] Face-following mode ENABLED')
            self.info_label.config(
                text="👤 Follow mode ON — arm will track detected face",
                fg='#00ccff'
            )
        else:
            self.get_logger().info('[FOLLOW] Face-following mode DISABLED')
            self.info_label.config(
                text="👤 Follow mode OFF",
                fg='#aaaaaa'
            )

    def _follow_person(self, frame_shape):
        """Track the largest detected face by adjusting base (yaw) and shoulder (pitch).

        Called from _refresh_camera_frame when follow mode is active.

        The camera is mounted on the gripper (eye-in-hand). This means servo
        corrections must account for the fact that moving a joint also moves
        the camera.

        Eye-in-hand direction mapping:
          Base axis — increasing base rotates the whole arm (and camera) LEFT,
            so a face on the RIGHT of frame (offset_x > 0) needs base to
            DECREASE to swing the camera toward it.
          Shoulder axis — increasing shoulder tilts the arm (and camera) DOWN.
            Because the camera rides on the arm, tilting down makes the scene
            shift UP in the image (positive feedback). So a face BELOW centre
            (offset_y > 0) needs shoulder to DECREASE (lift arm up) to bring
            the camera up and centre the face.  This is the OPPOSITE of a
            fixed/base-mounted camera.
        """
        now = time.time()
        if now - self._follow_last_time < self._follow_interval:
            return
        self._follow_last_time = now

        dets = self.object_detector.detections
        # Filter for faces
        faces = [d for d in dets if d['class_name'] == 'face']
        if not faces:
            self._follow_lost_count += 1
            if self._follow_lost_count > 25:  # ~3 s at 8 Hz
                if self.detect_info_label:
                    self.detect_info_label.config(
                        text="👤 Follow: no face detected"
                    )
            return

        self._follow_lost_count = 0

        # Pick the largest face (primary target)
        face = max(faces, key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']))
        bbox_cx = (face['x1'] + face['x2']) / 2.0
        bbox_cy = (face['y1'] + face['y2']) / 2.0

        frame_h, frame_w = frame_shape[:2]

        # Normalised offsets from image centre: [-1, +1]
        offset_x = (bbox_cx - frame_w / 2.0) / (frame_w / 2.0)
        offset_y = (bbox_cy - frame_h / 2.0) / (frame_h / 2.0)

        # ---- Per-axis dead-zone (narrow — we *want* to re-centre) ----
        DEAD_X = 0.05   # ~3 % of half-width
        DEAD_Y = 0.05

        moved = False

        # ---- Base (yaw) — correct horizontal offset ----
        current_base = float(self.joint_sliders[0].get())
        if abs(offset_x) > DEAD_X:
            BASE_GAIN = 12.0  # degrees per unit offset (responsive)
            # Eye-in-hand: face right (offset_x > 0) → decrease base to swing camera right
            base_delta = -offset_x * BASE_GAIN
            new_base = max(0, min(270, current_base + base_delta))
            self.joint_sliders[0].set(new_base)
            self.target_positions[0] = new_base
            moved = True
        else:
            new_base = current_base

        # ---- Shoulder (pitch) — correct vertical offset ----
        current_shoulder = float(self.joint_sliders[1].get())
        if abs(offset_y) > DEAD_Y:
            SHOULDER_GAIN = 8.0
            # Eye-in-hand: face below centre (offset_y > 0) → DECREASE shoulder
            # to lift arm/camera up, centering the face.
            shoulder_delta = -offset_y * SHOULDER_GAIN
            new_shoulder = max(0, min(180, current_shoulder + shoulder_delta))
            self.joint_sliders[1].set(new_shoulder)
            self.target_positions[1] = new_shoulder
            moved = True
        else:
            new_shoulder = current_shoulder

        if moved:
            # Update 3D visualisation
            if self.viz_3d:
                self.viz_3d.set_joint_angles(self.target_positions)
            self.send_command()

        if self.detect_info_label:
            arrow  = "→" if offset_x >  DEAD_X else ("←" if offset_x < -DEAD_X else "·")
            v_arrow = "↓" if offset_y >  DEAD_Y else ("↑" if offset_y < -DEAD_Y else "·")
            status = "centred" if not moved else f"{arrow}{v_arrow}"
            self.detect_info_label.config(
                text=f"👤 Face {status}  base={new_base:.0f}°"
                     f"  shoulder={new_shoulder:.0f}°"
                     f"  off=({offset_x:+.2f},{offset_y:+.2f})"
            )

    # ------------------------------------------------------------------ #
    #  Handshake Mode                                                      #
    # ------------------------------------------------------------------ #

    # Relative offsets from the default (center) pose for each handshake motion.
    # Format: {joint_index: delta_degrees}
    #   0=base, 1=shoulder, 2=elbow, 3=forearm, 4=wrist_rotate, 5=gripper
    #
    # The sequence is:
    #   1. Close gripper (lean forward + grip)
    #   2. Slap right (base rotates right) → return to center
    #   3. Slap left  (base rotates left)  → return to center
    #   4. Knuckle up (shoulder lifts)     → return to center
    #   5. Knuckle down (shoulder drops)   → return to center
    _HS_LEAN_FORWARD = {1: -15.0, 2: +25.0}              # lean elbow toward person (no grip yet)
    _HS_SLAP_RIGHT   = {0: -30.0}                         # base rotates right
    _HS_SLAP_LEFT    = {0: +30.0}                          # base rotates left
    _HS_KNUCKLE_UP   = {1: -25.0}                          # shoulder lifts arm up
    _HS_KNUCKLE_DOWN = {1: +25.0}                          # shoulder drops arm down

    # Ordered sequence: (label, delta_dict)
    # Each motion plays out then returns to center before the next.
    _HANDSHAKE_SEQUENCE = [
        ('slap_right',   _HS_SLAP_RIGHT),
        ('slap_left',    _HS_SLAP_LEFT),
        ('knuckle_up',   _HS_KNUCKLE_UP),
        ('knuckle_down', _HS_KNUCKLE_DOWN),
    ]

    _HANDSHAKE_PAUSE_MS = 1500   # pause between motions (ms)
    _HANDSHAKE_STEPS    = 20     # interpolation steps per motion
    _HANDSHAKE_STEP_MS  = 40     # ms between interpolation steps (~0.8 s per motion)

    def _on_handshake_toggle(self):
        """Handle the 🤝 Handshake checkbox toggle."""
        on = self.handshake_enabled.get()
        if on:
            # Auto-enable camera and detection
            if not self.camera_enabled.get():
                self.camera_enabled.set(True)
                self._on_camera_toggle()
            if not self.detect_enabled.get():
                self.detect_enabled.set(True)
                self.object_detector.toggle(True)
            # Disable follow and visual servo to avoid conflicts
            if self.follow_enabled.get():
                self.follow_enabled.set(False)
                self._on_follow_toggle()
            if self.visual_servo_enabled.get():
                self.visual_servo_enabled.set(False)
                self._on_visual_servo_toggle()

            self._handshake_detect_count = 0
            self._handshake_cancelled = False
            self.get_logger().info('[HANDSHAKE] Mode ENABLED — waiting for person')
            self.info_label.config(
                text="\U0001F91D Handshake mode ON — show your face to start",
                fg='#ffcc00'
            )
        else:
            self.get_logger().info('[HANDSHAKE] Mode DISABLED')
            if self._handshake_in_progress:
                self._handshake_cancelled = True
                self.get_logger().info('[HANDSHAKE] Cancelling in-progress sequence')
            self._handshake_detect_count = 0
            self.info_label.config(
                text="\U0001F91D Handshake mode OFF",
                fg='#aaaaaa'
            )

    def _handshake_apply_delta(self, delta_dict):
        """Build a full 6-joint target by applying delta offsets to the lean pose.

        The lean pose is the captured base angles + forward lean, which is the
        neutral position for all handshake motions.  Each step only changes
        the joints listed in *delta_dict* relative to that lean pose.
        """
        # Joint limits: base 0-270, others 0-180
        limits = [270.0, 180.0, 180.0, 180.0, 180.0, 180.0]
        target = list(self._handshake_lean_pose)
        for idx, delta in delta_dict.items():
            target[idx] = max(0.0, min(limits[idx], target[idx] + delta))
        return target

    def _start_handshake(self):
        """Begin the handshake sequence from the current arm position."""
        self._handshake_in_progress = True
        self._handshake_cancelled = False

        # Snapshot current motor angles as the base pose
        self._handshake_home_pose = [float(self.joint_sliders[i].get()) for i in range(6)]

        # Correct base toward the detected face
        self._handshake_home_pose[0] = self._handshake_base

        # Lean pose = home + forward lean (shoulder down, elbow out) — gripper stays as-is for now
        limits = [245.0, 180.0, 180.0, 180.0, 180.0, 180.0]
        self._handshake_lean_pose = list(self._handshake_home_pose)
        for idx, delta in self._HS_LEAN_FORWARD.items():
            self._handshake_lean_pose[idx] = max(
                0.0, min(limits[idx], self._handshake_lean_pose[idx] + delta)
            )

        # Disable follow/servo during sequence
        if self.follow_enabled.get():
            self.follow_enabled.set(False)
        if self.visual_servo_enabled.get():
            self.visual_servo_enabled.set(False)

        self.get_logger().info('[HANDSHAKE] Person detected — closing gripper')
        self.info_label.config(
            text="\U0001F91D Closing gripper...",
            fg='#ffcc00'
        )

        # Set a moderate speed
        self.speed_value.set(50)
        speed_msg = Float32()
        speed_msg.data = 50.0
        self.speed_pub.publish(speed_msg)

        # Step 1: Close the gripper first (from current position, only change gripper)
        grip_pose = list(self._handshake_home_pose)
        grip_pose[5] = min(180.0, grip_pose[5] + 40.0)  # close gripper
        self._handshake_lean_pose[5] = grip_pose[5]      # remember closed grip for lean pose

        self._handshake_interpolate(
            grip_pose,
            on_complete=lambda: self.root.after(500, self._handshake_lean_then_go)
        )

    def _handshake_lean_then_go(self):
        """Step 2: Lean forward to ready position, then start the 4-motion sequence."""
        if self._handshake_cancelled:
            self._handshake_finish_cancelled()
            return

        self.get_logger().info('[HANDSHAKE] Leaning forward to ready position')
        self.info_label.config(
            text="\U0001F91D Moving to ready position...",
            fg='#ffcc00'
        )
        self._handshake_interpolate(
            self._handshake_lean_pose,
            on_complete=lambda: self.root.after(
                self._HANDSHAKE_PAUSE_MS,
                lambda: self._handshake_step(0)
            )
        )

    def _handshake_step(self, index):
        """Execute step `index` of the handshake sequence.

        Each step: move to the offset pose, pause, return to center, pause,
        then proceed to the next step.
        """
        if self._handshake_cancelled:
            self._handshake_finish_cancelled()
            return

        if index >= len(self._HANDSHAKE_SEQUENCE):
            # All motions done — return to the pose we started from
            self.get_logger().info('[HANDSHAKE] Sequence complete — returning home')
            self.info_label.config(
                text="\U0001F91D Handshake done! Returning home...",
                fg='#00ff00'
            )
            self._handshake_interpolate(
                self._handshake_home_pose,
                on_complete=self._handshake_finish
            )
            return

        label, delta = self._HANDSHAKE_SEQUENCE[index]
        target = self._handshake_apply_delta(delta)
        step_num = index + 1
        total = len(self._HANDSHAKE_SEQUENCE)
        self.get_logger().info(f'[HANDSHAKE] Step {step_num}/{total}: {label}')
        self.info_label.config(
            text=f"\U0001F91D Step {step_num}/{total}: {label.replace('_', ' ')}",
            fg='#ffcc00'
        )

        # Move to offset pose, then return to center, then next step
        self._handshake_interpolate(
            list(target),
            on_complete=lambda: self.root.after(
                self._HANDSHAKE_PAUSE_MS,
                lambda: self._handshake_return_center(index)
            )
        )

    def _handshake_return_center(self, index):
        """Return to center (lean pose) after a handshake motion, then advance."""
        if self._handshake_cancelled:
            self._handshake_finish_cancelled()
            return

        self.info_label.config(
            text="\U0001F91D Returning to center...",
            fg='#ffcc00'
        )
        self._handshake_interpolate(
            self._handshake_lean_pose,
            on_complete=lambda: self.root.after(
                self._HANDSHAKE_PAUSE_MS,
                lambda: self._handshake_step(index + 1)
            )
        )

    def _handshake_interpolate(self, target_angles, on_complete=None):
        """Smoothly interpolate to target_angles, then call on_complete."""
        start_angles = [float(self.joint_sliders[i].get()) for i in range(6)]
        steps = self._HANDSHAKE_STEPS
        delay = self._HANDSHAKE_STEP_MS
        increments = [(t - s) / steps for s, t in zip(start_angles, target_angles)]

        def _step(n):
            if self._handshake_cancelled:
                self._handshake_finish_cancelled()
                return

            if n >= steps:
                # Final: snap to exact target
                for i, angle in enumerate(target_angles):
                    self.joint_sliders[i].set(angle)
                    self.target_positions[i] = angle
                if self.viz_3d:
                    self.viz_3d.set_joint_angles(self.target_positions)
                self.send_command()
                if on_complete:
                    on_complete()
                return

            for i in range(6):
                val = start_angles[i] + increments[i] * n
                self.joint_sliders[i].set(val)
                self.target_positions[i] = val
            self.send_command()

            self.root.after(delay, lambda: _step(n + 1))

        _step(0)

    def _handshake_finish(self):
        """Clean up after a completed handshake sequence."""
        self._handshake_in_progress = False
        self._handshake_detect_count = 0
        self.get_logger().info('[HANDSHAKE] Ready for next person')
        self.info_label.config(
            text="\U0001F91D Handshake complete! Waiting for next person...",
            fg='#00ff00'
        )

    def _handshake_finish_cancelled(self):
        """Clean up after a cancelled handshake — return to home."""
        self.get_logger().info('[HANDSHAKE] Cancelled — returning home')
        self.info_label.config(
            text="\U0001F91D Cancelled — returning home...",
            fg='#ff8800'
        )
        self._handshake_cancelled = False  # prevent re-entry

        def _go_home():
            start = [float(self.joint_sliders[i].get()) for i in range(6)]
            home = getattr(self, '_handshake_home_pose',
                           [90.0, 100.0, 100.0, 80.0, 90.0, 120.0])
            steps = self._HANDSHAKE_STEPS
            inc = [(h - s) / steps for s, h in zip(start, home)]

            def _s(n):
                if n >= steps:
                    for i, a in enumerate(home):
                        self.joint_sliders[i].set(a)
                        self.target_positions[i] = a
                    if self.viz_3d:
                        self.viz_3d.set_joint_angles(self.target_positions)
                    self.send_command()
                    self._handshake_in_progress = False
                    self._handshake_detect_count = 0
                    self.info_label.config(
                        text="\U0001F91D Handshake mode ON — show your face to start",
                        fg='#ffcc00'
                    )
                    return
                for i in range(6):
                    self.joint_sliders[i].set(start[i] + inc[i] * n)
                    self.target_positions[i] = start[i] + inc[i] * n
                self.send_command()
                self.root.after(self._HANDSHAKE_STEP_MS, lambda: _s(n + 1))

            _s(0)

        _go_home()

    # ------------------------------------------------------------------ #
    #  Bartender Mode                                                      #
    # ------------------------------------------------------------------ #

    # Bartender sequence poses (absolute angles — NOT deltas)
    # Sequence: detect cup → reach toward cup → close gripper → lift →
    #           tilt pour → upright → lower → open gripper → home
    _BART_PAUSE_MS   = 600
    _BART_STEPS      = 25
    _BART_STEP_MS    = 18

    def _on_bartender_toggle(self):
        """Handle the 🥃 Bartender checkbox toggle."""
        on = self.bartender_enabled.get()
        if on:
            if not self.object_detector.cup_detection_available:
                self.get_logger().warn('[BARTENDER] No YOLOv8n ONNX model — cannot detect cups')
                self.info_label.config(
                    text="\U0001F943 YOLO model not found — place yolov8n.onnx in workspace",
                    fg='#ff4444'
                )
                self.bartender_enabled.set(False)
                return

            # Auto-enable camera & detection in cup mode
            if not self.camera_enabled.get():
                self.camera_enabled.set(True)
                self._on_camera_toggle()
            if not self.detect_enabled.get():
                self.detect_enabled.set(True)
            self.object_detector.set_mode('cup')
            self.object_detector.toggle(True)

            # Disable conflicting modes
            if self.follow_enabled.get():
                self.follow_enabled.set(False)
                self._on_follow_toggle()
            if self.visual_servo_enabled.get():
                self.visual_servo_enabled.set(False)
                self._on_visual_servo_toggle()
            if self.handshake_enabled.get():
                self.handshake_enabled.set(False)
                self._on_handshake_toggle()

            self._bartender_detect_count = 0
            self._bartender_cancelled = False
            self._bartender_cup = None
            self.get_logger().info('[BARTENDER] Mode ENABLED — looking for cups')
            self.info_label.config(
                text="\U0001F943 Bartender mode ON — place a cup in view",
                fg='#32ff7e'
            )
        else:
            self.get_logger().info('[BARTENDER] Mode DISABLED')
            if self._bartender_in_progress:
                self._bartender_cancelled = True
            self._bartender_detect_count = 0
            self.object_detector.set_mode('face')  # restore face mode
            self.info_label.config(
                text="\U0001F943 Bartender mode OFF",
                fg='#aaaaaa'
            )

    def _bartender_check_cup(self, dets):
        """Check for cup detections and trigger bartender sequence."""
        if self._bartender_in_progress or not self.bartender_enabled.get():
            return

        cups = [d for d in dets if d.get('class_name') in ('cup', 'wine glass', 'bottle')]
        if cups:
            self._bartender_detect_count += 1
            self._bartender_cup = max(cups, key=lambda d: (d['x2'] - d['x1']) * (d['y2'] - d['y1']))
            if self._bartender_detect_count >= self._bartender_detect_threshold:
                self.get_logger().info(f'[BARTENDER] Cup detected! Starting sequence')
                self._start_bartender()
                self._bartender_detect_count = 0
        else:
            self._bartender_detect_count = 0

    def _start_bartender(self):
        """Begin the bartender sequence — reach, grab, lift, pour, return."""
        self._bartender_in_progress = True
        self._bartender_cancelled = False

        # Capture current pose as home
        self._bart_home = [float(self.joint_sliders[i].get()) for i in range(6)]

        # Estimate base angle from cup position in frame
        cup = self._bartender_cup
        if cup and hasattr(self, 'camera_label'):
            frame_w = 640  # camera frame width
            cup_cx = (cup['x1'] + cup['x2']) / 2.0
            offset_x = (cup_cx - frame_w / 2.0) / (frame_w / 2.0)  # -1..+1
            base_now = float(self.joint_sliders[0].get())
            # Eye-in-hand: negative mapping
            self._bart_base = max(0, min(270, base_now - offset_x * 15.0))
        else:
            self._bart_base = float(self.joint_sliders[0].get())

        # Build the sequence of absolute poses
        b = self._bart_base
        self._bart_sequence = [
            # (label, [base, shoulder, elbow, forearm, wrist, gripper])
            ('open_gripper',  [b, self._bart_home[1], self._bart_home[2],
                               self._bart_home[3], self._bart_home[4], 30.0]),
            ('reach_down',    [b, 60.0, 140.0, 90.0, 90.0, 30.0]),
            ('close_gripper', [b, 60.0, 140.0, 90.0, 90.0, 140.0]),
            ('lift_cup',      [b, 100.0, 90.0, 80.0, 90.0, 140.0]),
            ('tilt_pour',     [b, 100.0, 90.0, 80.0, 40.0, 140.0]),
            ('hold_pour',     None),  # pause only
            ('upright',       [b, 100.0, 90.0, 80.0, 90.0, 140.0]),
            ('lower_cup',     [b, 60.0, 140.0, 90.0, 90.0, 140.0]),
            ('release',       [b, 60.0, 140.0, 90.0, 90.0, 30.0]),
            ('home',          self._bart_home),
        ]

        self._bartender_step(0)

    def _bartender_step(self, idx):
        """Execute step `idx` of the bartender sequence."""
        if self._bartender_cancelled:
            self._bartender_finish_cancel()
            return

        if idx >= len(self._bart_sequence):
            self._bartender_finish()
            return

        label, target = self._bart_sequence[idx]
        step_num = idx + 1
        total = len(self._bart_sequence)
        self.get_logger().info(f'[BARTENDER] Step {step_num}/{total}: {label}')
        self.info_label.config(
            text=f"\U0001F943 Step {step_num}/{total}: {label.replace('_', ' ')}",
            fg='#32ff7e'
        )

        if target is None:
            # Pause-only step (e.g. hold pour)
            self.root.after(1500, lambda: self._bartender_step(idx + 1))
            return

        self._bartender_interpolate(
            target,
            on_complete=lambda: self.root.after(
                self._BART_PAUSE_MS,
                lambda: self._bartender_step(idx + 1)
            )
        )

    def _bartender_interpolate(self, target_angles, on_complete=None):
        """Smooth interpolation for bartender moves."""
        start = [float(self.joint_sliders[i].get()) for i in range(6)]
        steps = self._BART_STEPS
        delay = self._BART_STEP_MS
        inc = [(t - s) / steps for s, t in zip(start, target_angles)]

        def _s(n):
            if self._bartender_cancelled:
                self._bartender_finish_cancel()
                return
            if n >= steps:
                for i, a in enumerate(target_angles):
                    self.joint_sliders[i].set(a)
                    self.target_positions[i] = a
                if self.viz_3d:
                    self.viz_3d.set_joint_angles(self.target_positions)
                self.send_command()
                if on_complete:
                    on_complete()
                return
            for i in range(6):
                val = start[i] + inc[i] * n
                self.joint_sliders[i].set(val)
                self.target_positions[i] = val
            self.send_command()
            self.root.after(delay, lambda: _s(n + 1))

        _s(0)

    def _bartender_finish(self):
        """Clean up after completed bartender sequence."""
        self._bartender_in_progress = False
        self._bartender_detect_count = 0
        self._bartender_cup = None
        self.get_logger().info('[BARTENDER] Sequence complete — ready for next cup')
        self.info_label.config(
            text="\U0001F943 Done! Place another cup to pour again...",
            fg='#32ff7e'
        )

    def _bartender_finish_cancel(self):
        """Clean up after cancelled bartender — go home."""
        self.get_logger().info('[BARTENDER] Cancelled — returning home')
        self.info_label.config(text="\U0001F943 Cancelled", fg='#ff8800')
        self._bartender_cancelled = False
        home = getattr(self, '_bart_home', [90.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        self._bartender_interpolate(home, on_complete=lambda: setattr(self, '_bartender_in_progress', False))

    # ------------------------------------------------------------------ #
    #  Eye-in-Hand Visual Servoing                                         #
    # ------------------------------------------------------------------ #
    def _on_visual_servo_toggle(self):
        """Handle the 👁 Servo checkbox."""
        on = self.visual_servo_enabled.get()
        if on:
            # Initialise the flying target from current FK position
            fk = self.forward_kinematics(
                self.target_positions[0], self.target_positions[1],
                self.target_positions[2], self.target_positions[3],
                self.target_positions[4],
            )
            self._vs_target = [fk[0], fk[1], fk[2]]
            self._vs_last_error = Vector3()

            # Auto-enable auto-follow so send_command() fires immediately
            if not self.auto_follow_active.get():
                self.toggle_auto_follow()

            self.get_logger().info(
                f'[VS] Visual servoing ENABLED — initial target '
                f'({fk[0]:.3f}, {fk[1]:.3f}, {fk[2]:.3f})'
            )
            self.info_label.config(
                text="👁 Visual servoing ON — arm follows face via /vision/error",
                fg='#ff66ff'
            )
            # Kick off the periodic servoing loop
            self.root.after(int(self._vs_interval * 1000), self._vs_tick)
        else:
            self._vs_target = None
            self.get_logger().info('[VS] Visual servoing DISABLED')
            self.info_label.config(text="👁 Visual servoing OFF", fg='#aaaaaa')

    def _vision_error_callback(self, msg: Vector3):
        """ROS2 callback: store the latest error from /vision/error."""
        self._vs_last_error = msg
        self._vs_error_fresh = True

    def _get_ee_rotation_matrix(self):
        """Return the 3×3 rotation matrix R_ee (base ← camera frame).

        The camera is mounted on the end-effector (eye-in-hand), so its
        optical axis is aligned with the gripper.  We reconstruct R_ee
        from the current joint angles using FK conventions:

        * **z_ee** (camera optical axis) = direction the gripper points.
        * **x_ee** = perpendicular in the base XY plane (→ camera "right").
        * **y_ee** = z_ee × x_ee (→ camera "up").

        All vectors are expressed in the **base frame**.
        """
        base = self.target_positions[0]
        shoulder = self.target_positions[1]
        elbow = self.target_positions[2]
        forearm = self.target_positions[3]

        base_rad = math.radians(base - 90.0)
        shoulder_rad = math.radians(90.0 - shoulder)
        elbow_rad = math.radians(90.0 - elbow)
        forearm_rad = math.radians(90.0 - forearm)   # INVERTED (matches FK)

        # Cumulative pitch angle of the gripper (from vertical)
        pitch = shoulder_rad + elbow_rad + forearm_rad

        # z_ee: direction the gripper/camera is pointing (in base frame)
        #   pitch = 0 → straight up  (sin=0, cos=1)
        #   pitch = π/2 → horizontal  (sin=1, cos=0)
        z_ee = np.array([
            math.sin(pitch) * math.cos(base_rad),
            math.sin(pitch) * math.sin(base_rad),
            math.cos(pitch),
        ])

        # x_ee: "camera right" — perpendicular to z_ee in the base XY plane
        # When the camera looks along z_ee, its "right" is the base-frame
        # direction perpendicular to the radial arm direction in XY.
        x_ee = np.array([
            -math.sin(base_rad),
             math.cos(base_rad),
             0.0,
        ])

        # y_ee: "camera up" = z_ee × x_ee  (right-hand rule)
        y_ee = np.cross(z_ee, x_ee)
        y_norm = np.linalg.norm(y_ee)
        if y_norm > 1e-6:
            y_ee /= y_norm
        else:
            # Degenerate (looking straight up/down) — use world Y as fallback
            y_ee = np.array([0.0, 1.0, 0.0])

        # R_ee columns: [x_ee | y_ee | z_ee]
        R = np.column_stack([x_ee, y_ee, z_ee])
        return R

    def _vs_tick(self):
        """Periodic visual-servoing update (runs on the Tkinter thread).

        1. Read the latest /vision/error.
        2. Transform camera-frame error → base-frame correction.
        3. Cap the step size so the target only nudges a tiny amount.
        4. Try IK — if it fails, roll back the target (don't let it drift
           into unreachable space).
        5. Reschedule itself.
        """
        if not self.visual_servo_enabled.get() or self._vs_target is None:
            return  # stop the loop

        err = self._vs_last_error

        # Only act when a fresh error has arrived from the vision node.
        # Without a running vision_error_publisher, the arm simply holds.
        if not self._vs_error_fresh:
            self.root.after(int(self._vs_interval * 1000), self._vs_tick)
            return
        self._vs_error_fresh = False

        # Dead-zone — hold position if the error is tiny
        if abs(err.x) < 0.08 and abs(err.y) < 0.08 and abs(err.z) < 0.008:
            self.root.after(int(self._vs_interval * 1000), self._vs_tick)
            return

        # Build the error vector in Camera Frame: (right, up, forward)
        # Negate x/y so the arm chases the face (moves toward the error)
        # rather than fleeing from it.
        V_cam = np.array([
            -err.x * self._vs_gain_xy,   # camera-right (negated to chase)
            -err.y * self._vs_gain_xy,   # camera-up    (negated to chase)
             err.z * self._vs_gain_z,    # camera-forward (depth — keep sign)
        ])

        # Transform to Base Frame:  V_base = R_ee @ V_cam
        R_ee = self._get_ee_rotation_matrix()
        V_base = R_ee @ V_cam

        # ---- Velocity cap: limit the step to _vs_max_step metres ----
        step_len = float(np.linalg.norm(V_base))
        if step_len > self._vs_max_step:
            V_base = V_base * (self._vs_max_step / step_len)

        # Propose a new target
        prev_target = list(self._vs_target)  # keep copy for rollback
        new_x = self._vs_target[0] + V_base[0]
        new_y = self._vs_target[1] + V_base[1]
        new_z = self._vs_target[2] + V_base[2]

        # Clamp to reachable workspace (metres)
        max_r = 0.40   # max radial reach
        r_xy = math.sqrt(new_x**2 + new_y**2)
        if r_xy > max_r:
            scale = max_r / r_xy
            new_x *= scale
            new_y *= scale
        new_z = max(0.0, min(0.45, new_z))

        # Try IK at the proposed target
        trial_ik = self.custom_ik_solver.solve_ik_to_point(
            new_x, new_y, new_z, roll=0, seed_state={
                'base': self.target_positions[0],
                'shoulder': self.target_positions[1],
                'elbow': self.target_positions[2],
                'forearm': self.target_positions[3],
                'wrist': self.target_positions[4],
            }
        )

        if trial_ik is None:
            # Unreachable — roll back, don't drift outward
            self._vs_target = prev_target
            self.root.after(int(self._vs_interval * 1000), self._vs_tick)
            return

        # IK succeeded — accept the new target
        self._vs_target = [new_x, new_y, new_z]

        # Drive the arm via the existing on_3d_target_move pipeline
        self.on_3d_target_move(new_x, new_y, new_z)

        # Update 3D viz target marker
        if self.viz_3d:
            self.viz_3d.set_target(new_x, new_y, new_z)

        self.info_label.config(
            text=f"👁 VS target ({new_x*1000:.0f}, {new_y*1000:.0f}, "
                 f"{new_z*1000:.0f}) mm  err=({err.x:.2f},{err.y:.2f},{err.z:.3f})",
            fg='#ff66ff'
        )

        # Reschedule
        self.root.after(int(self._vs_interval * 1000), self._vs_tick)

    def _camera_snapshot(self):
        """Save the current camera frame to a file."""
        frame = None
        with self._camera_lock:
            if self.latest_camera_frame is not None:
                frame = self.latest_camera_frame.copy()
        if frame is None:
            messagebox.showwarning("Snapshot", "No camera frame available.")
            return
        self._camera_snapshot_count += 1
        filename = f'/tmp/jarvis_snapshot_{self._camera_snapshot_count:04d}.png'
        try:
            pil_img = PILImage.fromarray(frame)
            pil_img.save(filename)
            self.get_logger().info(f'Snapshot saved: {filename}')
            self.info_label.config(text=f"📸 Snapshot saved: {filename}", fg='#00ff00')
        except Exception as e:
            self.get_logger().error(f'Snapshot failed: {e}')
            messagebox.showerror("Snapshot", f"Failed to save snapshot:\n{e}")

    def _cleanup_camera(self):
        """Release direct camera resources."""
        if self._direct_cam_cap is not None:
            try:
                self._direct_cam_cap.release()
            except Exception:
                pass
            self._direct_cam_cap = None
        if self._direct_cam_proc is not None:
            try:
                self._direct_cam_proc.terminate()
                self._direct_cam_proc.wait(timeout=3)
            except Exception:
                try:
                    self._direct_cam_proc.kill()
                except Exception:
                    pass
            self._direct_cam_proc = None

    def on_speed_change(self, value):
        """Handle speed slider change"""
        speed = int(float(value))
        self.speed_label.config(text=f"Speed: {speed}%")
        
        # Publish speed to servo_node
        msg = Float32()
        msg.data = float(speed)
        self.speed_pub.publish(msg)
        self.get_logger().debug(f"Speed set to: {speed}")
    
    def on_slider_change(self, joint_idx, value):
        """Handle slider value change"""
        self.target_positions[joint_idx] = float(value)
        self.joint_labels[joint_idx].config(text=f"{float(value):.1f}°")
        
        # Auto-send if real-time mode is enabled
        if self.realtime_mode.get():
            self.schedule_update()
    
    def on_cartesian_slider_change(self, axis, value):
        """Handle cartesian slider change"""
        # Update entry and label
        self.cartesian_entries[axis].delete(0, tk.END)
        self.cartesian_entries[axis].insert(0, str(int(float(value))))
        self.cartesian_labels[axis].config(text=f"{int(float(value))} mm")
        
        # Sync with 3D visualization
        if self.viz_3d:
            x_mm = float(self.cartesian_entries['X'].get())
            y_mm = float(self.cartesian_entries['Y'].get())
            z_mm = float(self.cartesian_entries['Z'].get())
            self.viz_3d.set_target(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
        
        # Solve IK if real-time mode enabled
        if self.realtime_mode.get():
            self.schedule_cartesian_update()
    
    def on_cartesian_entry_change(self, axis):
        """Handle cartesian entry change"""
        try:
            value = float(self.cartesian_entries[axis].get())
            # Update slider and label
            self.cartesian_sliders[axis].set(value)
            self.cartesian_labels[axis].config(text=f"{int(value)} mm")
            
            # Sync with 3D visualization
            if self.viz_3d:
                x_mm = float(self.cartesian_entries['X'].get())
                y_mm = float(self.cartesian_entries['Y'].get())
                z_mm = float(self.cartesian_entries['Z'].get())
                self.viz_3d.set_target(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
            
            # Solve IK if real-time mode enabled
            if self.realtime_mode.get():
                self.schedule_cartesian_update()
        except ValueError:
            pass  # Ignore invalid input
    
    def solve_ik(self, x, y, z, pitch=0.0, roll=0.0, gripper_angle=None):
        """
        Solve inverse kinematics using MoveIt (preferred) or custom solver (fallback).
        
        Args:
            x, y, z: Target position in meters
            pitch, roll: End effector orientation in radians
            gripper_angle: Gripper opening angle in degrees
            
        Returns:
            Dict of joint angles in degrees, or None if IK fails
        """
        joint_angles = None
        
        # Build current seed state from GUI positions
        seed_state = {
            'base': self.target_positions[0],
            'shoulder': self.target_positions[1],
            'elbow': self.target_positions[2],
            'wrist': self.target_positions[3],
            'wrist_rotate': self.target_positions[4],
        }
        
        # Try MoveIt first if available
        if self.use_moveit and self.moveit_ik and self.moveit_ik.is_available():
            self.get_logger().debug("Using MoveIt IK solver...")
            joint_angles = self.moveit_ik.solve_ik(x, y, z, pitch, roll, gripper_angle, seed_state)
            if joint_angles:
                self.info_label.config(text="[MoveIt IK] ", fg='#00ff88')
        
        # Fall back to custom solver
        if joint_angles is None:
            self.get_logger().debug("Using custom geometric IK solver...")
            joint_angles = self.custom_ik_solver.solve_ik(x, y, z, pitch, roll, gripper_angle, seed_state)
            if joint_angles:
                self.get_logger().info("Using custom geometric IK (MoveIt unavailable)")
        
        return joint_angles
    
    def solve_ik_to_point(self, x, y, z, gripper_angle=None):
        """Solve IK so the gripper tip lands on (x, y, z).

        Automatically searches for the best pitch angle.
        Falls back to the old pitch-guess method if the search fails.
        """
        seed_state = {
            'base': self.target_positions[0],
            'shoulder': self.target_positions[1],
            'elbow': self.target_positions[2],
            'forearm': self.target_positions[3],
            'wrist': self.target_positions[4],
        }
        result = self.custom_ik_solver.solve_ik_to_point(
            x, y, z, roll=0, gripper_angle=gripper_angle, seed_state=seed_state
        )
        return result

    def forward_kinematics(self, base, shoulder, elbow, forearm, wrist):
        """
        Calculate end effector position using MoveIt (TF) or custom FK.
        
        Args:
            Joint angles in degrees (GUI convention: 90° = home)
            
        Returns:
            Tuple of (x, y, z, pitch, roll)
        """
        # Try MoveIt/TF first if available
        if self.use_moveit and self.moveit_ik:
            return self.moveit_ik.forward_kinematics(base, shoulder, elbow, forearm, wrist)
        
        # Fall back to custom solver
        return self.custom_ik_solver.forward_kinematics(base, shoulder, elbow, forearm, wrist)
    
    def toggle_auto_follow(self):
        """Toggle the Start/Stop auto-follow mode for cartesian tracking."""
        active = not self.auto_follow_active.get()
        self.auto_follow_active.set(active)
        
        if active:
            # Switch 3D view to isometric
            if self.viz_3d:
                self.viz_3d.set_view(45, 25)
                self.viz_3d.set_auto_follow(True)
            
            self.start_stop_btn.config(
                text="⏹  Stop",
                bg='#cc0000',
                activebackground='#ff2222'
            )
            self.info_label.config(
                text="⚡ Auto-follow ACTIVE — drag the target, arm follows in real-time",
                fg='#ffaa00'
            )
            self.get_logger().info('Auto-follow mode STARTED')
        else:
            if self.viz_3d:
                self.viz_3d.set_auto_follow(False)
            
            self.start_stop_btn.config(
                text="▶  Start",
                bg='#006600',
                activebackground='#008800'
            )
            self.info_label.config(
                text="Auto-follow stopped. Drag target and press Start to resume.",
                fg='#aaaaaa'
            )
            self.get_logger().info('Auto-follow mode STOPPED')

    # Maximum allowed per-joint change (degrees) in a single cartesian/IK step.
    # If ANY motor would jump more than this, the pose is rejected to protect
    # the hardware from sudden, potentially damaging movements.
    MAX_JOINT_DELTA_DEG = 35.0

    def _check_joint_delta_safe(self, target_angles, label="IK"):
        """Return True if every joint's change is within MAX_JOINT_DELTA_DEG.

        Args:
            target_angles: list of 6 target angles (degrees, 0-180)
            label: short tag for log messages

        Returns:
            True  – move is safe to execute
            False – at least one joint exceeds the threshold
        """
        for i, target in enumerate(target_angles):
            # Skip gripper (index 5) — it is independent of arm position
            # and should never block a cartesian/IK move.
            if i == 5:
                continue
            current = float(self.joint_sliders[i].get())
            delta = abs(target - current)
            if delta > self.MAX_JOINT_DELTA_DEG:
                self.get_logger().warn(
                    f'[{label}] Joint {i+1} delta {delta:.1f}° exceeds '
                    f'{self.MAX_JOINT_DELTA_DEG}° limit (cur={current:.1f}° → '
                    f'tgt={target:.1f}°). Move BLOCKED.'
                )
                self.info_label.config(
                    text=f"⛔ Blocked: Joint {i+1} would jump {delta:.0f}° "
                         f"(max {self.MAX_JOINT_DELTA_DEG:.0f}°)",
                    fg='#ff4444'
                )
                return False
        return True

    def on_3d_target_move(self, x, y, z):
        """
        Callback when target is dragged in 3D visualization (auto-follow mode)
        or when cartesian movement is triggered.
        Solves IK and moves the arm to the target position.
        In auto-follow mode, uses direct (non-interpolated) movement for speed.
        
        Args:
            x, y, z: Target position in meters
        """
        # Convert to millimeters for the UI
        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000
        
        # Update cartesian sliders and entries
        self.cartesian_sliders['X'].set(x_mm)
        self.cartesian_sliders['Y'].set(y_mm)
        self.cartesian_sliders['Z'].set(z_mm)
        
        self.cartesian_entries['X'].delete(0, tk.END)
        self.cartesian_entries['X'].insert(0, str(int(x_mm)))
        self.cartesian_entries['Y'].delete(0, tk.END)
        self.cartesian_entries['Y'].insert(0, str(int(y_mm)))
        self.cartesian_entries['Z'].delete(0, tk.END)
        self.cartesian_entries['Z'].insert(0, str(int(z_mm)))
        
        self.cartesian_labels['X'].config(text=f"{int(x_mm)} mm")
        self.cartesian_labels['Y'].config(text=f"{int(y_mm)} mm")
        self.cartesian_labels['Z'].config(text=f"{int(z_mm)} mm")
        
        # Solve IK — automatically finds the best pitch so the gripper
        # tip lands exactly on the target point
        joint_angles = self.solve_ik_to_point(x, y, z)
        
        if joint_angles is None:
            self.info_label.config(
                text=f"⚠ Position ({int(x_mm)}, {int(y_mm)}, {int(z_mm)}) mm is unreachable",
                fg='#ff8800'
            )
            return
        
        # Map IK solver's joint names to our joint order
        ik_to_gui_mapping = {
            'base': 0,
            'shoulder': 1,
            'elbow': 2,
            'forearm': 3,
            'wrist': 4,
            'gripper': 5
        }
        
        target_angles = [0.0] * 6
        for ik_joint, gui_idx in ik_to_gui_mapping.items():
            angle = joint_angles[ik_joint]
            hi = 270 if gui_idx == 0 else 180
            angle = max(0, min(hi, angle))
            target_angles[gui_idx] = angle

        # Keep whatever gripper angle the user has set — IK only
        # controls the arm joints, not the gripper.
        target_angles[5] = float(self.joint_sliders[5].get())

        # --- Safety: reject if any joint would jump too far ---
        if not self._check_joint_delta_safe(target_angles, label='3D-IK'):
            return

        # Publish target marker for RViz
        self.publish_target_marker(x, y, z)
        
        # In auto-follow mode: direct move (no interpolation) for real-time response
        if self.auto_follow_active.get():
            for i, angle in enumerate(target_angles):
                self.joint_sliders[i].set(angle)
                self.target_positions[i] = angle
            
            # Update 3D visualization
            if self.viz_3d:
                joint_angles_dict = {
                    'base': target_angles[0],
                    'shoulder': target_angles[1],
                    'elbow': target_angles[2],
                    'forearm': target_angles[3],
                    'wrist': target_angles[4],
                    'gripper': target_angles[5]
                }
                self.viz_3d.update_from_ik_solution(joint_angles_dict)
            
            # Send immediately
            self.send_command()
            self.info_label.config(
                text=f"⚡ Tracking ({int(x_mm)}, {int(y_mm)}, {int(z_mm)}) mm",
                fg='#ffaa00'
            )
        else:
            # Normal mode: smooth interpolation
            self.info_label.config(
                text=f"→ Moving to ({int(x_mm)}, {int(y_mm)}, {int(z_mm)}) mm...",
                fg='#00aaff'
            )
            self.interpolate_to_target(target_angles)

    def interpolate_to_target(self, target_angles, steps=30, delay_ms=50):
        """
        Smoothly interpolate from current position to target angles.
        
        Args:
            target_angles: List of target joint angles (degrees)
            steps: Number of interpolation steps
            delay_ms: Delay between steps in milliseconds
        """
        # Get current positions
        start_angles = self.target_positions.copy()
        
        # Calculate increments for each joint
        increments = [(target - start) / steps for start, target in zip(start_angles, target_angles)]
        
        def step_interpolation(current_step):
            if current_step >= steps:
                # Final step - set exact target
                for i, angle in enumerate(target_angles):
                    self.joint_sliders[i].set(angle)
                    self.target_positions[i] = angle
                
                # Update 3D visualization
                if self.viz_3d:
                    joint_angles_dict = {
                        'base': target_angles[0],
                        'shoulder': target_angles[1],
                        'elbow': target_angles[2],
                        'forearm': target_angles[3],
                        'wrist': target_angles[4],
                        'gripper': target_angles[5]
                    }
                    self.viz_3d.update_from_ik_solution(joint_angles_dict)
                
                # Send final command
                self.send_command()
                
                self.info_label.config(
                    text="✓ Movement complete",
                    fg='#00ff00'
                )
                return
            
            # Calculate intermediate angles
            for i in range(6):
                intermediate = start_angles[i] + increments[i] * current_step
                self.joint_sliders[i].set(intermediate)
                self.target_positions[i] = intermediate
            
            # Send intermediate command
            self.send_command()
            
            # Update 3D visualization
            if self.viz_3d:
                joint_angles_dict = {
                    'base': self.target_positions[0],
                    'shoulder': self.target_positions[1],
                    'elbow': self.target_positions[2],
                    'forearm': self.target_positions[3],
                    'wrist': self.target_positions[4],
                    'gripper': self.target_positions[5]
                }
                self.viz_3d.update_from_ik_solution(joint_angles_dict)
            
            # Schedule next step
            self.root.after(delay_ms, lambda: step_interpolation(current_step + 1))
        
        # Start interpolation
        step_interpolation(0)

    def schedule_update(self):
        """Schedule a robot update with rate limiting"""
        if not self.update_pending:
            self.update_pending = True
            self.root.after(int(self.update_interval * 1000), self.do_update)
    
    def do_update(self):
        """Execute the robot update"""
        self.update_pending = False
        self.send_command()
    
    def schedule_cartesian_update(self):
        """Schedule a cartesian IK update with rate limiting"""
        if not self.cartesian_update_pending:
            self.cartesian_update_pending = True
            self.root.after(int(self.update_interval * 1000), self.do_cartesian_update)
    
    def do_cartesian_update(self):
        """Execute the cartesian IK update"""
        self.cartesian_update_pending = False
        self.apply_cartesian_silent()
    
    def apply_preset(self, angles):
        """Apply preset joint angles"""
        for i, angle in enumerate(angles):
            self.joint_sliders[i].set(angle)
            self.target_positions[i] = float(angle)
        
        # Update 3D visualization
        if self.viz_3d:
            self.viz_3d.set_joint_angles(self.target_positions)
        
        self.info_label.config(text=f"Preset applied. Click 'Send to Robot' to execute.")
    
    def send_command(self):
        """Publish joint commands to ROS2"""
        try:
            # ALWAYS send speed first before any movement
            # This ensures cartesian/IK movements also respect the speed slider
            speed_msg = Float32()
            speed_msg.data = float(self.speed_value.get())
            self.speed_pub.publish(speed_msg)
            
            # Send GUI values directly to hardware (no offset needed)
            # GUI 90° = Servo 90° = Home/upright position
            # Ensure all values are native Python floats (not numpy floats)
            hardware_positions = [float(pos) for pos in self.target_positions]
            
            # Publish as Float32MultiArray for servo control
            msg = Float32MultiArray()
            msg.data = hardware_positions
            self.joint_pub.publish(msg)
            
            # For RViz visualization: URDF joints expect 0 radians as the "home" position
            # So we need to convert: GUI 90° should display as 0 radians in RViz
            # This means: radians = (GUI_degrees - 90) * pi/180
            # Some joints need inversion based on URDF joint axis orientation
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            # Publish all 6 joints including gripper_base for complete visualization
            joint_state_msg.name = self.rviz_joint_names
            
            # Build joint positions for RViz (all 6 joints)
            # The URDF has complex joint orientations, we need to match them:
            # - base: normal (positive = counter-clockwise when viewed from above)
            # - shoulder: inverted (positive in GUI = negative in URDF to lift arm up)
            # - elbow: inverted (positive in GUI = negative in URDF to bend arm)
            # - wrist (forearm): NEGATED to match hardware direction
            # - wrist_rotate: normal
            # - gripper_base: normal (controls gripper fingers)
            rviz_positions = []
            for i, pos in enumerate(self.target_positions):  # All 6 joints
                if i == 1:  # Shoulder - invert for correct up/down motion
                    rviz_positions.append(float(math.radians(90.0 - pos)))
                elif i == 2:  # Elbow joint - invert for correct bend direction
                    rviz_positions.append(float(math.radians(90.0 - pos)))
                elif i == 3:  # Wrist (forearm pitch) - normal with negation to match hardware
                    rviz_positions.append(float(-math.radians(pos - 90.0)))
                else:
                    # Normal conversion: 90° GUI = 0 rad RViz
                    rviz_positions.append(float(math.radians(pos - 90.0)))
            
            joint_state_msg.position = rviz_positions
            self.joint_state_pub.publish(joint_state_msg)
            
            # Publish green marker at actual gripper position in RViz
            self.publish_gripper_marker()
            
            current_speed = self.speed_value.get()
            self.info_label.config(
                text=f"✓ Sent at {current_speed}% speed: {[f'{x:.1f}°' for x in hardware_positions]}",
                fg='#00ff00'
            )
            self.get_logger().info(f"Published joint commands at speed {current_speed}: {hardware_positions}")
        except Exception as e:
            self.info_label.config(
                text=f"✗ Error sending command: {str(e)}",
                fg='#ff0000'
            )
            self.get_logger().error(f"Failed to publish: {str(e)}")
    
    def emergency_stop(self):
        """Toggle emergency stop - disables all servo outputs via PCA9685 OE pin"""
        try:
            self.estop_active = not self.estop_active
            
            # Publish emergency stop state
            msg = Bool()
            msg.data = self.estop_active
            self.estop_pub.publish(msg)
            Elbow
            if self.estop_active:
                self.info_label.config(text="⛔ EMERGENCY STOP ACTIVE - All outputs DISABLED!", fg='#ff0000')
                self.get_logger().warning("Emergency stop ACTIVATED - OE pin HIGH")
                Elbow
                messagebox.showwarning(
                    "Emergency Stop ACTIVATED",
                    "All servo outputs DISABLED!\n\nPress Emergency Stop again to release."
                )
            else:
                self.info_label.config(text="✓ Emergency stop released - Outputs enabled", fg='#00ff00')
                self.get_logger().info("Emergency stop RELEASED - OE pin LOW")
                
                messagebox.showinfo(
                    "Emergency Stop Released",
                    "Servo outputs re-enabled.\nYou can now control the arm."
                )
        except Exception as e:
            self.get_logger().error(f"Emergency stop failed: {str(e)}")
    
    def apply_cartesian(self):
        """Apply cartesian coordinates using IK solver (with user feedback)"""
        self._apply_cartesian_internal(show_feedback=True)
    
    def apply_cartesian_silent(self):
        """Apply cartesian coordinates using IK solver (silent, for real-time mode)"""
        self._apply_cartesian_internal(show_feedback=False)
    
    def _apply_cartesian_internal(self, show_feedback=True):
        """Internal method to apply cartesian coordinates using IK solver"""
        try:
            # Get X, Y, Z values in millimeters
            x_mm = float(self.cartesian_entries['X'].get())
            y_mm = float(self.cartesian_entries['Y'].get())
            z_mm = float(self.cartesian_entries['Z'].get())
            
            # Convert millimeters to meters for IK solver
            x_m = x_mm / 1000.0
            y_m = y_mm / 1000.0
            z_m = z_mm / 1000.0
            
            if show_feedback:
                self.info_label.config(
                    text=f"⚙ Calculating IK for position: X={x_mm}, Y={y_mm}, Z={z_mm} mm...",
                    fg='#ffaa00'
                )
                self.root.update()
            
            # Solve IK — automatically finds the best pitch so the
            # gripper tip lands exactly on the target point
            joint_angles = self.solve_ik_to_point(x_m, y_m, z_m)
            
            if joint_angles is None:
                if show_feedback:
                    messagebox.showerror(
                        "Unreachable Position",
                        f"Target position ({x_mm}, {y_mm}, {z_mm}) mm is out of reach!\n\n"
                        "Try a position closer to the robot."
                    )
                self.info_label.config(
                    text=f"✗ Position unreachable: X={x_mm}, Y={y_mm}, Z={z_mm} mm",
                    fg='#ff0000'
                )
                return
            
            # IK solver returns angles with its own calibration
            # Map IK solver's joint names to our joint order
            ik_to_gui_mapping = {
                'base': 0,      # J1
                'shoulder': 1,  # J2
                'elbow': 2,     # J3
                'forearm': 3,   # J4 (wrist pitch in our model)
                'wrist': 4,     # J5 (wrist roll in our model)
                'gripper': 5    # J6
            }
            
            # Build target angle list and clamp to servo limits
            target_angles = [0.0] * 6
            for ik_joint, gui_idx in ik_to_gui_mapping.items():
                angle = joint_angles[ik_joint]
                hi = 270 if gui_idx == 0 else 180
                angle = max(0, min(hi, angle))
                target_angles[gui_idx] = angle

            # Keep whatever gripper angle the user has set — IK only
            # controls the arm joints, not the gripper.
            target_angles[5] = float(self.joint_sliders[5].get())

            # --- Safety: reject if any joint would jump too far ---
            if not self._check_joint_delta_safe(target_angles, label='Cart-IK'):
                if show_feedback:
                    messagebox.showwarning(
                        "Movement Too Large",
                        f"One or more joints would move more than "
                        f"{self.MAX_JOINT_DELTA_DEG:.0f}° in a single step.\n\n"
                        "Move the arm closer to the target first using "
                        "the joint sliders, then try again."
                    )
                return

            # Update sliders with IK solution
            for gui_idx, angle in enumerate(target_angles):
                self.joint_sliders[gui_idx].set(angle)
                self.target_positions[gui_idx] = angle
            
            # Publish visualization marker at target position
            self.publish_target_marker(x_m, y_m, z_m)
            
            if show_feedback:
                self.info_label.config(
                    text=f"✓ IK solved! Position: X={x_mm}, Y={y_mm}, Z={z_mm} mm. Click 'Send to Robot'",
                    fg='#00ff00'
                )
                
                self.get_logger().info(f"IK solution: {joint_angles}")
                
                messagebox.showinfo(
                    "IK Solution Found",
                    f"Position: ({x_mm}, {y_mm}, {z_mm}) mm\n\n"
                    "Joint angles calculated!\n"
                    "Click 'Send to Robot' to execute."
                )
            else:
                # In real-time mode, show compact status and auto-send
                self.info_label.config(
                    text=f"⚡ Real-time IK: X={x_mm}, Y={y_mm}, Z={z_mm} mm",
                    fg='#ffaa00'
                )
                # Auto-send in real-time mode
                self.send_command()
            
        except ValueError:
            if show_feedback:
                messagebox.showerror(
                    "Invalid Input",
                    "Please enter valid numbers for X, Y, Z coordinates in millimeters."
                )
            self.info_label.config(text="✗ Invalid cartesian coordinates", fg='#ff0000')
        except Exception as e:
            if show_feedback:
                messagebox.showerror("IK Error", f"Failed to solve IK:\n{str(e)}")
            self.info_label.config(text=f"✗ IK error: {str(e)}", fg='#ff0000')
            self.get_logger().error(f"IK solver error: {str(e)}")
    
    def joint_state_callback(self, msg):
        """Callback for joint state updates from ROS2"""
        try:
            # Update current positions from feedback (if needed for monitoring)
            if len(msg.position) >= 6:
                # Could update GUI display with actual robot positions here
                pass
        except Exception as e:
            self.get_logger().error(f"Joint state callback error: {str(e)}")
    
    def publish_target_marker(self, x, y, z):
        """Publish a visualization marker at the target position in RViz"""
        # The URDF base joint has an initial rotation of 2.23697 rad (~128°) around Z
        # We need to rotate our coordinates to match the URDF frame
        # Note: The URDF also has an XY offset of (0.0284, -0.00137) which we ignore for now
        import math as m
        urdf_base_yaw = 2.23697  # From URDF base joint rpy
        
        # Rotate coordinates by -urdf_base_yaw to align with URDF frame
        cos_yaw = m.cos(-urdf_base_yaw)
        sin_yaw = m.sin(-urdf_base_yaw)
        x_rviz = x * cos_yaw - y * sin_yaw
        y_rviz = x * sin_yaw + y * cos_yaw
        z_rviz = z
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (in meters) - rotated to match URDF frame
        marker.pose.position.x = x_rviz
        marker.pose.position.y = y_rviz
        marker.pose.position.z = z_rviz
        marker.pose.orientation.w = 1.0
        
        # Scale (sphere diameter in meters)
        marker.scale.x = 0.02  # 20mm diameter sphere
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        # Color (orange for target)
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Lifetime (0 = forever until replaced)
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        self.marker_pub.publish(marker)
        self.get_logger().debug(f"Published target marker at ({x:.3f}, {y:.3f}, {z:.3f})")
    
    def publish_gripper_marker(self):
        """Publish a visualization marker at the actual gripper position in RViz"""
        # Use forward kinematics to calculate actual gripper position
        base = self.target_positions[0]
        shoulder = self.target_positions[1]
        elbow = self.target_positions[2]
        forearm = self.target_positions[3]
        wrist = self.target_positions[4]
        
        # Calculate end effector position using MoveIt or fallback FK
        x, y, z, pitch, roll = self.forward_kinematics(
            base, shoulder, elbow, forearm, wrist
        )
        
        # Debug: Print FK position
        self.get_logger().info(f'FK Position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}')
        
        # Update Tkinter 3D visualization with actual FK position from TF
        if self.viz_3d:
            self.viz_3d.set_joint_angles(self.target_positions)
            # Set the actual gripper position from FK/TF for accurate green marker
            self.viz_3d.set_actual_gripper_position(x, y, z)
        
        # The URDF base joint has an initial rotation of 2.23697 rad (~128°) around Z
        # Rotate coordinates to match the URDF frame
        import math as m
        urdf_base_yaw = 2.23697
        cos_yaw = m.cos(-urdf_base_yaw)
        sin_yaw = m.sin(-urdf_base_yaw)
        x_rviz = x * cos_yaw - y * sin_yaw
        y_rviz = x * sin_yaw + y * cos_yaw
        z_rviz = z
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper_position"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (in meters) - rotated to match URDF frame
        marker.pose.position.x = x_rviz
        marker.pose.position.y = y_rviz
        marker.pose.position.z = z_rviz
        marker.pose.orientation.w = 1.0
        
        # Scale (sphere diameter in meters)
        marker.scale.x = 0.025  # 25mm diameter sphere (slightly larger)
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        
        # Color (bright green for actual gripper)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        # Lifetime (0 = forever until replaced)
        marker.lifetime = Duration(sec=0, nanosec=0)
        
        self.gripper_marker_pub.publish(marker)
        self.get_logger().debug(f"Published gripper marker at ({x:.3f}, {y:.3f}, {z:.3f})")
    
    def spin_ros(self):
        """Spin ROS2 node in background thread"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception as e:
                self.get_logger().error(f"ROS spin error: {str(e)}")
                break
    
    def update_gui(self):
        """Periodically update GUI elements"""
        try:
            # Update status indicator
            # Could add actual connection checking here
            pass
        except Exception as e:
            self.get_logger().error(f"GUI update error: {str(e)}")
        
        # Schedule next update
        self.root.after(100, self.update_gui)
    
    def on_closing(self):
        """Cleanup on window close"""
        self.get_logger().info("Shutting down GUI...")
        self._cleanup_camera()
        self.destroy_node()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    app = ArmControlGUI(root)
    
    # Handle window close
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
