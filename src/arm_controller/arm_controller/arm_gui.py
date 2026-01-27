#!/usr/bin/env python3
"""
Jarvis Robot Arm Control GUI
Simple Tkinter-based control interface for testing ROS2 integration
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float32, Bool
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

# Import IK solver and 3D visualization
from arm_controller.ik_solver import ArmIKSolver
from arm_controller.arm_3d_visualization import Arm3DVisualization


class ArmControlGUI(Node):
    def __init__(self, root):
        super().__init__('arm_control_gui')
        
        self.root = root
        self.root.title("Jarvis Robot Arm Controller")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # Joint names matching your OnShape URDF
        self.joint_names = [
            'base',           # Base rotation (J1)
            'shoulder',       # Shoulder pitch (J2)
            'elbow',          # Elbow pitch (J3)
            'wrist',          # Wrist pitch (J4)
            'wrist_rotate',   # Wrist roll (J5)
            'gripper'         # Gripper (J6)
        ]
        
        # Current joint positions (in degrees)
        self.current_positions = [90.0] * 6
        self.target_positions = [90.0] * 6
        
        # Initialize IK solver
        self.ik_solver = ArmIKSolver()
        
        # 3D visualization (will be set up in setup_gui)
        self.viz_3d = None
        
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
        
        # Track emergency stop state
        self.estop_active = False
        
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
            
            # Slider
            slider = tk.Scale(
                joint_frame,
                from_=0,
                to=180,
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
        cartesian_ranges = {
            'X': (-300, 300),
            'Y': (-300, 300),
            'Z': (0, 400)
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
            self.ik_solver,
            on_target_move_callback=self.on_3d_target_move
        )
        
        # Sync initial cartesian values to 3D viz
        self.viz_3d.set_target(0.0, 0.0, 0.2)
        
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
    
    def on_3d_target_move(self, x, y, z):
        """
        Callback when the 3D visualization target is moved.
        Updates cartesian controls and solves IK.
        
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
        
        # Solve IK
        joint_angles = self.ik_solver.solve_ik(x, y, z, pitch=0, roll=0)
        
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
        
        # Update sliders with IK solution
        for ik_joint, gui_idx in ik_to_gui_mapping.items():
            angle = joint_angles[ik_joint]
            angle = max(0, min(180, angle))
            self.joint_sliders[gui_idx].set(angle)
            self.target_positions[gui_idx] = angle
        
        # Update 3D visualization with the IK solution
        if self.viz_3d:
            self.viz_3d.update_from_ik_solution(joint_angles)
        
        # Publish target marker for RViz
        self.publish_target_marker(x, y, z)
        
        # Real-time mode: auto-send to robot
        if self.realtime_mode.get():
            self.schedule_update()
        else:
            self.info_label.config(
                text=f"✓ IK solved for ({int(x_mm)}, {int(y_mm)}, {int(z_mm)}) mm. Click 'Send to Robot'",
                fg='#00ff00'
            )

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
            # Elbow (index 2) needs to be inverted for correct visualization
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            
            # Build joint positions with elbow inversion for RViz
            rviz_positions = []
            for i, pos in enumerate(self.target_positions):
                if i == 2:  # Elbow joint - invert for RViz
                    # Invert: if GUI shows 90°, RViz shows 0°; if GUI shows 0°, RViz shows +90°
                    rviz_positions.append(float(math.radians(90.0 - pos)))
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
            
            if self.estop_active:
                self.info_label.config(text="⛔ EMERGENCY STOP ACTIVE - All outputs DISABLED!", fg='#ff0000')
                self.get_logger().warning("Emergency stop ACTIVATED - OE pin HIGH")
                
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
            
            # Solve IK (pitch=0 for horizontal gripper, roll=0)
            joint_angles = self.ik_solver.solve_ik(x_m, y_m, z_m, pitch=0, roll=0)
            
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
            
            # Update sliders with IK solution
            for ik_joint, gui_idx in ik_to_gui_mapping.items():
                angle = joint_angles[ik_joint]
                # Clamp to 0-180 range
                angle = max(0, min(180, angle))
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
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_position"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (in meters)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
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
        
        # Calculate end effector position
        x, y, z, pitch, roll = self.ik_solver.forward_kinematics(
            base, shoulder, elbow, forearm, wrist
        )
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gripper_position"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (in meters)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
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
