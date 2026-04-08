#!/usr/bin/env python3
"""
Bartender Behavior Tree Node with YOLOv8 Nano Detection

Uses py_trees to orchestrate the bartender logic:
1. Wait for 'make drink' command
2. Look for a cup (YOLOv8n detection on camera feed)
3. Pick up the cup
4. Move it to the dispenser location
5. Wait 3 seconds for filling
6. Move the cup to the serving area
7. Release the gripper
8. Return to home position

Features:
- YOLOv8 Nano (yolov8n) for real-time cup detection (~40+ FPS)
- IK follows arm_gui.py pattern (smooth transitions with seed state)
- Slower movements (30% speed, longer durations) for safety
- Web dashboard with live camera feed and emergency stop button
- Flask server on localhost:5000
- Direct YOLO integration (no dependency on separate vision node)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import py_trees
from py_trees.common import Status

from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from sensor_msgs.msg import JointState, Image as ROSImage
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped

import time
import json
import math
import threading
import base64
import numpy as np
import os
from io import BytesIO

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from flask import Flask, render_template_string, jsonify, request
from arm_controller.ik_solver import ArmIKSolver

# Global Flask app instance
flask_app = None
bartender_node = None


class WaitForDrinkCommand(py_trees.behaviour.Behaviour):
    """Wait for a /make_drink command (can be triggered via CLI or another node)."""
    
    def __init__(self, name):
        super().__init__(name)
        self.command_received = False
        self.node = None
        
    def setup(self, **kwargs):
        """Called once when the tree is setup."""
        self.node = kwargs.get('node')
        if self.node:
            self.node.create_subscription(
                String,
                '/make_drink',
                self.drink_callback,
                10
            )
        return True
    
    def drink_callback(self, msg):
        """Callback when a drink command is received."""
        self.command_received = True
        self.node.get_logger().info(f"Drink command received: {msg.data}")
    
    def update(self):
        """Check if a command has been received."""
        if self.command_received:
            self.command_received = False  # Reset for next command
            return Status.SUCCESS
        return Status.RUNNING


class LookForCup(py_trees.behaviour.Behaviour):
    """Search for a cup using YOLOv8 Nano detection on camera frames."""
    
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.cup_found = False
        self.cup_position = None
        self.frame_skip = 2  # Process every 2nd frame for FPS optimization
        self.frame_count = 0
    
    def setup(self, **kwargs):
        """Called once when the tree is setup."""
        self.node = kwargs.get('node')
        if self.node:
            # Subscribe to camera for YOLO detection
            self.node.create_subscription(
                ROSImage,
                '/camera/image_raw',
                self.camera_callback,
                10
            )
        return True
    
    def camera_callback(self, msg):
        """Process camera frames for cup detection using YOLOv8n."""
        try:
            if not YOLO_AVAILABLE or not CV2_AVAILABLE:
                return
            
            # Skip frames to improve FPS
            self.frame_count += 1
            if self.frame_count % self.frame_skip != 0:
                return
            
            # Convert ROS Image to OpenCV format
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            
            # Detect cups using YOLOv8n
            if not hasattr(self.node, 'yolo_model') or self.node.yolo_model is None:
                return
            
            # Run inference (YOLOv8n is very fast)
            results = self.node.yolo_model(data, conf=0.45, verbose=False)
            
            # Look for cups (COCO class 41 = cup, 39 = bottle, 40 = wine glass)
            cup_detections = []
            for result in results:
                for box in result.boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Cup, bottle, or wine glass
                    if cls in [41, 39, 40] and conf > 0.45:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        width = x2 - x1
                        height = y2 - y1
                        
                        cup_detections.append({
                            'class': ['cup', 'bottle', 'wine_glass'][{41: 0, 39: 1, 40: 2}[cls]],
                            'confidence': conf,
                            'center_x': center_x,
                            'center_y': center_y,
                            'width': width,
                            'height': height,
                            'area': width * height
                        })
            
            if cup_detections:
                # Sort by confidence and area (pick largest confident detection)
                best_cup = max(cup_detections, key=lambda x: x['confidence'] * min(x['area'], 100000))
                self.cup_position = best_cup
                self.cup_found = True
                self.node.get_logger().debug(f"Cup detected: {best_cup['class']} (conf: {best_cup['confidence']:.2f})")
            else:
                self.cup_found = False
                
        except Exception as e:
            self.node.get_logger().debug(f"YOLO detection error: {e}")
    
    def update(self):
        """Check if a cup has been found."""
        if self.cup_found and self.cup_position:
            # Store the cup position in the blackboard for use by other behaviors
            self.blackboard.set('cup_position', self.cup_position, overwrite=True)
            self.node.get_logger().info(f"Cup found: {self.cup_position['class']} at ({self.cup_position['center_x']}, {self.cup_position['center_y']})")
            return Status.SUCCESS
        return Status.RUNNING


class PickUpCup(py_trees.behaviour.Behaviour):
    """Pick up the detected cup using IK solver based on YOLO detection."""
    
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.pick_in_progress = False
        self.ik_solver = ArmIKSolver()
        self.movement_start_time = None
        # Camera parameters for depth estimation
        self.focal_length = 500  # pixels (for 640x480 resolution)
        self.cup_real_width = 0.10  # 10cm typical cup width
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        return True
    
    def estimate_depth(self, pixel_width):
        """Estimate depth from pixel width using known object size."""
        if pixel_width < 5:
            return 0.20  # Default distance if detection too small
        depth = (self.cup_real_width * self.focal_length) / pixel_width
        return max(0.08, min(0.35, depth))  # Clamp between 8cm and 35cm
    
    def pixel_to_robot_coords(self, pixel_x, pixel_y, depth):
        """Convert pixel coordinates to robot coordinates."""
        # Camera frame assumptions:
        # - Image center at (320, 240) for 640x480
        # - Camera mounted looking down
        # - Depth perpendicular to image plane
        
        img_center_x, img_center_y = 320, 240
        
        # Convert pixels to offset from center
        dx_pixels = pixel_x - img_center_x
        dy_pixels = pixel_y - img_center_y
        
        # Convert to camera coordinates (roughly linear for small angles)
        # Assume ~60 degree FOV gives ~0.5m visible width at 0.2m depth
        pixels_per_meter = 320 / 0.5  # ~640 pixels = 0.5m width
        
        dx_cam = dx_pixels / pixels_per_meter
        dy_cam = dy_pixels / pixels_per_meter
        
        # Convert from camera to robot frame
        # Assuming camera mounted overhead looking down
        robot_x = depth
        robot_y = -dx_cam  # Flip y axis
        robot_z = 0.10  # Slightly above table
        
        return robot_x, robot_y, robot_z
    
    def update(self):
        """Execute IK-based pick operation based on YOLO detection."""
        cup_position = self.blackboard.get('cup_position')
        
        if not cup_position:
            self.node.get_logger().error("Cup position not found in blackboard!")
            return Status.FAILURE
        
        if not self.pick_in_progress:
            # Start the pick operation
            try:
                self.node.get_logger().info(f"Picking up {cup_position['class']}...")
                self.pick_in_progress = True
                self.movement_start_time = time.time()
                
                # Estimate 3D position from pixel coordinates
                depth = self.estimate_depth(cup_position['width'])
                pick_x, pick_y, pick_z = self.pixel_to_robot_coords(
                    cup_position['center_x'],
                    cup_position['center_y'],
                    depth
                )
                
                self.node.get_logger().info(f"Pick position: ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})")
                
                # Get seed state from current joint positions
                seed_state = self.node.get_current_seed_state()
                
                # Solve IK with seed state for smooth transition
                solution = self.ik_solver.solve_ik_to_point(
                    pick_x, pick_y, pick_z, 
                    seed_state=seed_state
                )
                
                if solution is None:
                    self.node.get_logger().error(f"Could not reach pick position ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})")
                    return Status.FAILURE
                
                # Publish joint commands with SLOW speed (30%)
                self.node.send_joint_commands(solution, speed=30.0)
                
            except Exception as e:
                self.node.get_logger().error(f"Pick operation failed: {e}")
                return Status.FAILURE
        
        # Wait for arm to move (3.0 seconds - slower movement)
        elapsed = time.time() - self.movement_start_time
        if elapsed < 3.0:
            return Status.RUNNING
        
        # Now close gripper
        try:
            gripper_client = self.node.create_client(SetBool, '/gripper/control')
            if gripper_client.wait_for_service(timeout_sec=1.0):
                req = SetBool.Request()
                req.data = True  # Close gripper
                future = gripper_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future)
                self.node.get_logger().info("Cup grasped!")
        except Exception as e:
            self.node.get_logger().warn(f"Gripper control failed: {e}")
        
        self.node.get_logger().info("Cup picked up successfully!")
        self.pick_in_progress = False
        self.movement_start_time = None
        return Status.SUCCESS


class MoveToDispenser(py_trees.behaviour.Behaviour):
    """Move the cup to the dispenser location using IK solver (like arm_gui.py)."""
    
    def __init__(self, name, dispenser_x=0.0, dispenser_y=0.2, dispenser_z=0.25):
        super().__init__(name)
        self.node = None
        self.dispenser_x = dispenser_x
        self.dispenser_y = dispenser_y
        self.dispenser_z = dispenser_z
        self.move_in_progress = False
        self.movement_start_time = None
        self.ik_solver = ArmIKSolver()
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node:
            self.node.get_logger().info(
                f"Dispenser location set to: ({self.dispenser_x}, "
                f"{self.dispenser_y}, {self.dispenser_z})"
            )
        return True
    
    def update(self):
        """Move arm to dispenser using IK with seed state (like arm_gui.py)."""
        if not self.move_in_progress:
            try:
                self.node.get_logger().info("Moving cup to dispenser...")
                self.move_in_progress = True
                self.movement_start_time = time.time()
                
                # Get seed state from current joint positions
                seed_state = self.node.get_current_seed_state()
                
                # Solve IK with seed state for smooth transition
                solution = self.ik_solver.solve_ik_to_point(
                    self.dispenser_x, 
                    self.dispenser_y, 
                    self.dispenser_z,
                    seed_state=seed_state
                )
                
                if solution is None:
                    self.node.get_logger().error(
                        f"Could not reach dispenser position ({self.dispenser_x}, "
                        f"{self.dispenser_y}, {self.dispenser_z})"
                    )
                    return Status.FAILURE
                
                # Publish joint commands with SLOW speed (30%)
                self.node.send_joint_commands(solution, speed=30.0)
                
            except Exception as e:
                self.node.get_logger().error(f"Move to dispenser failed: {e}")
                return Status.FAILURE
        
        # Wait for arm to move (4.0 seconds - slower movement)
        elapsed = time.time() - self.movement_start_time
        if elapsed < 4.0:
            return Status.RUNNING
        
        # Movement complete
        self.node.get_logger().info("Reached dispenser location!")
        self.move_in_progress = False
        self.movement_start_time = None
        return Status.SUCCESS


class WaitForFilling(py_trees.behaviour.Behaviour):
    """Wait for 3 seconds while the cup is being filled."""
    
    def __init__(self, name, wait_time=3.0):
        super().__init__(name)
        self.node = None
        self.wait_time = wait_time
        self.wait_start = None
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        return True
    
    def update(self):
        """Wait for the specified duration."""
        if self.wait_start is None:
            self.node.get_logger().info(f"Waiting {self.wait_time} seconds for filling...")
            self.wait_start = time.time()
        
        elapsed = time.time() - self.wait_start
        if elapsed >= self.wait_time:
            self.node.get_logger().info("Filling complete!")
            self.wait_start = None
            return Status.SUCCESS
        
        return Status.RUNNING


class MoveToServing(py_trees.behaviour.Behaviour):
    """Move the cup to the serving area using IK solver (like arm_gui.py)."""
    
    def __init__(self, name, serving_x=-0.1, serving_y=0.3, serving_z=0.15):
        super().__init__(name)
        self.node = None
        self.serving_x = serving_x
        self.serving_y = serving_y
        self.serving_z = serving_z
        self.move_in_progress = False
        self.movement_start_time = None
        self.ik_solver = ArmIKSolver()
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node:
            self.node.get_logger().info(
                f"Serving location set to: ({self.serving_x}, "
                f"{self.serving_y}, {self.serving_z})"
            )
        return True
    
    def update(self):
        """Move arm to serving area using IK with seed state (like arm_gui.py)."""
        if not self.move_in_progress:
            try:
                self.node.get_logger().info("Moving cup to serving area...")
                self.move_in_progress = True
                self.movement_start_time = time.time()
                
                # Get seed state from current joint positions
                seed_state = self.node.get_current_seed_state()
                
                # Solve IK with seed state for smooth transition
                solution = self.ik_solver.solve_ik_to_point(
                    self.serving_x,
                    self.serving_y,
                    self.serving_z,
                    seed_state=seed_state
                )
                
                if solution is None:
                    self.node.get_logger().error(
                        f"Could not reach serving position ({self.serving_x}, "
                        f"{self.serving_y}, {self.serving_z})"
                    )
                    return Status.FAILURE
                
                # Publish joint commands with SLOW speed (30%)
                self.node.send_joint_commands(solution, speed=30.0)
                
            except Exception as e:
                self.node.get_logger().error(f"Move to serving failed: {e}")
                return Status.FAILURE
        
        # Wait for arm to move (4.0 seconds - slower movement)
        elapsed = time.time() - self.movement_start_time
        if elapsed < 4.0:
            return Status.RUNNING
        
        # Movement complete
        self.node.get_logger().info("Reached serving area!")
        self.move_in_progress = False
        self.movement_start_time = None
        return Status.SUCCESS


class ReleaseGripper(py_trees.behaviour.Behaviour):
    """Release the gripper to drop the cup."""
    
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.gripper_client = None
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node:
            self.gripper_client = self.node.create_client(SetBool, '/gripper/control')
            # Wait for the service
            while not self.gripper_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('Gripper service not available, waiting...')
        return True
    
    def update(self):
        """Send an open command to the gripper."""
        self.node.get_logger().info("Releasing gripper...")
        
        # Send open command (False = open)
        req = SetBool.Request()
        req.data = False
        
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result().success:
            self.node.get_logger().info("Gripper released!")
            return Status.SUCCESS
        else:
            self.node.get_logger().error("Gripper release failed!")
            return Status.FAILURE


class ReturnToHome(py_trees.behaviour.Behaviour):
    """Return the arm to the home position using IK solver."""
    
    def __init__(self, name):
        super().__init__(name)
        self.node = None
        self.move_in_progress = False
        self.movement_start_time = None
        self.ik_solver = ArmIKSolver()
    
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        return True
    
    def update(self):
        """Move the arm back to home (like arm_gui.py pattern)."""
        if not self.move_in_progress:
            try:
                self.node.get_logger().info("Returning arm to home position...")
                self.move_in_progress = True
                self.movement_start_time = time.time()
                
                # Get home position from IK solver
                home_position = self.ik_solver.home_position.copy()
                
                # Publish joint commands to move arm to home with SLOW speed (30%)
                self.node.send_joint_commands(home_position, speed=30.0)
                
            except Exception as e:
                self.node.get_logger().error(f"Return to home failed: {e}")
                return Status.FAILURE
        
        # Wait for arm to move (4.0 seconds - slower movement)
        elapsed = time.time() - self.movement_start_time
        if elapsed < 4.0:
            return Status.RUNNING
        
        # Movement complete
        self.node.get_logger().info("Arm returned to home!")
        self.move_in_progress = False
        self.movement_start_time = None
        return Status.SUCCESS


class BartenderBehaviorTree(Node):
    """Main node that creates and runs the bartender behavior tree.
    
    Follows the same IK pattern as arm_gui.py:
    - Tracks current joint state from /joint_states
    - Uses seed state for smooth IK transitions
    - Publishes joint commands with speed control
    """
    
    def __init__(self):
        super().__init__('bartender_behavior_tree')
        
        self.declare_parameter('dispenser_x', 0.0)
        self.declare_parameter('dispenser_y', 0.2)
        self.declare_parameter('dispenser_z', 0.25)
        self.declare_parameter('serving_x', -0.1)
        self.declare_parameter('serving_y', 0.3)
        self.declare_parameter('serving_z', 0.15)
        self.declare_parameter('wait_time', 3.0)
        self.declare_parameter('speed', 50.0)
        
        # Joint state tracking (like arm_gui.py)
        self.joint_names = ['base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper']
        self.current_joint_positions = {name: 90.0 for name in self.joint_names}  # Default 90° = home
        
        # Camera frame for web dashboard
        self.latest_camera_frame = None
        self.camera_frame_lock = threading.Lock()
        self.emergency_stop_active = False
        
        # Initialize YOLOv8n model for cup detection
        self.yolo_model = None
        self.init_yolo_model()
        
        # Publishers
        self.joint_commands_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )
        
        self.speed_pub = self.create_publisher(
            Float32,
            '/servo_speed',
            10
        )
        
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        # Subscribers
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to camera feed for web dashboard
        self.create_subscription(
            ROSImage,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # Store node reference in a class variable so behaviors can access it
        BartenderBehaviorTree.instance = self
        
        # Build the behavior tree
        self.root = self.build_tree()
        
        # Create a timer to tick the tree at 10 Hz
        self.create_timer(0.1, self.tick_tree)
        
        # Start Flask web server in background thread
        self.setup_flask_server()
        
        self.get_logger().info("Bartender Behavior Tree initialized!")
        self.get_logger().info("🌐 Web Dashboard available at: http://localhost:5000")
    
    def init_yolo_model(self):
        """Initialize YOLOv8 Nano model for cup detection."""
        if not YOLO_AVAILABLE:
            self.get_logger().error("YOLOv8 not available! Install with: pip install ultralytics")
            return
        
        try:
            self.get_logger().info("Loading YOLOv8n model...")
            # Try local model first
            model_path = '/workspace/yolov8n.pt'
            if os.path.exists(model_path):
                self.yolo_model = YOLO(model_path)
                self.get_logger().info(f"✅ Loaded YOLOv8n from {model_path}")
            else:
                # Download model if not found (first run only)
                self.get_logger().info("Downloading YOLOv8n model...")
                self.yolo_model = YOLO('yolov8n.pt')
                self.get_logger().info("✅ YOLOv8n model loaded (cached for future runs)")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8n: {e}")
            self.yolo_model = None
    
    def build_tree(self):
        """Build the behavior tree structure."""
        
        # Get parameters
        dispenser_x = self.get_parameter('dispenser_x').value
        dispenser_y = self.get_parameter('dispenser_y').value
        dispenser_z = self.get_parameter('dispenser_z').value
        serving_x = self.get_parameter('serving_x').value
        serving_y = self.get_parameter('serving_y').value
        serving_z = self.get_parameter('serving_z').value
        wait_time = self.get_parameter('wait_time').value
        
        # Main bartender sequence
        main_sequence = py_trees.composites.Sequence(name="Bartender Sequence", memory=False)
        
        # Step 1: Wait for drink command
        main_sequence.add_child(WaitForDrinkCommand(name="Wait for Drink Command"))
        
        # Step 2: Look for cup (using YOLOv8n detection)
        main_sequence.add_child(LookForCup(name="Look for Cup (YOLOv8n)"))
        
        # Step 3: Pick up cup
        main_sequence.add_child(PickUpCup(name="Pick Up Cup"))
        
        # Step 4: Move to dispenser
        main_sequence.add_child(
            MoveToDispenser(
                name="Move to Dispenser",
                dispenser_x=dispenser_x,
                dispenser_y=dispenser_y,
                dispenser_z=dispenser_z
            )
        )
        
        # Step 5: Wait for filling
        main_sequence.add_child(WaitForFilling(name="Wait for Filling", wait_time=wait_time))
        
        # Step 6: Move to serving area
        main_sequence.add_child(
            MoveToServing(
                name="Move to Serving",
                serving_x=serving_x,
                serving_y=serving_y,
                serving_z=serving_z
            )
        )
        
        # Step 7: Release gripper
        main_sequence.add_child(ReleaseGripper(name="Release Gripper"))
        
        # Step 8: Return to home
        main_sequence.add_child(ReturnToHome(name="Return to Home"))
        
        # Return the main sequence as the root
        # It will be ticked repeatedly by the timer callback
        return main_sequence
    
    def tick_tree(self):
        """Tick the behavior tree at regular intervals."""
        self.root.tick_once()
    
    def joint_state_callback(self, msg):
        """Update current joint positions from /joint_states (like arm_gui.py)."""
        try:
            # Extract joint positions in GUI convention (90° = home)
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    # Convert from radians to degrees: degrees = radians * 180/pi + 90
                    # Account for URDF inversion for shoulder and elbow
                    if name in ['shoulder', 'elbow']:
                        # These are inverted in URDF
                        self.current_joint_positions[name] = 90.0 - math.degrees(msg.position[i])
                    elif name == 'forearm':
                        # Wrist forearm has special negation
                        self.current_joint_positions[name] = 90.0 - math.degrees(msg.position[i])
                    else:
                        # Normal joints
                        self.current_joint_positions[name] = 90.0 + math.degrees(msg.position[i])
        except Exception as e:
            self.get_logger().warn(f"Error updating joint state: {e}")
    
    def get_current_seed_state(self):
        """Return current joint positions as seed state for smooth IK transitions (like arm_gui.py)."""
        return self.current_joint_positions.copy()
    
    def send_joint_commands(self, joint_angles_dict, speed=50.0):
        """
        Publish joint commands with speed control (like arm_gui.py).
        
        Args:
            joint_angles_dict: Dictionary with joint names as keys and angles (in degrees) as values.
                              Must contain: base, shoulder, elbow, forearm, wrist, gripper
            speed: Speed percentage (0-100)
        """
        try:
            # Publish speed first (important: before movement)
            speed_msg = Float32()
            speed_msg.data = float(speed)
            self.speed_pub.publish(speed_msg)
            
            # Extract joint angles in the correct order for hardware
            joint_order = ['base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper']
            
            joint_values = []
            for joint in joint_order:
                if joint in joint_angles_dict:
                    joint_values.append(float(joint_angles_dict[joint]))
                else:
                    self.get_logger().warn(f"Joint '{joint}' not found in solution, using 90°")
                    joint_values.append(90.0)
            
            # Update current positions for tracking
            for joint, value in zip(joint_order, joint_values):
                self.current_joint_positions[joint] = value
            
            # Publish as Float32MultiArray (like arm_gui.py)
            msg = Float32MultiArray()
            msg.data = joint_values
            self.joint_commands_pub.publish(msg)
            
            self.get_logger().debug(f"Published joint commands at {speed}% speed: {[f'{x:.1f}°' for x in joint_values]}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish joint commands: {e}")
    
    def camera_callback(self, msg):
        """Store latest camera frame for web dashboard streaming."""
        try:
            if not CV2_AVAILABLE:
                return
            # Convert ROS Image message to OpenCV format
            import numpy as np
            data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            with self.camera_frame_lock:
                self.latest_camera_frame = data.copy()
        except Exception as e:
            self.get_logger().debug(f"Camera callback error: {e}")
    
    def setup_flask_server(self):
        """Initialize Flask web server in background thread."""
        global flask_app, bartender_node
        bartender_node = self
        
        html_template = '''
<!DOCTYPE html>
<html>
<head>
    <title>RoboArm - Web Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }
        .header h1 {
            margin: 0;
            font-size: 2.5em;
        }
        .header p {
            margin: 10px 0 0 0;
            font-size: 1.1em;
            opacity: 0.9;
        }
        .content {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            padding: 30px;
        }
        .camera-section {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .camera-feed {
            width: 100%;
            height: 480px;
            background: #000;
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
            border: 3px solid #667eea;
        }
        .camera-feed img {
            max-width: 100%;
            max-height: 100%;
            object-fit: contain;
        }
        .camera-feed.no-feed {
            color: white;
            font-size: 1.2em;
        }
        .controls {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }
        .control-panel {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 10px;
            border-left: 4px solid #667eea;
        }
        .control-panel h3 {
            margin: 0 0 15px 0;
            color: #667eea;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid #ddd;
        }
        .status-item:last-child {
            border-bottom: none;
        }
        .status-label {
            font-weight: bold;
            color: #555;
        }
        .status-value {
            color: #667eea;
            font-family: monospace;
        }
        .button-group {
            display: flex;
            gap: 10px;
            flex-direction: column;
        }
        .btn {
            padding: 15px 25px;
            font-size: 1em;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .btn-primary {
            background: #667eea;
            color: white;
        }
        .btn-primary:hover {
            background: #5568d3;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        .btn-danger {
            background: #ff6b6b;
            color: white;
            animation: pulse-danger 2s infinite;
        }
        .btn-danger:hover {
            background: #ff5252;
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(255, 107, 107, 0.4);
        }
        @keyframes pulse-danger {
            0%, 100% { box-shadow: 0 0 0 0 rgba(255, 107, 107, 0.7); }
            50% { box-shadow: 0 0 0 10px rgba(255, 107, 107, 0); }
        }
        .status-active {
            color: #27ae60;
            font-weight: bold;
        }
        .status-inactive {
            color: #e74c3c;
            font-weight: bold;
        }
        @media (max-width: 768px) {
            .content {
                grid-template-columns: 1fr;
            }
            .header h1 {
                font-size: 1.8em;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 RoboArm Dashboard</h1>
            <p>Real-time Control & Monitoring</p>
        </div>
        <div class="content">
            <div class="camera-section">
                <h2>Camera Feed</h2>
                <div class="camera-feed" id="cameraFeed">
                    <div class="camera-feed no-feed">Loading camera...</div>
                </div>
            </div>
            <div class="controls">
                <div class="control-panel">
                    <h3>⚡ Emergency Control</h3>
                    <button class="btn btn-danger" id="emergencyBtn" onclick="toggleEmergencyStop()">
                        🔴 EMERGENCY STOP (All Motors OFF)
                    </button>
                </div>
                <div class="control-panel">
                    <h3>📊 System Status</h3>
                    <div class="status-item">
                        <span class="status-label">Motors:</span>
                        <span class="status-value" id="motorStatus" class="status-active">ACTIVE</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Emergency Stop:</span>
                        <span class="status-value" id="emergencyStatus" class="status-inactive">INACTIVE</span>
                    </div>
                    <div class="status-item">
                        <span class="status-label">Last Update:</span>
                        <span class="status-value" id="lastUpdate">Waiting...</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Update timestamp
        setInterval(function() {
            document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
        }, 1000);

        // Camera streaming
        function streamCamera() {
            const img = document.createElement('img');
            img.onerror = function() {
                setTimeout(streamCamera, 2000);
            };
            img.onload = function() {
                const feed = document.getElementById('cameraFeed');
                feed.innerHTML = '';
                feed.appendChild(img);
                setTimeout(streamCamera, 50);
            };
            img.src = '/stream?t=' + Date.now();
        }
        streamCamera();

        // Emergency stop
        let emergencyActive = false;
        function toggleEmergencyStop() {
            emergencyActive = !emergencyActive;
            fetch('/emergency_stop', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({active: emergencyActive})
            }).then(r => r.json()).then(data => {
                const btn = document.getElementById('emergencyBtn');
                const status = document.getElementById('emergencyStatus');
                const motorStatus = document.getElementById('motorStatus');
                
                if (emergencyActive) {
                    btn.textContent = '✅ EMERGENCY STOP ACTIVE - Click to Resume';
                    status.textContent = 'ACTIVE';
                    status.className = 'status-value status-active';
                    motorStatus.textContent = 'DISABLED';
                    motorStatus.className = 'status-value status-inactive';
                } else {
                    btn.textContent = '🔴 EMERGENCY STOP (All Motors OFF)';
                    status.textContent = 'INACTIVE';
                    status.className = 'status-value status-inactive';
                    motorStatus.textContent = 'ACTIVE';
                    motorStatus.className = 'status-value status-active';
                }
            });
        }
    </script>
</body>
</html>
        '''
        
        flask_app = Flask(__name__)
        
        @flask_app.route('/')
        def dashboard():
            return render_template_string(html_template)
        
        @flask_app.route('/stream')
        def stream():
            """Stream camera frame as JPEG."""
            if not CV2_AVAILABLE:
                return b'', 204
            
            # Try to read directly from shared JPEG file created by camera_stream.sh
            try:
                camera_file = '/workspace/.camera_frame.jpg'
                if os.path.exists(camera_file):
                    with open(camera_file, 'rb') as f:
                        img_bytes = f.read()
                    if img_bytes:
                        return b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + \
                               str(len(img_bytes)).encode() + b'\r\n\r\n' + img_bytes + b'\r\n'
            except Exception as e:
                pass
            
            # Fallback: try ROS2 subscription if file not available
            if bartender_node.latest_camera_frame is not None:
                with bartender_node.camera_frame_lock:
                    frame = bartender_node.latest_camera_frame
                    if frame is not None:
                        ret, buffer = cv2.imencode('.jpg', frame)
                        if ret:
                            img_bytes = buffer.tobytes()
                            return b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + \
                                   str(len(img_bytes)).encode() + b'\r\n\r\n' + img_bytes + b'\r\n'
            
            return b'', 204
        
        @flask_app.route('/emergency_stop', methods=['POST'])
        def emergency_stop():
            """Handle emergency stop button."""
            try:
                data = request.get_json()
                bartender_node.emergency_stop_active = data.get('active', False)
                
                # Publish emergency stop command
                msg = Bool()
                msg.data = bartender_node.emergency_stop_active
                bartender_node.emergency_stop_pub.publish(msg)
                
                if bartender_node.emergency_stop_active:
                    bartender_node.get_logger().warning("⛔ EMERGENCY STOP ACTIVATED - All motors disabled!")
                else:
                    bartender_node.get_logger().info("✅ Emergency stop released - Motors enabled")
                
                return jsonify({'status': 'ok', 'active': bartender_node.emergency_stop_active})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        # Run Flask in background thread
        flask_thread = threading.Thread(
            target=lambda: flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False),
            daemon=True
        )
        flask_thread.start()
        self.get_logger().info("Flask web server started on port 5000")


def main(args=None):
    rclpy.init(args=args)
    node = BartenderBehaviorTree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down bartender behavior tree...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
