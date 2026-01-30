#!/usr/bin/env python3
"""
MoveIt IK Client for Robotic Arm
Uses MoveIt's compute_ik service for accurate inverse kinematics
"""

import math
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class MoveItIKClient:
    """
    MoveIt IK Client that provides inverse and forward kinematics
    using MoveIt's KDL solver configured for the arm.
    """
    
    # Joint names in the arm group (from SRDF)
    JOINT_NAMES = ['base', 'shoulder', 'elbow', 'wrist', 'wrist_rotate']
    
    # Planning group name (from SRDF)
    PLANNING_GROUP = 'arm'
    
    # End effector link (tip of the kinematic chain)
    END_EFFECTOR_LINK = 'gripper_base'
    
    def __init__(self, node: Node):
        """
        Initialize the MoveIt IK client.
        
        Args:
            node: The ROS2 node to use for service clients and TF
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Create callback group for service calls
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create IK service client
        self.ik_client = node.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )
        
        # TF buffer for forward kinematics via transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # Home position (GUI degrees, 90° = home)
        self.home_position = {
            'base': 90.0,
            'shoulder': 90.0,
            'elbow': 90.0,
            'wrist': 90.0,
            'wrist_rotate': 90.0,
            'gripper': 90.0
        }
        
        # Track if MoveIt is available
        self._moveit_available = False
        self._check_moveit_availability()
    
    def _check_moveit_availability(self):
        """Check if MoveIt's compute_ik service is available."""
        self._moveit_available = self.ik_client.wait_for_service(timeout_sec=2.0)
        if self._moveit_available:
            self.logger.info("MoveIt IK service is available!")
        else:
            self.logger.warning("MoveIt IK service NOT available. Is MoveIt running?")
    
    def is_available(self) -> bool:
        """Check if MoveIt IK service is available."""
        return self._moveit_available
    
    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        """Wait for MoveIt IK service to become available."""
        self._moveit_available = self.ik_client.wait_for_service(timeout_sec=timeout_sec)
        return self._moveit_available
    
    def solve_ik(
        self,
        x: float,
        y: float, 
        z: float,
        pitch: float = 0.0,
        roll: float = 0.0,
        gripper_angle: Optional[float] = None,
        seed_state: Optional[Dict[str, float]] = None
    ) -> Optional[Dict[str, float]]:
        """
        Solve inverse kinematics for a target position.
        
        Args:
            x, y, z: Target position in meters (world frame)
            pitch: Desired pitch angle of end effector (radians)
            roll: Desired roll angle of end effector (radians)
            gripper_angle: Gripper opening angle (degrees), None to keep current
            seed_state: Optional seed joint state for IK solver (degrees)
            
        Returns:
            Dict of joint angles in degrees (GUI convention: 90° = home),
            or None if IK fails
        """
        if not self._moveit_available:
            self.logger.error("MoveIt IK service not available!")
            return None
        
        # Build the IK request
        request = GetPositionIK.Request()
        
        # Set the planning group
        request.ik_request.group_name = self.PLANNING_GROUP
        
        # Build target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Calculate orientation for gripper to point towards the target
        # The gripper should point outward from the base towards the XY position
        # Yaw: rotation around Z to point towards target in XY plane
        # Pitch: angle to reach the height (pointing down if z < arm height, up if z > arm height)
        yaw = math.atan2(y, x)  # Point towards target in XY plane
        
        # For a reaching pose, we typically want gripper pointing outward and slightly down
        # The pitch should be relative to horizontal, not the input pitch
        # A pitch of 0 means horizontal gripper, negative means pointing down
        gripper_pitch = pitch  # Use the provided pitch (0 = horizontal)
        
        # Roll is rotation around the gripper's pointing direction
        gripper_roll = roll
        
        quat = self._euler_to_quaternion(gripper_roll, gripper_pitch, yaw)
        target_pose.pose.orientation = quat
        
        request.ik_request.pose_stamped = target_pose
        
        # Set the tip link
        request.ik_request.ik_link_name = self.END_EFFECTOR_LINK
        
        # ALWAYS set seed state - MoveIt needs this for proper IK solving
        robot_state = RobotState()
        robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        robot_state.joint_state.name = self.JOINT_NAMES
        
        # Convert from GUI degrees (90° = home) to radians (0 = home)
        # Use provided seed_state or default to home position
        positions = []
        for name in self.JOINT_NAMES:
            if seed_state and name in seed_state:
                gui_deg = seed_state[name]
            else:
                gui_deg = self.home_position.get(name, 90.0)
            
            # Handle shoulder and elbow inversion for MoveIt (matches URDF joint axes)
            if name == 'shoulder' or name == 'elbow':
                rad_val = math.radians(90.0 - gui_deg)
            else:
                rad_val = math.radians(gui_deg - 90.0)
            positions.append(rad_val)
        
        robot_state.joint_state.position = positions
        request.ik_request.robot_state = robot_state
        
        # IK solver attempts - increase timeout for better solutions
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.5 * 1e9)  # 500ms timeout
        
        try:
            # Call the service synchronously
            future = self.ik_client.call_async(request)
            
            # Wait for result (with timeout)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            
            if future.result() is None:
                self.logger.error("IK service call failed (no result)")
                return None
            
            response = future.result()
            
            if response.error_code.val != response.error_code.SUCCESS:
                self.logger.warning(f"IK failed with error code: {response.error_code.val}")
                return None
            
            # Extract joint positions
            joint_state = response.solution.joint_state
            
            # Build result dictionary with proper GUI joint names
            # GUI uses: base, shoulder, elbow, forearm (=URDF wrist), wrist (=URDF wrist_rotate), gripper
            result = {}
            for i, name in enumerate(joint_state.name):
                if name in self.JOINT_NAMES:
                    # Convert from radians (0 = home) to GUI degrees (90° = home)
                    rad_value = joint_state.position[i]
                    
                    # Handle shoulder and elbow inversion (matches URDF joint axes)
                    if name == 'shoulder' or name == 'elbow':
                        deg_value = 90.0 - math.degrees(rad_value)
                    else:
                        deg_value = math.degrees(rad_value) + 90.0
                    
                    # Clamp to 0-180
                    deg_value = max(0.0, min(180.0, deg_value))
                    
                    # Map URDF joint names to GUI joint names
                    # URDF 'wrist' -> GUI 'forearm' (index 3)
                    # URDF 'wrist_rotate' -> GUI 'wrist' (index 4)
                    if name == 'wrist':
                        result['forearm'] = deg_value
                    elif name == 'wrist_rotate':
                        result['wrist'] = deg_value
                    else:
                        result[name] = deg_value
            
            # Add gripper (not controlled by MoveIt)
            result['gripper'] = gripper_angle if gripper_angle is not None else 90.0
            
            self.logger.info(f"IK solution found: {result}")
            return result
            
        except Exception as e:
            self.logger.error(f"IK service call exception: {e}")
            return None
    
    def forward_kinematics(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        forearm: float,
        wrist: float
    ) -> Tuple[float, float, float, float, float]:
        """
        Calculate end effector position from joint angles using TF.
        
        Args:
            Joint angles in degrees (GUI convention: 90° = home)
            
        Returns:
            Tuple of (x, y, z, pitch, roll) position and orientation
        """
        try:
            # Try to get transform from TF
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.END_EFFECTOR_LINK,
                rclpy.time.Time()
            )
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # Extract orientation as pitch/roll
            quat = trans.transform.rotation
            pitch, roll = self._quaternion_to_pitch_roll(quat)
            
            return (x, y, z, pitch, roll)
            
        except TransformException as e:
            self.logger.debug(f"TF lookup failed: {e}, using fallback FK")
            # Fallback to simple geometric FK
            return self._fallback_forward_kinematics(base, shoulder, elbow, forearm, wrist)
    
    def _fallback_forward_kinematics(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        forearm: float,
        wrist: float
    ) -> Tuple[float, float, float, float, float]:
        """
        Fallback forward kinematics using geometric model with URDF link lengths.
        Used when TF is not available.
        """
        # Link lengths from URDF measurements (in meters)
        L_base = 0.059    # Base to shoulder height
        L_upper = 0.036   # Shoulder to elbow
        L_lower = 0.084   # Elbow to wrist
        L_wrist = 0.086   # Wrist to gripper
        
        # Convert to radians matching RViz/URDF convention
        base_rad = math.radians(base - 90.0)
        shoulder_rad = math.radians(90.0 - shoulder)  # Inverted
        elbow_rad = math.radians(90.0 - elbow)  # Inverted
        forearm_rad = math.radians(forearm - 90.0)
        
        # Calculate positions using vertical-based system (matching Tkinter)
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_lower * math.sin(arm_angle)
        wrist_z = elbow_z + L_lower * math.cos(arm_angle)
        
        pitch = arm_angle + forearm_rad
        end_x = wrist_x + L_wrist * math.sin(pitch)
        end_z = wrist_z + L_wrist * math.cos(pitch)
        
        x = end_x * math.cos(base_rad)
        y = end_x * math.sin(base_rad)
        z = end_z
        
        wrist_rad = math.radians(wrist - 90.0)
        
        return (x, y, z, pitch, wrist_rad)
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles (radians) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def _quaternion_to_pitch_roll(self, q: Quaternion) -> Tuple[float, float]:
        """Extract pitch and roll from quaternion."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        return (pitch, roll)
