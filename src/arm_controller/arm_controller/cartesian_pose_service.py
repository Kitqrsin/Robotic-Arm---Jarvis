#!/usr/bin/env python3
"""
Cartesian Pose Service Node
Provides ROS2 service to move arm to Cartesian positions using IK
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
import math
import json
import time
try:
    from .ik_solver import ArmIKSolver, PredefinedPoses
except ImportError:
    from ik_solver import ArmIKSolver, PredefinedPoses


class CartesianPoseService(Node):
    def __init__(self):
        super().__init__('cartesian_pose_service')
        
        # Initialize IK solver
        self.ik_solver = ArmIKSolver()
        self.predefined_poses = PredefinedPoses()
        
        # Track current joint positions
        self.current_angles = {
            'base': 90,
            'shoulder': 90,
            'elbow': 90,
            'forearm': 70,
            'wrist': 60,
            'gripper': 90
        }
        
        # Movement parameters
        self.interpolation_steps = 20  # Number of steps for smooth movement
        self.step_delay = 0.03  # Delay between steps (30ms = reasonable speed)
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )
        
        # Subscriber for pose commands (topic-based interface)
        self.pose_sub = self.create_subscription(
            String,
            'execute_pose',
            self.execute_pose_callback,
            10
        )
        
        # Subscriber for Cartesian position commands
        self.cartesian_sub = self.create_subscription(
            Point,
            'move_to_position',
            self.move_to_position_callback,
            10
        )
        
        self.get_logger().info('Cartesian Pose Service started')
        self.get_logger().info(f'Available poses: {list(self.predefined_poses.get_all_poses().keys())}')
    
    def execute_pose_callback(self, msg):
        """Execute a predefined pose by name"""
        pose_name = msg.data.lower()
        all_poses = self.predefined_poses.get_all_poses()
        
        if pose_name not in all_poses:
            self.get_logger().warn(
                f"Pose '{pose_name}' not found. Available: {list(all_poses.keys())}"
            )
            return
        
        pose_data = all_poses[pose_name]
        self.get_logger().info(f'Executing predefined pose: {pose_name}')
        
        # Solve IK for this pose
        x, y, z = pose_data['position']
        pitch, roll = pose_data['orientation']
        gripper = pose_data['gripper']
        
        joint_angles = self.ik_solver.solve_ik(x, y, z, pitch, roll, gripper)
        
        if joint_angles is None:
            self.get_logger().error(f"Pose '{pose_name}' is unreachable")
            return
        
        # Publish joint command
        self._publish_joint_command_smoothly(joint_angles)
        self.get_logger().info(f'Executed {pose_name}')
    
    def move_to_position_callback(self, msg):
        """Move to Cartesian position (Point message)"""
        self.get_logger().info(
            f'Moving to position: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})'
        )
        
        # Solve IK with default orientation
        joint_angles = self.ik_solver.solve_ik(
            msg.x, msg.y, msg.z,
            pitch=0, roll=0, gripper_angle=None
        )
        
        if joint_angles is None:
            self.get_logger().warn('Target position unreachable!')
            return
        
        # Publish joint command
        self._publish_joint_command_smoothly(joint_angles)
        self.get_logger().info('Moving to target position')
    
    def _interpolate_angles(self, start_angles, target_angles, steps):
        """Generate interpolated angles between start and target"""
        interpolated = []
        for i in range(steps + 1):
            alpha = i / steps
            intermediate = {}
            for joint in start_angles.keys():
                start = start_angles[joint]
                target = target_angles[joint]
                intermediate[joint] = start + (target - start) * alpha
            interpolated.append(intermediate)
        return interpolated
    
    def _publish_joint_command_smoothly(self, target_angles):
        """Publish joint angles with smooth interpolation"""
        # Generate interpolated trajectory
        trajectory = self._interpolate_angles(
            self.current_angles,
            target_angles,
            self.interpolation_steps
        )
        
        # Execute trajectory
        for angles in trajectory:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper']
            msg.position = [
                math.radians(angles['base']),
                math.radians(angles['shoulder']),
                math.radians(angles['elbow']),
                math.radians(angles['forearm']),
                math.radians(angles['wrist']),
                math.radians(angles['gripper'])
            ]
            
            self.joint_pub.publish(msg)
            time.sleep(self.step_delay)
        
        # Update current position
        self.current_angles = target_angles.copy()
    
    def _publish_joint_command(self, joint_angles):
        """Publish joint angles as command"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper']
        msg.position = [
            math.radians(joint_angles['base']),
            math.radians(joint_angles['shoulder']),
            math.radians(joint_angles['elbow']),
            math.radians(joint_angles['forearm']),
            math.radians(joint_angles['wrist']),
            math.radians(joint_angles['gripper'])
        ]
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPoseService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
