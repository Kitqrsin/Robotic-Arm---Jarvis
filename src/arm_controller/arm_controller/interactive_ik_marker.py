#!/usr/bin/env python3
"""
Interactive IK Marker Node for RViz
Publishes an interactive marker that can be dragged to set the target position
for inverse kinematics. The arm follows the marker using IK calculations.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
import numpy as np

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

# Import IK solver
from arm_controller.ik_solver import ArmIKSolver


class InteractiveIKMarker(Node):
    """
    Interactive marker node that allows dragging a point in RViz
    to control the robot arm using inverse kinematics.
    """
    
    def __init__(self):
        super().__init__('interactive_ik_marker')
        
        # Initialize IK solver
        self.ik_solver = ArmIKSolver()
        
        # Current target position (meters)
        self.target_x = 0.2
        self.target_y = 0.0
        self.target_z = 0.15
        
        # Gripper orientation (pitch and roll in radians)
        self.target_pitch = 0.0  # Horizontal gripper
        self.target_roll = 0.0
        self.gripper_angle = 90.0  # Default gripper opening
        
        # Joint names matching URDF
        self.joint_names = [
            'base',           # Base rotation (J1)
            'shoulder',       # Shoulder pitch (J2)
            'elbow',          # Elbow pitch (J3)
            'wrist',          # Wrist pitch (J4)
            'wrist_rotate',   # Wrist roll (J5)
            'gripper'         # Gripper (J6)
        ]
        
        # Publishers
        self.joint_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )
        
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, 'ik_target_marker')
        
        # Create the draggable marker
        self.create_ik_target_marker()
        
        # Apply changes to make marker visible
        self.server.applyChanges()
        
        # Timer to periodically update marker (in case of external changes)
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Interactive IK Marker node started')
        self.get_logger().info(f'Initial target position: ({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f})')
    
    def create_ik_target_marker(self):
        """Create an interactive marker for IK target position"""
        
        # Create the main interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = "ik_target"
        int_marker.description = "Drag to move gripper"
        int_marker.scale = 0.1
        
        # Set initial position
        int_marker.pose.position.x = self.target_x
        int_marker.pose.position.y = self.target_y
        int_marker.pose.position.z = self.target_z
        int_marker.pose.orientation.w = 1.0
        
        # Create a sphere marker for visualization
        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.04  # 40mm diameter
        sphere_marker.scale.y = 0.04
        sphere_marker.scale.z = 0.04
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.5
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.9
        
        # Add a control that shows the sphere and handles mouse interaction
        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        sphere_control.markers.append(sphere_marker)
        sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(sphere_control)
        
        # Add XYZ translation controls (arrows)
        # X-axis (red)
        x_control = InteractiveMarkerControl()
        x_control.name = "move_x"
        x_control.orientation.w = 1.0
        x_control.orientation.x = 1.0
        x_control.orientation.y = 0.0
        x_control.orientation.z = 0.0
        x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(x_control)
        
        # Y-axis (green)  
        y_control = InteractiveMarkerControl()
        y_control.name = "move_y"
        y_control.orientation.w = 1.0
        y_control.orientation.x = 0.0
        y_control.orientation.y = 0.0
        y_control.orientation.z = 1.0
        y_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(y_control)
        
        # Z-axis (blue)
        z_control = InteractiveMarkerControl()
        z_control.name = "move_z"
        z_control.orientation.w = 1.0
        z_control.orientation.x = 0.0
        z_control.orientation.y = 1.0
        z_control.orientation.z = 0.0
        z_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(z_control)
        
        # Insert marker into server
        self.server.insert(int_marker, feedback_callback=self.marker_feedback_callback)
        
        self.get_logger().info('Created interactive IK target marker')
    
    def marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):
        """Handle marker feedback (user dragging the marker)"""
        
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Get new position
            new_x = feedback.pose.position.x
            new_y = feedback.pose.position.y
            new_z = feedback.pose.position.z
            
            # Update target position
            self.target_x = new_x
            self.target_y = new_y
            self.target_z = new_z
            
            # Solve IK and move arm
            self.solve_and_move()
        
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # Final position after drag
            self.get_logger().info(
                f'Target position set: ({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f})'
            )
    
    def solve_and_move(self):
        """Solve inverse kinematics and send joint commands"""
        
        # Solve IK
        joint_angles = self.ik_solver.solve_ik(
            self.target_x, 
            self.target_y, 
            self.target_z,
            pitch=self.target_pitch,
            roll=self.target_roll,
            gripper_angle=self.gripper_angle
        )
        
        if joint_angles is None:
            self.get_logger().warn(
                f'Position ({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f}) is unreachable'
            )
            return
        
        # Map IK solver joint names to our array order
        ik_to_idx = {
            'base': 0,
            'shoulder': 1,
            'elbow': 2,
            'forearm': 3,  # wrist pitch
            'wrist': 4,    # wrist roll
            'gripper': 5
        }
        
        # Build joint positions array
        positions = [90.0] * 6
        for ik_joint, idx in ik_to_idx.items():
            if ik_joint in joint_angles:
                positions[idx] = float(joint_angles[ik_joint])
        
        # Publish to hardware
        msg = Float32MultiArray()
        msg.data = positions
        self.joint_pub.publish(msg)
        
        # Publish to RViz (joint_states)
        # Convert to radians for RViz visualization
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        
        # Build joint positions with proper conversion for RViz
        rviz_positions = []
        for i, pos in enumerate(positions):
            if i == 2:  # Elbow joint - invert for RViz
                rviz_positions.append(float(math.radians(90.0 - pos)))
            else:
                rviz_positions.append(float(math.radians(pos - 90.0)))
        
        joint_state_msg.position = rviz_positions
        self.joint_state_pub.publish(joint_state_msg)
        
        self.get_logger().debug(f'IK solution applied: {positions}')
    
    def timer_callback(self):
        """Periodic timer callback - keeps marker server alive"""
        pass  # The server handles its own updates


def main(args=None):
    rclpy.init(args=args)
    
    node = InteractiveIKMarker()
    
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
