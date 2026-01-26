#!/usr/bin/env python3
"""
Joint State Publisher Node
Publishes current joint positions to /joint_states topic for visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import json
import os


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Current joint positions (in radians)
        self.joint_positions = [0.0] * 6
        self.joint_names = [
            'base',
            'shoulder', 
            'elbow',
            'wrist',
            'wrist_rotate',
            'gripper_base'
        ]
        
        # Subscriber to joint commands to track current state
        self.create_subscription(
            JointState,
            'joint_commands',
            self.command_callback,
            10
        )
        
        # Timer to publish at fixed rate
        publish_rate = self.declare_parameter('publish_rate', 10.0).value
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_states)
        
        # Try to load last known position
        self.load_last_position()
        
        self.get_logger().info('Joint State Publisher started')

    def command_callback(self, msg):
        """Update internal state when new commands are received"""
        if len(msg.position) == 6:
            self.joint_positions = list(msg.position)

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = [0.0] * 6  # Not tracking velocity yet
        msg.effort = [0.0] * 6    # Not tracking effort yet
        
        self.publisher.publish(msg)

    def load_last_position(self):
        """Load last known position from file"""
        try:
            position_file = os.path.join(
                os.path.dirname(__file__),
                'last_position.json'
            )
            if os.path.exists(position_file):
                with open(position_file, 'r') as f:
                    data = json.load(f)
                    if 'angles' in data and len(data['angles']) == 6:
                        # Convert degrees to radians
                        self.joint_positions = [
                            angle * 3.14159 / 180.0 for angle in data['angles']
                        ]
                        self.get_logger().info(f'Loaded last position: {data["angles"]}°')
        except Exception as e:
            self.get_logger().warn(f'Could not load last position: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
