#!/usr/bin/env python3
"""
Simple Gripper Control Service
Provides a ROS2 service to open or close the gripper.

Service: /gripper/control
Type: std_srvs/SetBool
  - request.data = true:  Close gripper
  - request.data = false: Open gripper

This is a placeholder implementation. You should replace the `move_gripper`
method with the actual code required to control your hardware gripper,
e.g., by publishing to a servo topic or calling a hardware driver.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import time

class GripperControlService(Node):
    """A service to open and close the robotic arm's gripper."""

    def __init__(self):
        super().__init__('gripper_control_service')
        
        # --- Parameters for Gripper Joint Positions ---
        # These should match the PWM or signal values your servo controller expects.
        self.declare_parameter('gripper_open_value', 75.0)
        self.declare_parameter('gripper_close_value', 110.0)
        self.declare_parameter('gripper_hold_value', 90.0)
        self.declare_parameter('gripper_actuation_time', 1.0) # Time to wait for open/close
        
        self.open_val = self.get_parameter('gripper_open_value').value
        self.close_val = self.get_parameter('gripper_close_value').value
        self.hold_val = self.get_parameter('gripper_hold_value').value
        self.actuation_time = self.get_parameter('gripper_actuation_time').value

        # --- Service ---
        self.srv = self.create_service(
            SetBool,
            '/gripper/control',
            self.handle_gripper_request
        )
        
        # --- Publisher ---
        # This assumes your gripper is controlled by a controller
        # that accepts a Float64MultiArray.
        # For a single servo, it might expect one value in the array.
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands', # Check this topic name
            10
        )

        self.get_logger().info(
            'Gripper Control Service is ready.\n'
            f'  Open Value: {self.open_val}\n'
            f'  Close Value: {self.close_val}\n'
            f'  Hold Value: {self.hold_val}'
        )

    def handle_gripper_request(self, request: SetBool.Request, response: SetBool.Response):
        """Callback to handle open/close requests with a sequence."""
        if request.data:
            self.get_logger().info(f'Request received: Close Gripper (sending {self.close_val} then {self.hold_val})')
            # Sequence to close: send CLOSE value, wait, send HOLD value
            self.publish_gripper_command(self.close_val)
            time.sleep(self.actuation_time)
            self.publish_gripper_command(self.hold_val)
            success = True
        else:
            self.get_logger().info(f'Request received: Open Gripper (sending {self.open_val} then {self.hold_val})')
            # Sequence to open: send OPEN value, wait, send HOLD value
            self.publish_gripper_command(self.open_val)
            time.sleep(self.actuation_time)
            self.publish_gripper_command(self.hold_val)
            success = True
            
        response.success = success
        response.message = "Gripper sequence executed."
        return response

    def publish_gripper_command(self, value: float) -> bool:
        """Publishes a raw value to the gripper controller topic."""
        try:
            msg = Float64MultiArray()
            # Assuming a single joint/servo for the gripper.
            msg.data = [value]
            
            self.get_logger().info(f"Publishing gripper command: {msg.data}")
            self.gripper_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish gripper command: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # On shutdown, send the hold command to be safe
        node.get_logger().info(f"Setting gripper to hold position ({node.hold_val}) on exit.")
        node.publish_gripper_command(node.hold_val)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

    def handle_gripper_request(self, request: SetBool.Request, response: SetBool.Response):
        """Callback to handle open/close requests."""
        if request.data:
            self.get_logger().info('Request received: Close Gripper')
            success = self.move_gripper(self.closed_pos)
        else:
            self.get_logger().info('Request received: Open Gripper')
            success = self.move_gripper(self.open_pos)
            
        response.success = success
        response.message = "Gripper command executed."
        return response

    def move_gripper(self, position: float) -> bool:
        """
        Publishes the command to move the gripper.
        
        *** IMPORTANT ***
        This is a placeholder. You MUST adapt this function to your specific
        gripper hardware interface.
        
        Common methods:
        1. Publishing to a `/joint_trajectory_controller/joint_trajectory` topic.
        2. Publishing to a custom topic (like `/gripper_controller/commands`).
        3. Calling another service provided by your hardware driver.
        """
        try:
            msg = Float64MultiArray()
            # Assuming a single joint for the gripper. If you have two,
            # you might need to send [position, position].
            msg.data = [position]
            
            self.get_logger().info(f"Publishing gripper command: {msg.data}")
            self.gripper_pub.publish(msg)
            
            # Give it a moment to actuate
            time.sleep(1.0)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to move gripper: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
