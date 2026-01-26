#!/usr/bin/env python3
"""
Simple Terminal Controller for Robot Arm
Control joints interactively from terminal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        self.publisher = self.create_publisher(Float32MultiArray, '/joint_commands', 10)
        self.positions = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist', 'Wrist_Rotate', 'Gripper']
        
    def send_command(self):
        msg = Float32MultiArray()
        msg.data = self.positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent: {[f"{p:.1f}" for p in self.positions]}')

def main():
    rclpy.init()
    controller = SimpleArmController()
    
    print("\n" + "="*60)
    print("  ROBOT ARM CONTROLLER")
    print("="*60)
    print("\nCommands:")
    print("  1-6: Select joint (Base, Shoulder, Elbow, Wrist, WristRot, Gripper)")
    print("  +/-: Increase/decrease selected joint by 5°")
    print("  h: Go to home (all 90°)")
    print("  s: Show current positions")
    print("  q: Quit")
    print("="*60 + "\n")
    
    selected_joint = 0
    
    controller.send_command()  # Send initial position
    
    while True:
        controller.positions[selected_joint]
        print(f"\nSelected: {controller.joint_names[selected_joint]} = {controller.positions[selected_joint]:.1f}°")
        command = input("Command: ").strip().lower()
        
        if command == 'q':
            print("Exiting...")
            break
        elif command == 'h':
            controller.positions = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
            controller.send_command()
            print("Moved to HOME position")
        elif command == 's':
            print("\nCurrent Positions:")
            for i, name in enumerate(controller.joint_names):
                print(f"  {name:12s}: {controller.positions[i]:6.1f}°")
        elif command in ['1', '2', '3', '4', '5', '6']:
            selected_joint = int(command) - 1
            print(f"Selected: {controller.joint_names[selected_joint]}")
        elif command == '+':
            controller.positions[selected_joint] = min(180.0, controller.positions[selected_joint] + 5.0)
            controller.send_command()
        elif command == '-':
            controller.positions[selected_joint] = max(0.0, controller.positions[selected_joint] - 5.0)
            controller.send_command()
        else:
            print("Unknown command. Use: 1-6, +, -, h, s, q")
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted")
