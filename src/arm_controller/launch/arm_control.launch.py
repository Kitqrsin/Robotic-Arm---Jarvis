#!/usr/bin/env python3
"""
Launch file for RoboARM control system
Starts all necessary nodes for controlling the 6-DOF robotic arm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (no hardware required)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Launch configuration
    use_sim = LaunchConfiguration('use_sim')
    log_level = LaunchConfiguration('log_level')

    # Servo controller node
    servo_node = Node(
        package='arm_controller',
        executable='servo_node',
        name='arm_servo_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim,
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Joint state publisher node (publishes current joint angles)
    joint_state_publisher = Node(
        package='arm_controller',
        executable='joint_state_publisher_node',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,  # Hz
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        use_sim_arg,
        log_level_arg,
        LogInfo(msg='Starting RoboARM control system...'),
        servo_node,
        # joint_state_publisher,  # Uncomment when node is created
    ])
