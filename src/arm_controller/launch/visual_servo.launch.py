#!/usr/bin/env python3
"""
Visual Servoing Launch File — Eye-in-Hand face tracking

Starts the nodes required for visual servoing:
  1. camera_node              — captures frames from Pi Camera V2
  2. vision_error_publisher   — detects faces, publishes /vision/error
  3. servo_node               — drives the PCA9685 servos
  4. arm_gui                  — Tkinter GUI with 👁 Servo toggle

Usage:
    ros2 launch arm_controller visual_servo.launch.py

    # Override target face area fraction (default 0.08 = 8% of frame):
    ros2 launch arm_controller visual_servo.launch.py target_area_frac:=0.10

    # Adjust detection interval (process every Nth camera frame):
    ros2 launch arm_controller visual_servo.launch.py detect_interval:=3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ---- #
    target_area_arg = DeclareLaunchArgument(
        'target_area_frac', default_value='0.08',
        description='Desired face-area fraction (0-1) for depth control'
    )
    detect_interval_arg = DeclareLaunchArgument(
        'detect_interval', default_value='2',
        description='Run face detection every Nth camera frame'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logging level'
    )

    log_level = LaunchConfiguration('log_level')

    # ---- Nodes ---- #

    camera_node = Node(
        package='arm_controller',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'frame_rate': 15.0,
            'width': 640,
            'height': 480,
            'camera_backend': 'auto',
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    vision_error_node = Node(
        package='arm_controller',
        executable='vision_error_publisher',
        name='vision_error_publisher',
        output='screen',
        parameters=[{
            'target_area_frac': LaunchConfiguration('target_area_frac'),
            'detect_interval': LaunchConfiguration('detect_interval'),
            'min_face_size': 40,
            'scale_factor': 1.2,
            'min_neighbours': 5,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    servo_node = Node(
        package='arm_controller',
        executable='servo_node',
        name='arm_servo_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    gui_node = Node(
        package='arm_controller',
        executable='arm_gui',
        name='arm_control_gui',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        target_area_arg,
        detect_interval_arg,
        log_level_arg,
        LogInfo(msg='Starting Eye-in-Hand Visual Servoing pipeline...'),
        camera_node,
        vision_error_node,
        servo_node,
        gui_node,
    ])
