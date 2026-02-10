#!/usr/bin/env python3
"""
Vision + Pick-and-Place Launch File

Starts the full pipeline:
  1. camera_node        — captures frames from Pi Camera V2
  2. object_detector_node — runs YOLOv8 detection
  3. servo_node          — controls the arm hardware
  4. pick_place_node     — orchestrates pick-and-handover

Usage:
    ros2 launch arm_controller vision_pick_place.launch.py

    # With auto-pick enabled:
    ros2 launch arm_controller vision_pick_place.launch.py auto_pick:=true

    # Target a specific class:
    ros2 launch arm_controller vision_pick_place.launch.py target_class:=cup
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch arguments ---- #
    auto_pick_arg = DeclareLaunchArgument(
        'auto_pick', default_value='false',
        description='Automatically pick detected objects'
    )
    target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='',
        description='Only pick this YOLO class name (empty = any)'
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence', default_value='0.45',
        description='YOLO detection confidence threshold'
    )
    calibration_arg = DeclareLaunchArgument(
        'calibration_file', default_value='',
        description='Path to camera_calibration.json'
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

    detector_node = Node(
        package='arm_controller',
        executable='object_detector_node',
        name='object_detector_node',
        output='screen',
        parameters=[{
            'model': 'yolov8n.pt',
            'confidence': LaunchConfiguration('confidence'),
            'detect_interval': 3,         # process every 3rd frame
            'target_classes': LaunchConfiguration('target_class'),
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

    pick_place = Node(
        package='arm_controller',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[{
            'auto_pick': LaunchConfiguration('auto_pick'),
            'target_class': LaunchConfiguration('target_class'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'move_speed': 30.0,
            'gripper_open': 60.0,
            'gripper_closed': 150.0,
            'handover_hold_time': 3.0,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        auto_pick_arg,
        target_class_arg,
        confidence_arg,
        calibration_arg,
        log_level_arg,
        LogInfo(msg='=== Starting Vision + Pick-and-Place Pipeline ==='),
        camera_node,
        detector_node,
        servo_node,
        pick_place,
    ])
