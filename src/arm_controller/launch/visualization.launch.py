#!/usr/bin/env python3
"""
Launch file for visualizing the robot arm in RViz
Starts robot_state_publisher with URDF and joint_state_publisher
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get URDF file path
    pkg_share = get_package_share_directory('arm_controller')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf')
    
    # Alternative: use xacro if you convert to .xacro format later
    # robot_description = Command(['xacro ', urdf_file])
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher GUI for manual control'
    )

    use_gui = LaunchConfiguration('use_gui')

    # Robot state publisher - publishes TF transforms from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )

    # Joint state publisher GUI - allows manual joint control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # RViz with custom config
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
