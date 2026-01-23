#!/usr/bin/env python3
"""
Launch file for RViz with GUI control
Uses the arm_gui instead of joint_state_publisher_gui
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to your OnShape 3D model URDF with detailed meshes (fixed paths)
    urdf_file = '/workspace/robot_urdf/robot_fixed.urdf'
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Robot State Publisher - converts joint_states to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/workspace/robot.rviz']
        ),
        
        # Arm GUI Controller
        Node(
            package='arm_controller',
            executable='arm_gui',
            name='arm_gui_controller',
            output='screen'
        )
    ])
