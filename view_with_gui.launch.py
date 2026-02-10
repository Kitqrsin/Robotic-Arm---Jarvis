#!/usr/bin/env python3
"""
Launch file for RViz with GUI control, MoveIt kinematics (optional), and Interactive IK Marker
Uses the arm_gui with 3D visualization and MoveIt for accurate IK control (if available)
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Check if MoveIt is available
MOVEIT_AVAILABLE = False
try:
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory('moveit_ros_move_group')
    MOVEIT_AVAILABLE = True
except Exception:
    pass

def generate_launch_description():
    # Path to your OnShape 3D model URDF with detailed meshes (fixed paths)
    urdf_file = '/workspace/robot_urdf/robot_fixed.urdf'
    
    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Base nodes (always launched)
    nodes = [
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
        
        # Arm GUI Controller with 3D visualization and camera feed
        Node(
            package='arm_controller',
            executable='arm_gui',
            name='arm_gui_controller',
            output='screen'
        ),
        
        # Camera Node - publishes /camera/image_raw for GUI camera feed
        Node(
            package='arm_controller',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'frame_rate': 15.0,
                'width': 640,
                'height': 480,
                'camera_backend': 'auto',
            }]
        )
    ]
    
    # Add MoveIt if available
    if MOVEIT_AVAILABLE:
        from ament_index_python.packages import get_package_share_directory
        
        arm_moveit_config_pkg = get_package_share_directory('arm_moveit_config')
        
        srdf_file = os.path.join(arm_moveit_config_pkg, 'config', 'robot_arm.srdf')
        with open(srdf_file, 'r') as f:
            robot_description_semantic = f.read()
        
        ompl_planning_yaml = os.path.join(arm_moveit_config_pkg, 'config', 'ompl_planning.yaml')
        controllers_yaml = os.path.join(arm_moveit_config_pkg, 'config', 'controllers.yaml')
        
        # Load controller configuration
        controller_config = {}
        if os.path.exists(controllers_yaml):
            with open(controllers_yaml, 'r') as f:
                controller_config = yaml.safe_load(f) or {}
        
        # MoveIt configuration
        moveit_config = {
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': {
                'arm': {
                    'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                    'kinematics_solver_search_resolution': 0.005,
                    'kinematics_solver_timeout': 0.05,
                    'kinematics_solver_attempts': 3,
                }
            },
            'use_sim_time': False,
        }
        moveit_config.update(controller_config)
        
        if os.path.exists(ompl_planning_yaml):
            with open(ompl_planning_yaml, 'r') as f:
                ompl_config = yaml.safe_load(f)
                if ompl_config:
                    moveit_config.update(ompl_config)
        
        nodes.insert(0, LogInfo(msg="MoveIt2 detected - launching with IK services"))
        nodes.append(
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                output='screen',
                parameters=[moveit_config],
            )
        )
    else:
        nodes.insert(0, LogInfo(msg="MoveIt2 not installed - using custom geometric IK solver"))
    
    return LaunchDescription(nodes)
