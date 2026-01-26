from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    arm_moveit_config_pkg = get_package_share_directory('arm_moveit_config')
    
    urdf_file = '/workspace/robot_urdf/robot_fixed.urdf'
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
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
    
    # Trajectory execution config
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
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
        'use_sim_time': use_sim_time,
    }
    
    # Add controller configuration
    moveit_config.update(controller_config)
    moveit_config.update(trajectory_execution)
    
    if os.path.exists(ompl_planning_yaml):
        with open(ompl_planning_yaml, 'r') as f:
            ompl_config = yaml.safe_load(f)
            if ompl_config:
                moveit_config.update(ompl_config)
    
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config],
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        robot_state_publisher,
        move_group_node,
    ])
