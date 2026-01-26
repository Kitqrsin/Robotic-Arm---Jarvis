#!/usr/bin/env python3
"""
Trajectory Executor Node
Bridges MoveIt trajectory commands to hardware servo control
Executes multi-waypoint trajectories by feeding individual points to servo_node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, Float32, Bool
from sensor_msgs.msg import JointState

import time
import math
import threading


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        
        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Joint names matching your arm configuration
        self.joint_names = [
            'base',
            'shoulder', 
            'elbow',
            'wrist',
            'wrist_rotate',
            'gripper'
        ]
        
        # Current joint positions (in degrees)
        self.current_positions = [90.0] * 6
        self.position_lock = threading.Lock()
        
        # Trajectory execution state
        self.executing = False
        self.execution_lock = threading.Lock()
        self.should_cancel = False
        
        # Speed control (1-100%)
        self.current_speed = 50.0
        
        # Emergency stop state
        self.emergency_stop_active = False
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )
        
        self.speed_pub = self.create_publisher(
            Float32,
            '/servo_speed',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action server for MoveIt trajectory execution
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Trajectory Executor initialized')
        self.get_logger().info(f'Action server: /arm_controller/follow_joint_trajectory')
        self.get_logger().info(f'Publishing commands to: /joint_commands')
    
    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        try:
            with self.position_lock:
                # Convert radians to degrees if needed
                if len(msg.position) >= 6:
                    # Check if positions are in radians (typically < 7)
                    if all(abs(pos) < 7 for pos in msg.position[:6]):
                        self.current_positions = [math.degrees(pos) + 90.0 for pos in msg.position[:6]]
                    else:
                        self.current_positions = list(msg.position[:6])
        except Exception as e:
            self.get_logger().error(f'Joint state callback error: {e}')
    
    def estop_callback(self, msg):
        """Handle emergency stop state changes"""
        self.emergency_stop_active = msg.data
        if msg.data:
            self.get_logger().warning('Emergency stop ACTIVE - Canceling trajectory')
            self.should_cancel = True
    
    def goal_callback(self, goal_request):
        """Accept or reject trajectory goals"""
        if self.emergency_stop_active:
            self.get_logger().warning('Trajectory goal rejected - Emergency stop active')
            return GoalResponse.REJECT
        
        with self.execution_lock:
            if self.executing:
                self.get_logger().warning('Trajectory goal rejected - Already executing')
                return GoalResponse.REJECT
        
        self.get_logger().info(f'Trajectory goal accepted: {len(goal_request.trajectory.points)} waypoints')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle trajectory cancellation requests"""
        self.get_logger().info('Trajectory cancellation requested')
        self.should_cancel = True
        return CancelResponse.ACCEPT
    
    async def execute_trajectory_callback(self, goal_handle: ServerGoalHandle):
        """Execute the trajectory by sending waypoints sequentially"""
        self.get_logger().info('Starting trajectory execution')
        
        with self.execution_lock:
            self.executing = True
            self.should_cancel = False
        
        try:
            trajectory = goal_handle.request.trajectory
            
            # Validate trajectory
            if not trajectory.points:
                self.get_logger().error('Empty trajectory received')
                goal_handle.abort()
                return FollowJointTrajectory.Result(error_code=-1, error_string='Empty trajectory')
            
            # Set speed for trajectory execution (can be adjusted based on trajectory timing)
            speed_msg = Float32()
            speed_msg.data = self.current_speed
            self.speed_pub.publish(speed_msg)
            
            # Execute each waypoint
            feedback_msg = FollowJointTrajectory.Feedback()
            start_time = self.get_clock().now()
            
            for i, point in enumerate(trajectory.points):
                # Check for cancellation or emergency stop
                if self.should_cancel or self.emergency_stop_active:
                    self.get_logger().warning('Trajectory execution canceled')
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result(
                        error_code=-2,
                        error_string='Trajectory canceled'
                    )
                
                # Convert trajectory point to joint commands
                joint_commands = self._convert_trajectory_point(point, trajectory.joint_names)
                
                if joint_commands is None:
                    self.get_logger().error(f'Failed to convert waypoint {i}')
                    goal_handle.abort()
                    return FollowJointTrajectory.Result(
                        error_code=-3,
                        error_string=f'Invalid waypoint {i}'
                    )
                
                # Send command to hardware
                cmd_msg = Float32MultiArray()
                cmd_msg.data = joint_commands
                self.joint_cmd_pub.publish(cmd_msg)
                
                # Publish feedback
                feedback_msg.header.stamp = self.get_clock().now().to_msg()
                feedback_msg.joint_names = self.joint_names
                with self.position_lock:
                    feedback_msg.actual.positions = [math.radians(pos - 90.0) for pos in self.current_positions]
                feedback_msg.desired.positions = point.positions
                feedback_msg.error.positions = [
                    actual - desired 
                    for actual, desired in zip(feedback_msg.actual.positions, point.positions)
                ]
                
                # Calculate time from start
                time_from_start_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                feedback_msg.actual.time_from_start = point.time_from_start
                
                goal_handle.publish_feedback(feedback_msg)
                
                # Wait for the specified duration
                if i < len(trajectory.points) - 1:
                    next_time = trajectory.points[i + 1].time_from_start.sec + \
                               trajectory.points[i + 1].time_from_start.nanosec / 1e9
                    wait_time = next_time - time_from_start_sec
                    
                    if wait_time > 0:
                        # Sleep in small increments to allow cancellation
                        sleep_resolution = 0.05  # 50ms
                        slept = 0.0
                        while slept < wait_time:
                            if self.should_cancel or self.emergency_stop_active:
                                break
                            time.sleep(min(sleep_resolution, wait_time - slept))
                            slept += sleep_resolution
                
                self.get_logger().debug(f'Executed waypoint {i+1}/{len(trajectory.points)}')
            
            # Trajectory completed successfully
            self.get_logger().info('Trajectory execution completed successfully')
            goal_handle.succeed()
            
            return FollowJointTrajectory.Result(
                error_code=0,
                error_string='Trajectory executed successfully'
            )
            
        except Exception as e:
            self.get_logger().error(f'Trajectory execution failed: {str(e)}')
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=-4,
                error_string=f'Execution error: {str(e)}'
            )
        
        finally:
            with self.execution_lock:
                self.executing = False
                self.should_cancel = False
    
    def _convert_trajectory_point(self, point: JointTrajectoryPoint, joint_names: list) -> list:
        """
        Convert a trajectory point to hardware joint commands (degrees)
        
        Args:
            point: JointTrajectoryPoint with positions in radians
            joint_names: List of joint names in the trajectory
            
        Returns:
            List of joint angles in degrees for hardware, or None if conversion fails
        """
        try:
            if len(point.positions) != len(joint_names):
                self.get_logger().error(
                    f'Position count mismatch: {len(point.positions)} vs {len(joint_names)}'
                )
                return None
            
            # Create mapping from trajectory joint names to our joint indices
            joint_commands = [90.0] * 6  # Default to neutral position
            
            for i, name in enumerate(joint_names):
                # Find matching joint in our configuration
                try:
                    # MoveIt may use different joint naming (e.g., from URDF)
                    # Map common variations
                    if 'base' in name.lower():
                        joint_idx = 0
                    elif 'shoulder' in name.lower():
                        joint_idx = 1
                    elif 'elbow' in name.lower():
                        joint_idx = 2
                    elif 'wrist' in name.lower() and 'rotate' not in name.lower():
                        joint_idx = 3
                    elif 'wrist_rotate' in name.lower() or 'roll' in name.lower():
                        joint_idx = 4
                    elif 'gripper' in name.lower():
                        joint_idx = 5
                    else:
                        self.get_logger().warning(f'Unknown joint name: {name}')
                        continue
                    
                    # Convert radians to degrees and adjust for hardware offset
                    # RViz: 0 rad = 90°, so hardware_degrees = radians_to_degrees + 90
                    angle_deg = math.degrees(point.positions[i]) + 90.0
                    
                    # Clamp to safe range
                    angle_deg = max(0.0, min(180.0, angle_deg))
                    
                    joint_commands[joint_idx] = angle_deg
                    
                except Exception as e:
                    self.get_logger().error(f'Error mapping joint {name}: {e}')
                    return None
            
            return joint_commands
            
        except Exception as e:
            self.get_logger().error(f'Trajectory point conversion error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = TrajectoryExecutor()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
