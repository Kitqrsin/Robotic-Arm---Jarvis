#!/usr/bin/env python3
"""
MoveIt Pick and Place Node
Uses MoveIt2 `moveit_commander` to pick up a detected object and place it.

- Listens for a target TF frame from the vision node.
- Adds the object to the planning scene for collision avoidance.
- Defines and executes a multi-stage pick operation (approach, grasp, lift).
- Defines and executes a multi-stage place operation.
- Communicates with a gripper control service to open/close the gripper.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import CollisionObject, Grasp, GripperTranslation
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import sys
import time
import math

# Gripper control - replace with your actual service/action if different
from std_srvs.srv import SetBool

# Assumed constants - should match your MoveIt config
ARM_GROUP_NAME = "arm_controller"
GRIPPER_GROUP_NAME = "gripper_controller"
GRIPPER_FRAME = "gripper_link"  # The frame of the end-effector
BASE_FRAME = "base_link"
PLANNER_ID = "RRTConnectkConfigDefault"


class MoveItPickPlaceNode(Node):
    """Node to perform a full pick and place sequence using MoveIt."""

    def __init__(self):
        super().__init__('moveit_pick_place_node')

        # --- Parameters ---
        self.declare_parameter('target_object_tf', 'detected_cup_1')
        self.declare_parameter('object_height', 0.10)  # Approx height for collision box
        self.declare_parameter('object_width', 0.08)   # Approx width for collision box
        self.declare_parameter('place_x', 0.0)
        self.declare_parameter('place_y', -0.3)
        self.declare_parameter('place_z', 0.15)

        self.target_object_tf = self.get_parameter('target_object_tf').value
        self.object_height = self.get_parameter('object_height').value
        self.object_width = self.get_parameter('object_width').value
        self.place_pose = Pose()
        self.place_pose.position.x = self.get_parameter('place_x').value
        self.place_pose.position.y = self.get_parameter('place_y').value
        self.place_pose.position.z = self.get_parameter('place_z').value
        self.place_pose.orientation.w = 1.0

        # --- MoveIt Setup ---
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.robot_commander = RobotCommander()
            self.planning_scene_interface = PlanningSceneInterface()
            self.move_group = MoveGroupCommander(ARM_GROUP_NAME)
            self.gripper_group = MoveGroupCommander(GRIPPER_GROUP_NAME)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt: {e}")
            self.get_logger().error("Is the MoveIt launch file running? `ros2 launch arm_moveit_config moveit.launch.py`")
            rclpy.shutdown()
            sys.exit(1)

        self.move_group.set_planner_id(PLANNER_ID)
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.allow_replanning(True)

        # --- TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Gripper Control Client ---
        # This assumes a simple service to open (false) and close (true) the gripper
        self.gripper_client = self.create_client(SetBool, '/gripper/control')
        while not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Gripper service not available, waiting...')
        
        self.get_logger().info(
            f'MoveIt Pick and Place node ready.\n'
            f'  Arm Group: {ARM_GROUP_NAME}\n'
            f'  Gripper Group: {GRIPPER_GROUP_NAME}\n'
            f'  Target TF: {self.target_object_tf}'
        )

        # --- Main Execution Timer ---
        # Wait a bit for everything to initialize, then run the sequence
        self.create_timer(2.0, self.run_pick_and_place, oneshot=True)

    def run_pick_and_place(self):
        """The main state machine for the pick and place operation."""
        self.get_logger().info("--- Starting Pick and Place Sequence ---")

        # 1. Look up the object's transform
        try:
            self.get_logger().info(f"Waiting for transform from '{BASE_FRAME}' to '{self.target_object_tf}'...")
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, self.target_object_tf, rclpy.time.Time(), timeout=Duration(seconds=5.0)
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get transform for '{self.target_object_tf}': {e}")
            self.get_logger().error("Is the vision_tf_broadcaster running and detecting the object?")
            return

        target_pose = Pose()
        target_pose.position.x = transform.transform.translation.x
        target_pose.position.y = transform.transform.translation.y
        target_pose.position.z = transform.transform.translation.z
        target_pose.orientation.w = 1.0 # Assume top-down grasp

        self.get_logger().info(f"Object found at: [{target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f}]")

        # 2. Add the object to the planning scene
        self.add_collision_object(self.target_object_tf, target_pose)
        
        # 3. Open the gripper
        self.set_gripper(False) # False = Open

        # 4. Execute the pick
        pick_success = self.execute_pick(self.target_object_tf, target_pose)
        if not pick_success:
            self.get_logger().error("Pick operation failed. Aborting.")
            self.cleanup_scene()
            return

        self.get_logger().info("--- Pick Successful! ---")

        # 5. Execute the place
        place_success = self.execute_place()
        if not place_success:
            self.get_logger().error("Place operation failed.")
        else:
            self.get_logger().info("--- Place Successful! ---")

        # 6. Cleanup
        self.cleanup_scene()
        self.get_logger().info("--- Sequence Complete ---")
        rclpy.shutdown()

    def execute_pick(self, object_name, object_pose):
        """Use MoveIt's pick() interface to grasp the object."""
        self.get_logger().info(f"Attempting to pick object '{object_name}'")

        grasps = []
        grasp = Grasp()

        # --- Grasp Pose ---
        # This is the pose of the GRIPPER_FRAME, not the arm.
        grasp.grasp_pose.header.frame_id = BASE_FRAME
        grasp.grasp_pose.pose = object_pose
        # Offset the grasp pose slightly above the object's center
        grasp.grasp_pose.pose.position.z += self.object_height / 2.0 + 0.02
        # Set orientation for a top-down grasp (gripper pointing down)
        q = self.quaternion_from_euler(0, math.pi, 0) # Pitch by 180 deg
        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.z = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]

        # --- Pre-Grasp Approach ---
        grasp.pre_grasp_approach.direction.header.frame_id = GRIPPER_FRAME
        grasp.pre_grasp_approach.direction.vector.z = 1.0  # Approach from above (along gripper's Z)
        grasp.pre_grasp_approach.min_distance = 0.08
        grasp.pre_grasp_approach.desired_distance = 0.12

        # --- Gripper State Before Grasp ---
        grasp.pre_grasp_posture = self.make_gripper_posture(is_closed=False)

        # --- Gripper State During Grasp ---
        grasp.grasp_posture = self.make_gripper_posture(is_closed=True)

        # --- Post-Grasp Retreat ---
        grasp.post_grasp_retreat.direction.header.frame_id = BASE_FRAME
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # Retreat upwards (along base Z)
        grasp.post_grasp_retreat.min_distance = 0.1
        grasp.post_grasp_retreat.desired_distance = 0.15
        
        # Tell MoveIt that the object should be attached to the gripper
        grasp.allowed_touch_objects = [object_name]
        
        grasps.append(grasp)

        # --- Execute Pick ---
        # The pick interface can take a long time to plan
        self.move_group.set_support_surface_name("table") # Assume a table surface
        result = self.move_group.pick(object_name, grasps)

        if result:
            self.get_logger().info("Pick executed successfully!")
        else:
            self.get_logger().error("Pick failed!")
        
        return result

    def execute_place(self):
        """Use MoveIt's place() interface to set the object down."""
        self.get_logger().info("Attempting to place object.")

        place_locations = []
        place_location = PoseStamped()
        place_location.header.frame_id = BASE_FRAME
        
        # Define the pose for placing the object
        place_location.pose = self.place_pose
        
        # Add pre-place approach and post-place retreat
        # (Similar to the pick operation)
        
        # Execute Place
        result = self.move_group.place(self.target_object_tf, place_location)

        if result:
            self.get_logger().info("Place executed successfully!")
        else:
            self.get_logger().error("Place failed!")
            
        return result

    def add_collision_object(self, object_id, object_pose):
        """Add a cylindrical collision object to the planning scene."""
        self.get_logger().info(f"Adding collision object '{object_id}' to planning scene.")
        
        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_FRAME
        collision_object.id = object_id
        
        # Define the primitive (shape) of the object
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [self.object_height, self.object_width / 2.0] # height, radius

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(object_pose)
        collision_object.operation = CollisionObject.ADD
        
        self.planning_scene_interface.add_object(collision_object)
        time.sleep(1) # Give time for the scene to update

    def set_gripper(self, closed: bool):
        """Send a request to the gripper control service."""
        self.get_logger().info(f"Setting gripper to {'closed' if closed else 'open'}")
        req = SetBool.Request()
        req.data = closed
        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Gripper action successful.")
            else:
                self.get_logger().warn("Gripper action failed.")
        else:
            self.get_logger().error("Gripper service call failed.")
        time.sleep(1.0) # Wait for gripper to actuate

    def make_gripper_posture(self, is_closed: bool):
        """Create a JointTrajectory message for the gripper state."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_group.get_active_joints()
        
        point = JointTrajectoryPoint()
        if is_closed:
            # These values need to match your gripper's closed joint positions
            point.positions = self.gripper_group.get_named_target_values("closed")['positions']
        else:
            # These values need to match your gripper's open joint positions
            point.positions = self.gripper_group.get_named_target_values("open")['positions']
        point.time_from_start = Duration(seconds=1).to_msg()
        
        trajectory.points.append(point)
        return trajectory

    def cleanup_scene(self):
        """Remove the target object from the planning scene."""
        self.get_logger().info(f"Removing '{self.target_object_tf}' from scene.")
        self.planning_scene_interface.remove_collision_object(self.target_object_tf)
        time.sleep(0.5)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """Converts euler roll, pitch, yaw to a quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr  # x
        q[1] = sy * cp * sr + cy * sp * cr  # y
        q[2] = sy * cp * cr - cy * sp * sr  # z
        q[3] = cy * cp * cr + sy * sp * sr  # w
        return q

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPickPlaceNode()
    try:
        # The main logic is timer-based, so we just spin.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down pick-and-place node.")
        node.cleanup_scene()
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
