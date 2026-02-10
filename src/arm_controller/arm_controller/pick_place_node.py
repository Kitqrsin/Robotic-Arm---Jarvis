#!/usr/bin/env python3
"""
Pick & Place Node - Orchestrates detect → approach → grab → handover

Subscribes to /detected_objects (from object_detector_node),
converts pixel detections to arm-frame (x, y, z) using a calibrated
homography, then uses the IK solver to generate joint commands.

Workflow:
  1. Wait for a detection
  2. Move arm above the object  (pre-grasp)
  3. Lower arm to grasp height   (grasp approach)
  4. Close gripper               (grasp)
  5. Lift object                  (post-grasp)
  6. Move to handover position   (present)
  7. Wait, then open gripper     (release)
  8. Return to home              (reset)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String, Bool
import numpy as np
import json
import time
import math
import os

try:
    from .ik_solver import ArmIKSolver
except ImportError:
    from ik_solver import ArmIKSolver


# ====================================================================== #
#  Default calibration — overridden at runtime by calibration file         #
# ====================================================================== #
# Homography maps normalised image coords (0-1) → arm workspace (metres)
# You MUST run camera_calibration.py first to generate a proper matrix.
# These defaults assume camera centred ~30 cm above workspace looking down.
DEFAULT_HOMOGRAPHY = np.array([
    [ 0.30,  0.00, -0.15],
    [ 0.00,  0.30, -0.15],
    [ 0.00,  0.00,  1.00],
], dtype=np.float64)


class PickPlaceNode(Node):
    """Detects an object, picks it up, and hands it to the user."""

    # -- State machine states --
    IDLE        = 'idle'
    APPROACHING = 'approaching'
    LOWERING    = 'lowering'
    GRASPING    = 'grasping'
    LIFTING     = 'lifting'
    HANDING     = 'handing'
    RELEASING   = 'releasing'
    RETURNING   = 'returning'

    def __init__(self):
        super().__init__('pick_place_node')

        # --- Parameters ---
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('table_z', 0.02)          # Table surface height (m)
        self.declare_parameter('pre_grasp_z', 0.12)       # Hover height above table (m)
        self.declare_parameter('grasp_z', 0.03)           # Gripper height at grab (m)
        self.declare_parameter('post_grasp_z', 0.18)      # Lift height (m)
        self.declare_parameter('handover_x', 0.10)        # Handover position X (m)
        self.declare_parameter('handover_y', 0.00)        # Handover position Y (m)
        self.declare_parameter('handover_z', 0.22)        # Handover position Z (m)
        self.declare_parameter('gripper_open', 60.0)      # Gripper open angle
        self.declare_parameter('gripper_closed', 150.0)   # Gripper closed angle
        self.declare_parameter('move_speed', 30.0)        # Movement speed (1-100)
        self.declare_parameter('settle_time', 0.5)        # Seconds to wait after move
        self.declare_parameter('handover_hold_time', 3.0) # Seconds to hold at handover
        self.declare_parameter('auto_pick', False)        # Auto-pick on detection
        self.declare_parameter('target_class', '')        # Prefer this class (empty=any)

        self.table_z = self.get_parameter('table_z').value
        self.pre_grasp_z = self.get_parameter('pre_grasp_z').value
        self.grasp_z = self.get_parameter('grasp_z').value
        self.post_grasp_z = self.get_parameter('post_grasp_z').value
        self.handover = {
            'x': self.get_parameter('handover_x').value,
            'y': self.get_parameter('handover_y').value,
            'z': self.get_parameter('handover_z').value,
        }
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        self.move_speed = self.get_parameter('move_speed').value
        self.settle_time = self.get_parameter('settle_time').value
        self.handover_hold = self.get_parameter('handover_hold_time').value
        self.auto_pick = self.get_parameter('auto_pick').value
        self.target_class = self.get_parameter('target_class').value

        # --- IK solver ---
        self.ik = ArmIKSolver()

        # --- Homography (pixel-norm → arm metres) ---
        self.homography = DEFAULT_HOMOGRAPHY.copy()
        cal_file = self.get_parameter('calibration_file').value
        if cal_file and os.path.isfile(cal_file):
            self._load_calibration(cal_file)
        else:
            self.get_logger().warn(
                'No calibration file — using default homography. '
                'Run: ros2 run arm_controller camera_calibration'
            )

        # --- State machine ---
        self.state = self.IDLE
        self.current_target = None   # (x, y) in arm frame
        self.current_detection = None

        # --- Publishers ---
        self.joint_pub = self.create_publisher(
            Float32MultiArray, '/joint_commands', 10
        )
        self.speed_pub = self.create_publisher(
            Float32, '/servo_speed', 10
        )
        self.status_pub = self.create_publisher(
            String, '/pick_place/status', 10
        )

        # --- Subscribers ---
        self.detection_sub = self.create_subscription(
            String, '/detected_objects', self.detection_callback, 10
        )
        # Manual trigger: publish 'pick' to /pick_place/command
        self.command_sub = self.create_subscription(
            String, '/pick_place/command', self.command_callback, 10
        )

        # --- State machine timer (runs at 10 Hz) ---
        self.timer = self.create_timer(0.1, self.state_machine_tick)
        self.step_start_time = 0.0

        self._set_speed(self.move_speed)

        self.get_logger().info('Pick & Place node started')
        self.get_logger().info(
            f'  Handover position: ({self.handover["x"]:.2f}, '
            f'{self.handover["y"]:.2f}, {self.handover["z"]:.2f})'
        )
        self.get_logger().info(
            f'  Auto-pick: {self.auto_pick}, '
            f'Target class: {self.target_class or "any"}'
        )

    # ================================================================== #
    #  Calibration                                                         #
    # ================================================================== #
    def _load_calibration(self, path):
        """Load homography matrix from a .npz or .json file."""
        try:
            if path.endswith('.npz'):
                data = np.load(path)
                self.homography = data['homography']
            else:
                with open(path) as f:
                    data = json.load(f)
                self.homography = np.array(data['homography'], dtype=np.float64)
            self.get_logger().info(f'Loaded calibration from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def pixel_norm_to_arm(self, nx, ny):
        """Convert normalised image coords (0-1) to arm-frame metres via homography."""
        pt = np.array([nx, ny, 1.0])
        result = self.homography @ pt
        result /= result[2]  # normalise by w
        return float(result[0]), float(result[1])

    # ================================================================== #
    #  Detection callback                                                  #
    # ================================================================== #
    def detection_callback(self, msg):
        """Receive detections from object_detector_node."""
        if self.state != self.IDLE:
            return  # busy with a pick operation

        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
        except json.JSONDecodeError:
            return

        if not detections:
            return

        # Pick best detection (filter by target class if set)
        chosen = None
        for det in detections:
            if self.target_class and det['class_name'] != self.target_class:
                continue
            chosen = det
            break  # already sorted by confidence

        if chosen is None:
            return

        self.current_detection = chosen

        # Convert pixel centre → arm coordinates
        nx = chosen['center_norm']['x']
        ny = chosen['center_norm']['y']
        arm_x, arm_y = self.pixel_norm_to_arm(nx, ny)
        self.current_target = (arm_x, arm_y)

        self.get_logger().info(
            f"Object: {chosen['class_name']} ({chosen['confidence']:.0%}) "
            f"→ arm ({arm_x:.3f}, {arm_y:.3f}) m"
        )

        # Publish status
        self._publish_status(
            f"detected:{chosen['class_name']}:{arm_x:.3f},{arm_y:.3f}"
        )

        if self.auto_pick:
            self._start_pick()

    # ================================================================== #
    #  Command callback                                                    #
    # ================================================================== #
    def command_callback(self, msg):
        """Handle manual commands: 'pick', 'stop', 'home'."""
        cmd = msg.data.strip().lower()

        if cmd == 'pick':
            if self.current_target:
                self._start_pick()
            else:
                self.get_logger().warn('No object detected yet — cannot pick')
        elif cmd == 'stop':
            self.get_logger().warn('Stopping — returning to home')
            self.state = self.RETURNING
            self.step_start_time = time.time()
        elif cmd == 'home':
            self._go_home()
            self.state = self.IDLE
        else:
            self.get_logger().info(f'Unknown command: {cmd}')

    # ================================================================== #
    #  State machine                                                       #
    # ================================================================== #
    def _start_pick(self):
        """Begin the pick sequence."""
        if self.state != self.IDLE:
            self.get_logger().warn('Already busy — ignoring pick request')
            return
        self.get_logger().info('=== Starting pick & handover sequence ===')
        self.state = self.APPROACHING
        self.step_start_time = time.time()
        self._publish_status('state:approaching')

    def state_machine_tick(self):
        """Run every 100 ms — advance the state machine."""
        elapsed = time.time() - self.step_start_time

        if self.state == self.IDLE:
            return

        elif self.state == self.APPROACHING:
            # Move above object
            x, y = self.current_target
            self._move_to(x, y, self.pre_grasp_z, gripper=self.gripper_open)
            self.state = self.LOWERING
            self.step_start_time = time.time()
            self._publish_status('state:lowering')

        elif self.state == self.LOWERING:
            if elapsed < self.settle_time:
                return  # wait for approach to settle
            # Lower to grasp height
            x, y = self.current_target
            self._move_to(x, y, self.grasp_z, gripper=self.gripper_open)
            self.state = self.GRASPING
            self.step_start_time = time.time()
            self._publish_status('state:grasping')

        elif self.state == self.GRASPING:
            if elapsed < self.settle_time:
                return
            # Close gripper
            self._send_gripper(self.gripper_closed)
            self.state = self.LIFTING
            self.step_start_time = time.time()
            self._publish_status('state:lifting')

        elif self.state == self.LIFTING:
            if elapsed < self.settle_time:
                return
            # Lift object
            x, y = self.current_target
            self._move_to(x, y, self.post_grasp_z, gripper=self.gripper_closed)
            self.state = self.HANDING
            self.step_start_time = time.time()
            self._publish_status('state:handing')

        elif self.state == self.HANDING:
            if elapsed < self.settle_time:
                return
            # Move to handover position
            self._move_to(
                self.handover['x'], self.handover['y'], self.handover['z'],
                gripper=self.gripper_closed
            )
            self.state = self.RELEASING
            self.step_start_time = time.time()
            self._publish_status('state:releasing')

        elif self.state == self.RELEASING:
            if elapsed < self.handover_hold:
                return  # hold object for user to grab
            # Open gripper to release
            self._send_gripper(self.gripper_open)
            self.state = self.RETURNING
            self.step_start_time = time.time()
            self._publish_status('state:returning')

        elif self.state == self.RETURNING:
            if elapsed < self.settle_time:
                return
            # Go back to home
            self._go_home()
            self.state = self.IDLE
            self.current_target = None
            self.current_detection = None
            self.get_logger().info('=== Pick & handover complete ===')
            self._publish_status('state:idle')

    # ================================================================== #
    #  Movement helpers                                                    #
    # ================================================================== #
    def _move_to(self, x, y, z, gripper=None, pitch=0.0):
        """Solve IK and publish joint commands."""
        grip = gripper if gripper is not None else self.gripper_open

        joint_angles = self.ik.solve_ik(x, y, z, pitch=pitch, gripper_angle=grip)

        if joint_angles is None:
            self.get_logger().warn(
                f'IK failed for ({x:.3f}, {y:.3f}, {z:.3f}) — skipping'
            )
            return False

        self._publish_joints(joint_angles)
        return True

    def _go_home(self):
        """Move to safe home position."""
        home = self.ik.home_position
        msg = Float32MultiArray()
        msg.data = [
            float(home['base']),
            float(home['shoulder']),
            float(home['elbow']),
            float(home['forearm']),
            float(home['wrist']),
            float(home['gripper']),
        ]
        self.joint_pub.publish(msg)
        self.get_logger().info('Moving to home position')

    def _send_gripper(self, angle):
        """Send only the gripper angle (keep other joints unchanged)."""
        # We publish a full 6-DOF command; the last value is gripper
        # Re-use current IK home as a base so only gripper changes
        # In practice the arm is already positioned, so we just need
        # to update the gripper via a new command
        self.get_logger().info(f'Gripper → {angle:.0f}°')
        # We re-publish the last IK solution with updated gripper
        # For simplicity, we just re-solve at current handover/target position
        if self.current_target and self.state in (self.GRASPING, self.LIFTING):
            x, y = self.current_target
            z = self.grasp_z if self.state == self.GRASPING else self.post_grasp_z
            self._move_to(x, y, z, gripper=angle)
        elif self.state in (self.RELEASING,):
            self._move_to(
                self.handover['x'], self.handover['y'], self.handover['z'],
                gripper=angle
            )

    def _publish_joints(self, angles_dict):
        """Convert IK solution dict → Float32MultiArray and publish."""
        msg = Float32MultiArray()
        msg.data = [
            float(angles_dict.get('base', 90)),
            float(angles_dict.get('shoulder', 90)),
            float(angles_dict.get('elbow', 90)),
            float(angles_dict.get('forearm', 90)),
            float(angles_dict.get('wrist', 90)),
            float(angles_dict.get('gripper', 90)),
        ]
        self.joint_pub.publish(msg)
        self.get_logger().debug(
            f'Joint cmd: {[f"{v:.1f}" for v in msg.data]}'
        )

    def _set_speed(self, speed):
        """Publish movement speed."""
        msg = Float32()
        msg.data = float(speed)
        self.speed_pub.publish(msg)

    def _publish_status(self, status):
        """Publish status string for monitoring."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down pick & place node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
