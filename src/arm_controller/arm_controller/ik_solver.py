#!/usr/bin/env python3
"""
Simple Inverse Kinematics Solver for 6-DOF Robotic Arm
Uses geometric approach for position-based control
"""

import numpy as np
import math


class ArmIKSolver:
    """
    Inverse kinematics solver for the 6-DOF arm
    Assumes arm configuration: Base -> Shoulder -> Elbow -> Forearm -> Wrist -> Gripper
    """
    
    def __init__(self):
        # Link lengths from URDF joint origin xyz values (in meters)
        # Measured from robot_fixed.urdf - MUST MATCH arm_3d_visualization.py
        self.L1 = 0.059   # Base to shoulder height (base joint z=0.0592971)
        self.L2 = 0.084   # Shoulder to elbow (elbow joint y=0.0841198) - main upper arm!
        self.L3 = 0.086   # Elbow to wrist (wrist joint dist ~0.086)
        self.L4 = 0.061   # Wrist to gripper (wrist_rotate joint dist ~0.061)
        
        # Home position (neutral/straight forward)
        self.home_position = {
            'base': 98,      # b - base rotation
            'shoulder': 96,  # s - shoulder
            'elbow': 98,     # e - elbow
            'forearm': 70,   # f - forearm
            'wrist': 60,     # w - wrist
            'gripper': 90    # g - gripper
        }
        
        # Cache for previous solution to speed up solution selection
        self.previous_solution = None
    
    def solve_ik(self, x, y, z, pitch=0, roll=0, gripper_angle=None, seed_state=None):
        """
        Solve inverse kinematics for target position.
        This is the EXACT INVERSE of forward_kinematics().
        
        Uses INVERTED angle convention for shoulder/elbow to match RViz:
        FK uses: shoulder_rad = radians(90.0 - shoulder_degrees)
        So IK must use: shoulder_degrees = 90.0 - degrees(shoulder_rad)
        
        Args:
            x, y, z: Target position in meters (GRIPPER position at 5th joint)
            pitch: Desired pitch angle of end effector (radians)
            roll: Desired roll angle of end effector (radians)
            gripper_angle: Gripper opening angle (degrees), None to keep current
            seed_state: Previous joint angles dict (optional, for solution selection)
            
        Returns:
            dict: Joint angles in degrees, or None if unreachable
        """
        # Use seed_state or previous solution for continuity
        if seed_state is None:
            seed_state = self.previous_solution if self.previous_solution else self.home_position
        # Link lengths - MUST MATCH arm_3d_visualization.py and FK
        L_base = 0.059       # Base to shoulder height
        L_upper = 0.084      # Shoulder to elbow (main upper arm)
        L_forearm = 0.086    # Elbow to wrist
        L_wrist = 0.061      # Wrist to gripper (5th joint)
        
        # Base rotation (yaw) - rotation around vertical axis
        # The base servo can only rotate 0-180°, which covers the +X half-plane
        # atan2 gives angle from +X axis: 0 at +X, ±π at -X
        # Our base: 90° = +X, 0° = -Y, 180° = +Y
        # So targets at X<0 with small |Y| are UNREACHABLE
        base_angle = math.atan2(y, x)
        base_degrees = math.degrees(base_angle) + 90.0
        
        # Check if target direction is reachable with base servo limits
        # Allow small tolerance for numerical precision
        if base_degrees < -5 or base_degrees > 185:
            return None  # Target is in the -X half-plane, unreachable
        
        base_degrees = max(0, min(180, base_degrees))
        
        # Distance in XY plane to GRIPPER
        r_xy = math.sqrt(x**2 + y**2)
        
        # Handle case when target is very close to origin (directly above base)
        if r_xy < 0.001:
            # Target is directly above base, use current base angle
            base_degrees = 90.0  # Default to forward
        
        # Work backwards: gripper is at (x,y,z), need to find wrist position
        # The gripper extends from wrist at angle = full_angle = arm_angle + forearm_rad
        # full_angle should match desired pitch
        # gripper_x = wrist_x + L_wrist * sin(full_angle)
        # gripper_z = wrist_z + L_wrist * cos(full_angle)
        # 
        # Solving for wrist position:
        # wrist_x = gripper_x - L_wrist * sin(pitch)
        # wrist_z = gripper_z - L_wrist * cos(pitch)
        #
        # In 2D projection (from base axis):
        # wrist_r = r_xy - L_wrist * sin(pitch)
        # wrist_z = (z - L_base) - L_wrist * cos(pitch)
        wrist_r = r_xy - L_wrist * math.sin(pitch)  # Horizontal distance to wrist from base axis
        wrist_z = (z - L_base) - L_wrist * math.cos(pitch)  # Vertical distance to wrist from shoulder
        
        # Distance from shoulder to wrist in 2D vertical plane
        d = math.sqrt(wrist_r**2 + wrist_z**2)
        
        # Check if target is reachable
        if d > (L_upper + L_forearm) or d < abs(L_upper - L_forearm):
            return None  # Target unreachable
        if d < 0.001:
            return None  # Too close to shoulder
        
        # Elbow angle using law of cosines
        # L_upper² + L_forearm² - 2*L_upper*L_forearm*cos(elbow) = d²
        cos_elbow_internal = (L_upper**2 + L_forearm**2 - d**2) / (2 * L_upper * L_forearm)
        cos_elbow_internal = np.clip(cos_elbow_internal, -1, 1)
        elbow_internal = math.acos(cos_elbow_internal)  # Internal angle of elbow triangle
        
        # Elbow angle in arm coordinates: π - internal angle = bend angle
        elbow_rad = math.pi - elbow_internal
        
        # Shoulder angle: angle to target minus offset from triangle
        # gamma = angle from vertical to WRIST (not gripper)
        gamma = math.atan2(wrist_r, wrist_z)
        
        # alpha = angle in triangle at shoulder
        cos_alpha = (L_upper**2 + d**2 - L_forearm**2) / (2 * L_upper * d)
        cos_alpha = np.clip(cos_alpha, -1, 1)
        alpha = math.acos(cos_alpha)
        
        # Shoulder rad: gamma - alpha for elbow-up configuration
        # This makes the arm reach UP towards targets above
        shoulder_rad = gamma - alpha
        
        # Convert radians to GUI degrees using INVERTED convention
        # FK uses: joint_rad = radians(90.0 - joint_degrees)
        # So IK must use: joint_degrees = 90.0 - degrees(joint_rad)
        shoulder_degrees = 90.0 - math.degrees(shoulder_rad)
        elbow_degrees = 90.0 - math.degrees(elbow_rad)
        
        # Forearm: adjust for desired pitch (NORMAL convention for hardware)
        # full_angle = arm_angle + forearm_rad should equal pitch
        # Hardware expects: forearm_degrees where higher = pitch down
        # Visualization uses inverted display, but hardware gets normal value
        # forearm_rad = pitch - arm_angle
        # forearm_degrees = degrees(forearm_rad) + 90.0
        arm_angle = shoulder_rad + elbow_rad
        forearm_rad = pitch - arm_angle
        forearm_degrees = math.degrees(forearm_rad) + 90.0
        
        # Wrist roll
        wrist_degrees = math.degrees(roll) + 90.0
        
        joint_angles = {
            'base': base_degrees,
            'shoulder': shoulder_degrees,
            'elbow': elbow_degrees,
            'forearm': forearm_degrees,
            'wrist': wrist_degrees,
            'gripper': gripper_angle if gripper_angle is not None else 90.0
        }
        
        # Clamp to safe servo ranges
        joint_angles = self._clamp_angles(joint_angles)
        
        # Cache solution for next iteration (faster incremental moves)
        self.previous_solution = joint_angles.copy()
        
        return joint_angles
    
    def _clamp_angles(self, angles):
        """Clamp joint angles to safe servo ranges"""
        # Base motor is 180 degrees (calibrated at 90 as center)
        # All other joints are also 180 degree servos
        limits = {
            'base': (0, 180),
            'shoulder': (0, 180),
            'elbow': (0, 180),
            'forearm': (0, 180),
            'wrist': (0, 180),
            'gripper': (60, 160)
        }
        
        clamped = {}
        for joint, angle in angles.items():
            min_angle, max_angle = limits.get(joint, (0, 180))
            clamped[joint] = max(min_angle, min(max_angle, angle))
        
        return clamped
    
    def forward_kinematics(self, base, shoulder, elbow, forearm, wrist):
        """
        Calculate end effector position from joint angles.
        MUST match arm_3d_visualization.calculate_arm_positions() exactly!
        Uses SAME angle inversions as RViz/send_command():
        - Shoulder and Elbow: INVERTED (90.0 - pos)
        - Base, Forearm, Wrist: Normal (pos - 90.0)
        
        Args:
            Joint angles in degrees (GUI convention: 90° = home)
            
        Returns:
            tuple: (x, y, z, pitch, roll) position and orientation
        """
        # Link lengths from URDF - MUST MATCH arm_3d_visualization.py EXACTLY
        L_base = 0.059       # Base to shoulder height
        L_upper = 0.084      # Shoulder to elbow (main upper arm)
        L_forearm = 0.086    # Elbow to wrist (forearm)
        L_wrist = 0.061      # Wrist to gripper
        
        # Convert to radians - matches arm_3d_visualization.py
        # Shoulder and Elbow use INVERTED conversion
        # Forearm uses NORMAL for FK calculation (matches hardware)
        base_rad = math.radians(base - 90.0)
        shoulder_rad = math.radians(90.0 - shoulder)  # INVERTED
        elbow_rad = math.radians(90.0 - elbow)        # INVERTED
        forearm_rad = math.radians(forearm - 90.0)    # NORMAL (hardware convention)
        wrist_rad = math.radians(wrist - 90.0)
        
        # Calculate positions using SAME math as arm_3d_visualization.py
        # Elbow joint - uses sin/cos with vertical reference
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        
        # Wrist joint - cumulative angle
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_forearm * math.sin(arm_angle)
        wrist_z = elbow_z + L_forearm * math.cos(arm_angle)
        
        # Gripper position - includes forearm pitch
        full_angle = arm_angle + forearm_rad
        gripper_x = wrist_x + L_wrist * math.sin(full_angle)
        gripper_z = wrist_z + L_wrist * math.cos(full_angle)
        
        # Apply base rotation for 3D position
        x = gripper_x * math.cos(base_rad)
        y = gripper_x * math.sin(base_rad)
        z = gripper_z
        
        pitch = full_angle
        
        return (x, y, z, pitch, wrist_rad)


class PredefinedPoses:
    """Library of predefined poses in Cartesian space"""
    
    @staticmethod
    def get_home():
        """Neutral position - arm straight forward"""
        return {
            'name': 'home',
            'position': (0.25, 0.0, 0.08),  # Adjusted for smaller arm
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_forward():
        """Reach forward"""
        return {
            'name': 'reach_forward',
            'position': (0.28, 0.0, 0.08),  # Adjusted
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_up():
        """Reach upward"""
        return {
            'name': 'reach_up',
            'position': (0.15, 0.0, 0.20),  # Adjusted
            'orientation': (math.pi/4, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_down():
        """Reach down to pick something"""
        return {
            'name': 'reach_down',
            'position': (0.25, 0.0, 0.05),
            'orientation': (-math.pi/4, 0),
            'gripper': 120  # Open gripper
        }
    
    @staticmethod
    def get_reach_left():
        """Reach to the left"""
        return {
            'name': 'reach_left',
            'position': (0.2, 0.2, 0.15),
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_right():
        """Reach to the right"""
        return {
            'name': 'reach_right',
            'position': (0.2, -0.2, 0.15),
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_all_poses():
        """Get all predefined poses"""
        return {
            'home': PredefinedPoses.get_home(),
            'reach_forward': PredefinedPoses.get_reach_forward(),
            'reach_up': PredefinedPoses.get_reach_up(),
            'reach_down': PredefinedPoses.get_reach_down(),
            'reach_left': PredefinedPoses.get_reach_left(),
            'reach_right': PredefinedPoses.get_reach_right(),
        }


if __name__ == '__main__':
    # Test the IK solver
    solver = ArmIKSolver()
    poses = PredefinedPoses()
    
    print("Testing Inverse Kinematics Solver\n")
    print("=" * 50)
    
    for pose_name, pose_data in poses.get_all_poses().items():
        print(f"\nPose: {pose_name}")
        print(f"Target: {pose_data['position']}")
        
        x, y, z = pose_data['position']
        pitch, roll = pose_data['orientation']
        gripper = pose_data['gripper']
        
        joint_angles = solver.solve_ik(x, y, z, pitch, roll, gripper)
        
        if joint_angles:
            print("Joint Angles (degrees):")
            for joint, angle in joint_angles.items():
                print(f"  {joint}: {angle:.1f}°")
        else:
            print("  UNREACHABLE!")
