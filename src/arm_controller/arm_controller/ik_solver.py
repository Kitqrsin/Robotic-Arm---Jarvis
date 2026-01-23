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
        # Link lengths (in meters, from URDF measurements)
        self.L1 = 0.03   # Base to shoulder height (motor1)
        self.L2 = 0.128  # Shoulder to elbow length (hand_extension)
        self.L3 = 0.13   # Elbow to wrist length (pipe + part of motor4)
        self.L4 = 0.08   # Wrist to gripper length (motor4 tip)
        
        # Home position (neutral/straight forward)
        self.home_position = {
            'base': 98,      # b - base rotation
            'shoulder': 96,  # s - shoulder
            'elbow': 98,     # e - elbow
            'forearm': 70,   # f - forearm
            'wrist': 60,     # w - wrist
            'gripper': 90    # g - gripper
        }
    
    def solve_ik(self, x, y, z, pitch=0, roll=0, gripper_angle=None):
        """
        Solve inverse kinematics for target position
        
        Args:
            x, y, z: Target position in meters
            pitch: Desired pitch angle of end effector (radians)
            roll: Desired roll angle of end effector (radians)
            gripper_angle: Gripper opening angle (degrees), None to keep current
            
        Returns:
            dict: Joint angles in degrees, or None if unreachable
        """
        
        # Base rotation (yaw) - rotation around vertical axis
        base_angle = math.atan2(y, x)
        
        # Distance in XY plane
        r_xy = math.sqrt(x**2 + y**2)
        
        # Account for wrist offset based on pitch
        wrist_offset_z = self.L4 * math.sin(pitch)
        wrist_offset_xy = self.L4 * math.cos(pitch)
        
        # Wrist position (target minus end effector offset)
        wrist_x = r_xy - wrist_offset_xy
        wrist_z = z - self.L1 - wrist_offset_z
        
        # Distance to wrist from shoulder
        d = math.sqrt(wrist_x**2 + wrist_z**2)
        
        # Check if target is reachable
        if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
            return None  # Target unreachable
        
        # Shoulder angle using law of cosines
        cos_elbow = (self.L2**2 + self.L3**2 - d**2) / (2 * self.L2 * self.L3)
        cos_elbow = np.clip(cos_elbow, -1, 1)  # Numerical stability
        elbow_angle = math.acos(cos_elbow)
        
        # Angle from shoulder to wrist
        alpha = math.atan2(wrist_z, wrist_x)
        
        # Angle of triangle at shoulder
        cos_beta = (self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d)
        cos_beta = np.clip(cos_beta, -1, 1)
        beta = math.acos(cos_beta)
        
        shoulder_angle = alpha + beta
        
        # Forearm/wrist angle to achieve desired pitch
        forearm_angle = pitch - shoulder_angle + elbow_angle
        
        # Convert to degrees
        joint_angles = {
            'base': math.degrees(base_angle) + self.home_position['base'],
            'shoulder': 180 - math.degrees(shoulder_angle),  # Inverted for servo
            'elbow': 180 - math.degrees(elbow_angle),        # Inverted for servo
            'forearm': math.degrees(forearm_angle) + self.home_position['forearm'],
            'wrist': math.degrees(roll) + self.home_position['wrist'],
            'gripper': gripper_angle if gripper_angle is not None else self.home_position['gripper']
        }
        
        # Clamp to safe servo ranges
        joint_angles = self._clamp_angles(joint_angles)
        
        return joint_angles
    
    def _clamp_angles(self, angles):
        """Clamp joint angles to safe servo ranges"""
        limits = {
            'base': (0, 270),
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
        Calculate end effector position from joint angles
        
        Args:
            Joint angles in degrees
            
        Returns:
            tuple: (x, y, z, pitch, roll) position and orientation
        """
        # Convert to radians
        base_rad = math.radians(base - self.home_position['base'])
        shoulder_rad = math.radians(180 - shoulder)
        elbow_rad = math.radians(180 - elbow)
        forearm_rad = math.radians(forearm - self.home_position['forearm'])
        wrist_rad = math.radians(wrist - self.home_position['wrist'])
        
        # Calculate positions
        shoulder_z = self.L1
        
        # Shoulder link
        elbow_x = self.L2 * math.cos(shoulder_rad)
        elbow_z = shoulder_z + self.L2 * math.sin(shoulder_rad)
        
        # Elbow link
        wrist_x = elbow_x + self.L3 * math.cos(shoulder_rad + elbow_rad)
        wrist_z = elbow_z + self.L3 * math.sin(shoulder_rad + elbow_rad)
        
        # End effector
        pitch = shoulder_rad + elbow_rad + forearm_rad
        end_x = wrist_x + self.L4 * math.cos(pitch)
        end_z = wrist_z + self.L4 * math.sin(pitch)
        
        # Apply base rotation
        x = end_x * math.cos(base_rad)
        y = end_x * math.sin(base_rad)
        z = end_z
        
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
