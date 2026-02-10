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
    
    def solve_ik_to_point(self, x, y, z, roll=0, gripper_angle=None, seed_state=None):
        """Solve IK so the gripper tip lands exactly on (x, y, z).

        Unlike solve_ik() which requires the caller to supply a pitch angle,
        this method searches over a range of pitch values and picks the one
        whose FK-verified gripper position is closest to the target.

        It prefers solutions that keep joints near their current position
        (smooth motion) and penalises extreme pitch values.

        Returns:
            dict of joint angles in degrees, or None if unreachable.
        """
        L_base = 0.059
        r_xy = math.sqrt(x**2 + y**2)
        z_above = z - L_base

        # Build a coarse set of candidate pitch angles (from vertical → past horizontal)
        # pitch = 0 → gripper points up, π/2 → horizontal, π → down
        # We sweep a generous range so low-Z targets are reachable too.
        pitch_candidates = []

        # Start with an initial geometric guess (angle from shoulder to target)
        if r_xy > 0.001 or abs(z_above) > 0.001:
            guess = math.atan2(r_xy, z_above)
            guess = max(0.0, min(math.radians(160), guess))
        else:
            guess = 0.0

        # Dense sweep around the guess, plus a broad sweep for coverage
        for offset in [i * 0.02 for i in range(-30, 31)]:  # ±0.6 rad around guess
            p = guess + offset
            if 0.0 <= p <= math.radians(160):
                pitch_candidates.append(p)

        # Add broad sweep to not miss solutions far from the guess
        for deg in range(0, 161, 5):
            pitch_candidates.append(math.radians(deg))

        # Remove duplicates and sort
        pitch_candidates = sorted(set(round(p, 4) for p in pitch_candidates))

        best_solution = None
        best_error = float('inf')

        for pitch in pitch_candidates:
            sol = self.solve_ik(x, y, z, pitch=pitch, roll=roll,
                                gripper_angle=gripper_angle, seed_state=seed_state)
            if sol is None:
                continue

            # Verify with FK: where does the gripper actually end up?
            fk = self.forward_kinematics(
                sol['base'], sol['shoulder'], sol['elbow'],
                sol['forearm'], sol['wrist']
            )
            fk_x, fk_y, fk_z = fk[0], fk[1], fk[2]
            pos_err = math.sqrt((fk_x - x)**2 + (fk_y - y)**2 + (fk_z - z)**2)

            # Penalise extreme pitch (prefer more natural poses)
            pitch_penalty = 0.0002 * abs(pitch - math.radians(45))

            # Penalise large jumps from current joint positions (smoothness)
            jump_penalty = 0.0
            if seed_state:
                for jname in ('shoulder', 'elbow', 'forearm'):
                    if jname in seed_state:
                        jump_penalty += 0.00005 * abs(sol[jname] - seed_state[jname])

            total_err = pos_err + pitch_penalty + jump_penalty

            if total_err < best_error:
                best_error = total_err
                best_solution = sol

        # Accept if FK position error is below 15 mm
        if best_solution is not None and best_error < 0.015:
            self.previous_solution = best_solution.copy()
            return best_solution

        # --- Fallback: target is outside workspace ---------------------------
        # Instead of returning None (arm freezes), try nearby points that the
        # arm CAN reach.  Strategy: keep the base angle (direction) and search
        # outward along the radial direction at the same Z, because the usual
        # failure mode is "target too close to base at low Z".  We also try
        # scaling along the original ray as a second pass.
        direction_r = math.sqrt(x**2 + y**2)
        if direction_r < 0.001:
            return None  # target directly above base — nothing to adjust

        ux = x / direction_r  # unit vector in XY towards target
        uy = y / direction_r

        fallback_solution = None
        fallback_dist = float('inf')

        # Pass 1: vary radial distance at fixed Z
        for r_mm in range(int(direction_r * 1000), 250, 4):
            r = r_mm / 1000.0
            sx, sy, sz = ux * r, uy * r, z
            sol = self._try_solve_point(sx, sy, sz, roll, gripper_angle, seed_state)
            if sol is not None:
                d = math.sqrt((sx - x)**2 + (sy - y)**2 + (sz - z)**2)
                if d < fallback_dist:
                    fallback_dist = d
                    fallback_solution = sol
                break  # first hit at increasing r is closest

        # Pass 2: scale along original ray from 100% down to 40%
        for pct in range(100, 39, -3):
            sx = x * pct / 100.0
            sy = y * pct / 100.0
            sz = z * pct / 100.0
            sol = self._try_solve_point(sx, sy, sz, roll, gripper_angle, seed_state)
            if sol is not None:
                d = math.sqrt((sx - x)**2 + (sy - y)**2 + (sz - z)**2)
                if d < fallback_dist:
                    fallback_dist = d
                    fallback_solution = sol
                break  # first hit while shrinking is closest

        if fallback_solution is not None:
            self.previous_solution = fallback_solution.copy()
            return fallback_solution

        return None

    # ------------------------------------------------------------------
    def _try_solve_point(self, x, y, z, roll=0, gripper_angle=None, seed_state=None):
        """Quick helper: sweep pitches and return a solution with FK err < 15 mm."""
        L_base = 0.059
        r_xy = math.sqrt(x**2 + y**2)
        z_above = z - L_base

        if r_xy > 0.001 or abs(z_above) > 0.001:
            guess = math.atan2(r_xy, z_above)
            guess = max(0.0, min(math.radians(160), guess))
        else:
            guess = 0.0

        pitches = sorted(set(
            round(p, 4)
            for p in [guess + i * 0.05 for i in range(-15, 16)]
                    + [math.radians(d) for d in range(0, 161, 10)]
            if 0.0 <= p <= math.radians(160)
        ))

        best_sol = None
        best_err = float('inf')
        for pitch in pitches:
            sol = self.solve_ik(x, y, z, pitch=pitch, roll=roll,
                                gripper_angle=gripper_angle, seed_state=seed_state)
            if sol is None:
                continue
            fk = self.forward_kinematics(
                sol['base'], sol['shoulder'], sol['elbow'],
                sol['forearm'], sol['wrist']
            )
            err = math.sqrt((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-z)**2)
            if err < best_err:
                best_err = err
                best_sol = sol
        if best_sol is not None and best_err < 0.015:
            return best_sol
        return None

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
        # The base servo is 245° range (0-245°), centered at 90°
        # atan2 gives angle from +X axis: 0 at +X, ±π at -X
        # Our base: 90° = +X, 0° = -Y, 180° = +Y
        # 245° limit provides safety margin from full mechanical range
        base_angle = math.atan2(y, x)
        base_degrees = math.degrees(base_angle) + 90.0
        
        # Normalize to 0-360 range first
        if base_degrees < 0:
            base_degrees += 360.0
        
        # Check if target direction is reachable with base servo limits (0-245°)
        # Allow small tolerance for numerical precision
        if base_degrees > 250:
            return None  # Target is beyond the 245° range, unreachable
        
        base_degrees = max(0, min(245, base_degrees))
        
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
        
        # Try BOTH elbow configurations and pick the one that fits servo limits
        # Elbow-up:   shoulder_rad = gamma - alpha  (arm above target line)
        # Elbow-down: shoulder_rad = gamma + alpha  (arm below target line)
        best_config = None
        best_clamp_error = float('inf')

        for config_name, shoulder_rad in [('up', gamma - alpha),
                                           ('down', gamma + alpha)]:
            s_deg = 90.0 - math.degrees(shoulder_rad)
            e_deg = 90.0 - math.degrees(elbow_rad)

            arm_angle = shoulder_rad + elbow_rad
            forearm_rad = pitch - arm_angle
            f_deg = 90.0 - math.degrees(forearm_rad)

            wrist_degrees = math.degrees(roll) + 90.0

            candidate = {
                'base': base_degrees,
                'shoulder': s_deg,
                'elbow': e_deg,
                'forearm': f_deg,
                'wrist': wrist_degrees,
                'gripper': gripper_angle if gripper_angle is not None else 90.0
            }

            # Measure how much clamping would distort the solution
            clamp_err = 0.0
            for jname in ('shoulder', 'elbow', 'forearm'):
                lo, hi = (0, 180)
                v = candidate[jname]
                if v < lo:
                    clamp_err += (lo - v)
                elif v > hi:
                    clamp_err += (v - hi)

            if clamp_err < best_clamp_error:
                best_clamp_error = clamp_err
                best_config = candidate

        joint_angles = self._clamp_angles(best_config)
        
        # Cache solution for next iteration (faster incremental moves)
        self.previous_solution = joint_angles.copy()
        
        return joint_angles
    
    def _clamp_angles(self, angles):
        """Clamp joint angles to safe servo ranges"""
        # Base motor is 245 degrees (safety limit, mechanical max ~270)
        # All other joints are 180 degree servos
        limits = {
            'base': (0, 245),
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
        
        # Convert to radians - MUST match arm_3d_visualization.py EXACTLY
        # Shoulder and Elbow use INVERTED conversion
        # Forearm: also INVERTED to match visualization display
        base_rad = math.radians(base - 90.0)
        shoulder_rad = math.radians(90.0 - shoulder)  # INVERTED
        elbow_rad = math.radians(90.0 - elbow)        # INVERTED
        forearm_rad = -math.radians(forearm - 90.0)   # INVERTED (matches viz)
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
