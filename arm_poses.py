#!/usr/bin/env python3
"""
Robot Arm Pose Library
Defines preset poses based on URDF joint limits and arm geometry

MOTOR CALIBRATION INFO:
- Shoulder (channel 1): Moves LEFT when angle > 90°, RIGHT when angle < 90° | NEUTRAL = 90°
- Elbow (channel 2): Moves LEFT when angle > 90°, RIGHT when angle < 90° | NEUTRAL = 90°
- Forearm (channel 3): Moves LEFT when angle > 80° (inverted) | NEUTRAL = 80°
- Wrist (channel 4): Moves LEFT when angle > 60° (inverted) | NEUTRAL = 60°
- Gripper (channel 5): Opens at ~120°, Closes at ~160°
"""

import math

# Calibrated neutral positions (upright/straight)
NEUTRAL = {
    "shoulder": 90,
    "elbow": 90,
    "forearm": 80,
    "wrist": 60,
    "gripper": 120,  # Open position
}


# Convert radians to degrees
def rad_to_deg(rad):
    return rad * 180.0 / math.pi


# URDF Joint Limits (in radians)
JOINT_LIMITS = {
    "joint2": {"min": 0, "max": 4.276},     # Base rotation (Z-axis) - 245 degrees (safety limit)
    "joint3": {"min": -1.57, "max": 1.57},  # Shoulder pitch (X-axis)
    "joint4": {"min": -1.57, "max": 1.57},  # Elbow pitch (X-axis)
    "joint5": {"min": -1.57, "max": 1.57},  # Wrist pitch (X-axis)
}

# Servo channel mapping from URDF
URDF_MAPPING = {
    "joint2": 0,  # Base rotation
    "joint3": 1,  # Shoulder
    "joint4": 2,  # Elbow
    "joint5": 3,  # Wrist
}

# Current test.py servo names
# shoulder: channel 1 - moves LEFT above 90°, RIGHT below 90°
# elbow: channel 2 - moves LEFT above 90°, RIGHT below 90°
# forearm: channel 3 - moves LEFT above 80° (INVERTED)
# wrist: channel 4 - moves LEFT above 60° (INVERTED)
# gripper: channel 5 - opens at 120°, closes at 160°

# Define poses (angles in degrees, 0-180 range for servo control)
# Using calibrated neutral positions
# Deviations from neutral increased by 1.5x for more visible movements
POSES = {
    "home": {
        "description": "Safe home position - arm upright",
        "shoulder": 100,  # Neutral upright
        "elbow": 100,  # Neutral
        "forearm": 80,  # Neutral
        "wrist": 60,  # Neutral
        "gripper": 120,  # Open
    },
    "rest": {
        "description": "Resting position - arm folded down",
        "shoulder": 138,  # Was 140: (140-90)*1.5+90 = 75+90 = 165, reduced to 138 for safety
        "elbow": 60,  # Was 70: (70-90)*1.5+90 = -30+90 = 60
        "forearm": 80,  # Neutral
        "wrist": 60,  # Neutral
        "gripper": 120,  # Open
    },
    "reach_forward": {
        "description": "Reaching forward horizontally",
        "shoulder": 75,  # Was 80: (80-90)*1.5+90 = -15+90 = 75
        "elbow": 125,  # Was 120: (120-90)*1.5+90 = 45+90 = 135, reduced to 125
        "forearm": 80,  # Neutral
        "wrist": 60,  # Neutral
        "gripper": 120,  # Open
    },
    "reach_high": {
        "description": "Reaching upward",
        "shoulder": 75,  # Was 80: (80-90)*1.5+90 = -15+90 = 75
        "elbow": 125,  # Was 120: (120-90)*1.5+90 = 45+90 = 135, reduced to 125
        "forearm": 50,  # Was 60: (60-80)*1.5+80 = -30+80 = 50
        "wrist": 90,  # Was 80: (80-60)*1.5+60 = 30+60 = 90
        "gripper": 120,  # Open
    },
    "grab_low": {
        "description": "Position to grab object on ground",
        "shoulder": 150,  # Was 130: (130-90)*1.5+90 = 60+90 = 150
        "elbow": 150,  # Same as shoulder for better reach
        "forearm": 110,  # Was 100: (100-80)*1.5+80 = 30+80 = 110
        "wrist": 30,  # Was 40: (40-60)*1.5+60 = -30+60 = 30
        "gripper": 120,  # Open (ready to close)
    },
    "grab_close": {
        "description": "Grabbing object close to base",
        "shoulder": 90,  # Neutral
        "elbow": 90,  # Neutral
        "forearm": 80,  # Neutral
        "wrist": 60,  # Neutral
        "gripper": 160,  # Closed
    },
    "hold_up": {
        "description": "Holding object up high",
        "shoulder": 75,  # Was 80: (80-90)*1.5+90 = -15+90 = 75
        "elbow": 125,  # Was 120: (120-90)*1.5+90 = 45+90 = 135, reduced to 125
        "forearm": 50,  # Was 60: (60-80)*1.5+80 = -30+80 = 50
        "wrist": 60,  # Neutral
        "gripper": 160,  # Closed
    },
    "wave": {
        "description": "Waving position",
        "shoulder": 75,  # Was 80: (80-90)*1.5+90 = -15+90 = 75
        "elbow": 90,  # Neutral
        "forearm": 50,  # Was 60: (60-80)*1.5+80 = -30+80 = 50
        "wrist": 90,  # Was 80: (80-60)*1.5+60 = 30+60 = 90
        "gripper": 120,  # Open
    },
}


def print_poses():
    """Print all available poses"""
    print("\n" + "=" * 60)
    print("  ROBOT ARM POSE LIBRARY")
    print("=" * 60 + "\n")

    for pose_name, pose_data in POSES.items():
        print(f"📍 {pose_name.upper()}")
        print(f"   {pose_data['description']}")
        print("   Commands:")
        for joint, angle in pose_data.items():
            if joint != "description":
                print(f"     {joint} {angle}")
        print()


def get_pose(pose_name):
    """Get a specific pose by name"""
    return POSES.get(pose_name)


def execute_pose(pose_name):
    """Generate commands to execute a pose"""
    pose = get_pose(pose_name)
    if not pose:
        print(f"Pose '{pose_name}' not found!")
        return []

    commands = []
    print(f"\n🎯 Executing pose: {pose_name.upper()}")
    print(f"   {pose['description']}\n")

    for joint, angle in pose.items():
        if joint != "description":
            cmd = f"{joint} {angle}"
            commands.append(cmd)
            print(f"   → {cmd}")

    return commands


def demo_sequence():
    """Print a demo sequence of poses"""
    print("\n" + "=" * 60)
    print("  SUGGESTED DEMO SEQUENCE")
    print("=" * 60 + "\n")

    sequence = [
        ("home", 2),
        ("reach_forward", 2),
        ("grab_low", 2),
        ("grab_close", 1),
        ("hold_up", 2),
        ("home", 2),
    ]

    for pose_name, duration in sequence:
        print(f"1. Execute '{pose_name}' pose → wait {duration}s")

    print("\nTo run in test.py:")
    print("  Type each joint command from the pose definitions above")
    print("  Example: 'shoulder 90' then 'elbow 90' etc.")


if __name__ == "__main__":
    print_poses()
    demo_sequence()

    print("\n" + "=" * 60)
    print("USAGE:")
    print("-" * 60)
    print("In Python:")
    print("  from arm_poses import get_pose, execute_pose")
    print("  pose = get_pose('reach_forward')")
    print("  commands = execute_pose('home')")
    print("\nIn test.py interactive mode:")
    print("  Type: shoulder 90")
    print("  Type: elbow 135")
    print("  etc.")
    print("=" * 60 + "\n")
