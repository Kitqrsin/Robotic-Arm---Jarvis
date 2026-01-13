#!/usr/bin/env python3
"""
Robot Arm Pose Library
Defines preset poses based on URDF joint limits and arm geometry
"""
import math

# Convert radians to degrees
def rad_to_deg(rad):
    return rad * 180.0 / math.pi

# URDF Joint Limits (in radians)
JOINT_LIMITS = {
    'joint2': {'min': -1.57, 'max': 1.57},  # Base rotation (Z-axis)
    'joint3': {'min': -1.57, 'max': 1.57},  # Shoulder pitch (X-axis)
    'joint4': {'min': -1.57, 'max': 1.57},  # Elbow pitch (X-axis)
    'joint5': {'min': -1.57, 'max': 1.57},  # Wrist pitch (X-axis)
}

# Servo channel mapping from URDF
URDF_MAPPING = {
    'joint2': 0,  # Base rotation
    'joint3': 1,  # Shoulder
    'joint4': 2,  # Elbow/Forearm
    'joint5': 3,  # Wrist
}

# Current test.py servo names (for reference)
# shoulder: channel 1
# elbow: channel 2
# forearm: channel 3
# wrist: channel 4
# gripper: channel 5

# Define poses (angles in degrees, 0-180 range for servo control)
# Note: 90 degrees = neutral/straight position for most servos
POSES = {
    'home': {
        'description': 'Safe home position - arm upright',
        'shoulder': 90,   # Neutral upright
        'elbow': 90,      # Straight
        'forearm': 90,    # Neutral
        'wrist': 90,      # Neutral
        'gripper': 0,     # Open
    },
    
    'rest': {
        'description': 'Resting position - arm folded down',
        'shoulder': 180,  # Folded back
        'elbow': 45,      # Bent inward
        'forearm': 90,    # Neutral
        'wrist': 90,      # Neutral
        'gripper': 0,     # Open
    },
    
    'reach_forward': {
        'description': 'Reaching forward horizontally',
        'shoulder': 45,   # Lean forward
        'elbow': 135,     # Extended
        'forearm': 90,    # Straight
        'wrist': 90,      # Level with ground
        'gripper': 0,     # Open
    },
    
    'reach_high': {
        'description': 'Reaching upward',
        'shoulder': 45,   # Angle up
        'elbow': 45,      # Extended up
        'forearm': 45,    # Angled up
        'wrist': 135,     # Tip up
        'gripper': 0,     # Open
    },
    
    'grab_low': {
        'description': 'Position to grab object on ground',
        'shoulder': 135,  # Lean forward/down
        'elbow': 45,      # Reach down
        'forearm': 135,   # Point down
        'wrist': 45,      # Angle for grip
        'gripper': 0,     # Open (ready to close)
    },
    
    'grab_close': {
        'description': 'Grabbing object close to base',
        'shoulder': 90,   # Neutral
        'elbow': 90,      # Bent
        'forearm': 90,    # Neutral
        'wrist': 90,      # Neutral
        'gripper': 120,   # Closed
    },
    
    'hold_up': {
        'description': 'Holding object up high',
        'shoulder': 45,   # Raised
        'elbow': 45,      # Extended
        'forearm': 45,    # Up
        'wrist': 90,      # Level
        'gripper': 120,   # Closed
    },
    
    'wave': {
        'description': 'Waving position (alternate wrist angle)',
        'shoulder': 45,   # Raised
        'elbow': 90,      # Bent
        'forearm': 45,    # Angled
        'wrist': 135,     # Up for waving motion
        'gripper': 0,     # Open
    },
}

def print_poses():
    """Print all available poses"""
    print("\n" + "="*60)
    print("  ROBOT ARM POSE LIBRARY")
    print("="*60 + "\n")
    
    for pose_name, pose_data in POSES.items():
        print(f"📍 {pose_name.upper()}")
        print(f"   {pose_data['description']}")
        print(f"   Commands:")
        for joint, angle in pose_data.items():
            if joint != 'description':
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
        if joint != 'description':
            cmd = f"{joint} {angle}"
            commands.append(cmd)
            print(f"   → {cmd}")
    
    return commands

def demo_sequence():
    """Print a demo sequence of poses"""
    print("\n" + "="*60)
    print("  SUGGESTED DEMO SEQUENCE")
    print("="*60 + "\n")
    
    sequence = [
        ('home', 2),
        ('reach_forward', 2),
        ('grab_low', 2),
        ('grab_close', 1),
        ('hold_up', 2),
        ('home', 2),
    ]
    
    for pose_name, duration in sequence:
        print(f"1. Execute '{pose_name}' pose → wait {duration}s")
    
    print("\nTo run in test.py:")
    print("  Type each joint command from the pose definitions above")
    print("  Example: 'shoulder 90' then 'elbow 90' etc.")

if __name__ == '__main__':
    print_poses()
    demo_sequence()
    
    print("\n" + "="*60)
    print("USAGE:")
    print("-"*60)
    print("In Python:")
    print("  from arm_poses import get_pose, execute_pose")
    print("  pose = get_pose('reach_forward')")
    print("  commands = execute_pose('home')")
    print("\nIn test.py interactive mode:")
    print("  Type: shoulder 90")
    print("  Type: elbow 135")
    print("  etc.")
    print("="*60 + "\n")
