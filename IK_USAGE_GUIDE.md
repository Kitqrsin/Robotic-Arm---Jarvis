# Inverse Kinematics & Cartesian Pose Control

## Overview

The robotic arm now supports **inverse kinematics (IK)**, allowing you to control the arm by specifying where you want the end effector (gripper) to go, rather than manually setting joint angles.

## Predefined Poses

These poses are defined in Cartesian space and automatically calculated:

- **home** - Neutral position, arm straight forward
- **reach_forward** - Extend arm forward
- **reach_up** - Reach upward  
- **reach_down** - Reach down to pick objects
- **reach_left** - Reach to the left side
- **reach_right** - Reach to the right side

## Using Predefined Poses

### Method 1: Topic-Based (Simplest)

```bash
# Execute a predefined pose by name
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'home'"
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_up'"
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_down'"
```

### Method 2: Custom Cartesian Position

```bash
# Move to specific (x, y, z) position in meters
ros2 topic pub --once /move_to_position geometry_msgs/msg/Point "{x: 0.25, y: 0.1, z: 0.2}"
```

## Starting the IK System

### Launch with IK enabled

```bash
# Start Docker
sudo docker run -it --rm --privileged --network host \
  -v /home/tnt/arm_project/Robotic-Arm---Jarvis:/workspace \
  -v /dev:/dev -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-arm:latest

# Build and start
colcon build
source install/setup.bash

# Start servo controller
ros2 run arm_controller servo_node &

# Start Cartesian pose service
ros2 run arm_controller cartesian_pose_service
```

## Testing the IK Solver

Test the inverse kinematics calculations:

```bash
# From the workspace
cd /workspace/src/arm_controller/arm_controller
python3 ik_solver.py
```

This will show the calculated joint angles for all predefined poses.

## Creating Custom Poses

Edit [ik_solver.py](src/arm_controller/arm_controller/ik_solver.py) and add new poses to the `PredefinedPoses` class:

```python
@staticmethod
def get_my_custom_pose():
    """My custom pose description"""
    return {
        'name': 'my_custom_pose',
        'position': (x, y, z),  # in meters
        'orientation': (pitch, roll),  # in radians
        'gripper': 90  # gripper angle in degrees
    }
```

Then add it to `get_all_poses()`:

```python
def get_all_poses():
    return {
        ...
        'my_custom_pose': PredefinedPoses.get_my_custom_pose(),
    }
```

## Coordinate System

```
         Z↑
          |
          |_____ Y
         /
        /
       X

X: Forward/backward (+ = forward)
Y: Left/right (+ = left, - = right)  
Z: Up/down (+ = up)
```

Origin is at the base of the robot.

## Example Workflow

```bash
# 1. Start the system
ros2 run arm_controller servo_node &
ros2 run arm_controller cartesian_pose_service &

# 2. Go to home position
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'home'"

# 3. Reach forward
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_forward'"

# 4. Reach down to pick something
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_down'"

# 5. Move to custom position
ros2 topic pub --once /move_to_position geometry_msgs/msg/Point "{x: 0.3, y: 0.0, z: 0.15}"
```

## Adjusting Link Lengths

The IK solver needs accurate arm dimensions. Edit in [ik_solver.py](src/arm_controller/arm_controller/ik_solver.py):

```python
self.L1 = 0.10  # Base to shoulder height (meters)
self.L2 = 0.15  # Shoulder to elbow length
self.L3 = 0.15  # Elbow to wrist length
self.L4 = 0.10  # Wrist to gripper length
```

Measure your actual arm and update these values for accurate IK.

## Troubleshooting

### "Target position unreachable"
- Position is outside the robot's workspace
- Try positions closer to the robot (smaller X, Y, Z values)
- Check link lengths are configured correctly

### Joint angles look wrong
- Verify link lengths match your actual robot
- Check home position values in ik_solver.py
- Test with known good poses first (home, reach_forward)

### Gripper doesn't move
- Gripper angle is set independently  
- Use `gripper_angle` parameter or modify the predefined pose

## Advanced: Direct IK Calculation

```python
from ik_solver import ArmIKSolver

solver = ArmIKSolver()
joint_angles = solver.solve_ik(
    x=0.25,        # meters
    y=0.1,         # meters  
    z=0.2,         # meters
    pitch=0,       # radians
    roll=0,        # radians
    gripper_angle=90  # degrees
)

if joint_angles:
    print(f"Base: {joint_angles['base']}°")
    print(f"Shoulder: {joint_angles['shoulder']}°")
    # ... etc
else:
    print("Position unreachable!")
```
