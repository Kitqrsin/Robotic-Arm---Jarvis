# ✅ Inverse Kinematics System - Ready to Test!

## What We've Built

You now have a **complete inverse kinematics (IK) system** that lets you control your robotic arm by specifying WHERE you want it to go, instead of manually setting joint angles.

### ✅ Components Created:

1. **IK Solver** ([ik_solver.py](src/arm_controller/arm_controller/ik_solver.py))
   - Calculates joint angles from Cartesian positions
   - Configured for your actual arm dimensions (from URDF)
   - All 6 predefined poses are reachable ✓

2. **Cartesian Pose Service** ([cartesian_pose_service.py](src/arm_controller/arm_controller/cartesian_pose_service.py))
   - ROS2 node that executes poses
   - Topic-based interface (easy to use)
   - Publishes joint commands to servo controller

3. **Predefined Poses:**
   - `home` - Neutral position
   - `reach_forward` - Extend forward
   - `reach_up` - Reach upward
   - `reach_down` - Pick objects
   - `reach_left` - Reach left
   - `reach_right` - Reach right

## How to Use (Inside Docker)

### Step 1: Start Docker & Build
```bash
# On host machine
sudo docker run -it --rm --privileged --network host \
  -v /home/tnt/arm_project/Robotic-Arm---Jarvis:/workspace \
  -v /dev:/dev -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-arm:latest

# Inside Docker
colcon build
source install/setup.bash
```

### Step 2: Start the Nodes
```bash
# Terminal 1: Start servo controller (hardware interface)
ros2 run arm_controller servo_node &

# Terminal 2: Start IK pose service
ros2 run arm_controller cartesian_pose_service
```

### Step 3: Execute Poses
```bash
# In another terminal (or background the previous command with &)

# Go to home position
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'home'"

# Reach forward
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_forward'"

# Reach up
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_up'"

# Reach down (to pick something)
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_down'"

# Reach left
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_left'"

# Reach right
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'reach_right'"
```

### Step 4: Custom Positions
```bash
# Move to a specific (x, y, z) coordinate
ros2 topic pub --once /move_to_position geometry_msgs/msg/Point "{x: 0.20, y: 0.05, z: 0.10}"
```

## Current Arm Dimensions

The IK solver is configured with these link lengths (from your URDF):
- **L1**: 0.03m - Base to shoulder height
- **L2**: 0.128m - Shoulder to elbow
- **L3**: 0.13m - Elbow to wrist
- **L4**: 0.08m - Wrist to gripper

## Testing Without Hardware

You can test the IK calculations without hardware:

```bash
# Inside Docker
cd /workspace/src/arm_controller/arm_controller
python3 ik_solver.py
```

This will show calculated joint angles for all predefined poses.

## Next Steps

### 1. Test with Your Actual Robot
Deploy to your Raspberry Pi and test with real hardware:
```bash
# On Raspberry Pi
ros2 run arm_controller servo_node &
ros2 run arm_controller cartesian_pose_service &
ros2 topic pub --once /execute_pose std_msgs/msg/String "data: 'home'"
```

### 2. Create Custom Poses
Edit [ik_solver.py](src/arm_controller/arm_controller/ik_solver.py) to add your own poses:
```python
@staticmethod
def get_my_custom_pose():
    return {
        'name': 'my_custom_pose',
        'position': (0.25, 0.10, 0.15),  # x, y, z in meters
        'orientation': (0, 0),  # pitch, roll in radians
        'gripper': 90  # gripper angle
    }
```

### 3. Integrate with Web Interface
Bridge the Flask app to ROS2 so web clicks execute IK poses:
```python
# In Flask app
import rclpy
# Publish to /execute_pose topic when button is clicked
```

### 4. Add Trajectory Planning
Create smooth motion between poses instead of instant jumps.

### 5. Update URDF for Visualization
Once poses work well, update the URDF to match for RViz visualization.

## Troubleshooting

### "Position unreachable"
- The target is outside your arm's workspace
- Try positions closer to the robot
- Check link lengths are accurate

### Joint angles seem wrong
- Verify your arm's actual measurements
- Update link lengths in ik_solver.py
- Check home position values match your reset pose

### Hardware doesn't move
- Ensure servo_node is running
- Check I2C connection
- Verify PCA9685 is detected

## Quick Reference: Coordinate System

```
         Z↑ (up)
          |
          |_____ Y (left/right)
         /
        /
       X (forward/backward)
```

- **X**: 0.2-0.3m typical reach
- **Y**: -0.2 to +0.2m left/right
- **Z**: 0.05-0.2m height

## What Makes This Special

🎯 **No manual angle tuning** - Just specify where you want the gripper
🎯 **Reachable poses** - All predefined poses work with your arm
🎯 **Based on real dimensions** - Extracted from your existing URDF
🎯 **Easy to extend** - Add new poses or custom positions easily
🎯 **ROS2 integrated** - Works with your existing servo controller

**You're ready to control your arm with inverse kinematics!** 🚀
