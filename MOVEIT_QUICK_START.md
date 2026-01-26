# MoveIt Quick Start Guide

## 🚀 Launch the Complete System

This guide shows you how to launch the fully integrated robotic arm with MoveIt motion planning.

## Prerequisites

1. Docker container running: `ros2_robot_arm`
2. Hardware connected: Raspberry Pi 5, PCA9685, 6× MG996R servos
3. ROS2 workspace built: `colcon build` completed

## Launch Sequence

### Step 1: Start Hardware Interface (Terminal 1)

```bash
./ros2-access.sh exec
source /workspace/install/setup.bash
ros2 run arm_controller servo_node
```

**What it does:**
- Initializes PCA9685 (I2C 0x40)
- Controls 6 servos on channels [0,1,2,5,4,6]
- Subscribes to `/joint_commands` for target positions
- Manages GPIO17 OE pin for emergency stop
- Provides speed control (1-100%)

**Expected output:**
```
[INFO] [servo_node]: Servo node initialized
[INFO] [servo_node]: Emergency stop is OFF - Servos enabled
```

---

### Step 2: Joint State Publisher (Terminal 2)

```bash
./ros2-access.sh exec
source /workspace/install/setup.bash
ros2 run arm_controller joint_state_publisher_node
```

**What it does:**
- Publishes current joint positions to `/joint_states`
- Provides feedback to MoveIt and RViz
- Updates at 20 Hz

**Expected output:**
```
[INFO] [joint_state_publisher_node]: Publishing joint states
```

---

### Step 3: Trajectory Executor (Terminal 3)

```bash
./ros2-access.sh exec
source /workspace/install/setup.bash
ros2 run arm_controller trajectory_executor
```

**What it does:**
- Bridges MoveIt trajectories to hardware
- Action server: `/arm_controller/follow_joint_trajectory`
- Converts radians → degrees
- Executes multi-waypoint trajectories with timing
- Provides feedback to MoveIt

**Expected output:**
```
[INFO] [trajectory_executor]: Trajectory Executor initialized
[INFO] [trajectory_executor]: Action server: /arm_controller/follow_joint_trajectory
[INFO] [trajectory_executor]: Publishing commands to: /joint_commands
```

---

### Step 4: MoveIt with RViz (Terminal 4)

```bash
./ros2-access.sh exec
source /workspace/install/setup.bash
ros2 launch arm_moveit_config moveit.launch.py
```

**What it does:**
- Launches move_group node (motion planner)
- Loads URDF, SRDF, and planning configuration
- Starts RViz with MoveIt plugin
- Provides planning and execution interface

**Expected output:**
```
[INFO] [move_group]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[INFO] [move_group]: Loading robot model 'robot_arm'...
[INFO] [move_group]: Ready to service requests
```

---

## Verification Commands

### Check All Nodes Running

```bash
ros2 node list
```

**Expected nodes:**
```
/servo_node
/joint_state_publisher_node
/trajectory_executor
/move_group
/robot_state_publisher
/rviz2
```

### Check Action Servers

```bash
ros2 action list
```

**Expected actions:**
```
/arm_controller/follow_joint_trajectory
/recognize_objects
```

### Check Topics

```bash
ros2 topic list
```

**Key topics:**
```
/joint_states          # Current positions
/joint_commands        # Hardware commands
/servo_speed           # Speed control
/emergency_stop        # E-stop state
/tf                    # Transform tree
```

### Monitor Joint Commands

```bash
ros2 topic echo /joint_commands
```

Watch commands being sent to hardware during planning.

---

## Using MoveIt in RViz

### Plan to Named Pose

1. In RViz, expand **MotionPlanning** panel
2. Under **Planning**, select planning group: **arm**
3. Under **Goal State**, select: **home** or **ready**
4. Click **Plan** button
5. Review the planned trajectory (orange visualization)
6. Click **Execute** to run on hardware

### Plan to Custom Pose

1. In RViz, drag the **interactive marker** (blue sphere at end effector)
2. Move it to desired position
3. Click **Plan** in MotionPlanning panel
4. Click **Execute** to run

### Set Start State to Current

1. In **Planning** tab, click **Update** next to "Start State"
2. Ensures planning starts from actual hardware position

---

## Python API Usage

### Example: Plan and Execute

Create a Python script `test_moveit.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupActionGoal
from std_msgs.msg import Float32MultiArray
import time

class MoveItTest(Node):
    def __init__(self):
        super().__init__('moveit_test')
        
        # For simple testing, we can use the action directly
        # Or use moveit_commander (recommended)
        
        self.get_logger().info('MoveIt test node ready')

def main():
    rclpy.init()
    node = MoveItTest()
    
    # Simple test: publish directly to /joint_commands
    # This bypasses MoveIt, good for initial testing
    pub = node.create_publisher(Float32MultiArray, '/joint_commands', 10)
    
    time.sleep(1)
    
    # Move to home position (all 90°)
    msg = Float32MultiArray()
    msg.data = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
    pub.publish(msg)
    
    node.get_logger().info('Sent home command')
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

Run it:
```bash
./ros2-access.sh exec
source /workspace/install/setup.bash
python3 test_moveit.py
```

---

## Troubleshooting

### Trajectory Executor Not Responding

**Symptom:** MoveIt plans but doesn't execute

**Check:**
```bash
ros2 action list
```

**Should see:** `/arm_controller/follow_joint_trajectory`

**Fix:** Restart trajectory_executor node

---

### Planning Failures

**Symptom:** "Unable to find valid plan"

**Causes:**
1. Start state collision
2. Goal state out of reach
3. Joint limits violated

**Debug:**
```bash
# Check current joint states
ros2 topic echo /joint_states

# Check if goal is reachable
# In RViz: Planning → Update start state to current
```

---

### Hardware Not Moving

**Symptom:** Plan executes but servos don't move

**Check:**
1. Emergency stop state:
   ```bash
   ros2 topic echo /emergency_stop
   ```
   Should be `false`

2. Servo node running:
   ```bash
   ros2 node info /servo_node
   ```

3. Commands being published:
   ```bash
   ros2 topic echo /joint_commands
   ```

---

### RViz Not Launching

**Symptom:** `ros2 launch` fails to open RViz

**Solution:**
1. Check if running in Docker with display forwarding
2. Set `DISPLAY` variable:
   ```bash
   export DISPLAY=:0
   ```
3. Or launch without RViz:
   ```bash
   ros2 launch arm_moveit_config moveit.launch.py rviz:=false
   ```

---

## Safety Reminders

1. **Always test without power first**
   - Use dry-run mode or disconnect servo power
   
2. **Emergency stop ready**
   - GPIO17 controls OE pin on PCA9685
   - Pull HIGH to disable all servos immediately
   
3. **Speed control**
   - Start with low speed (10-20%)
   - Gradually increase after testing
   
4. **Collision checking**
   - MoveIt checks collisions in planning
   - But doesn't know about external objects
   - Always monitor execution

5. **Joint limits**
   - Current limits: ±π for most joints
   - May need tightening based on physical constraints
   - Edit `config/joint_limits.yaml` if needed

---

## Next Steps

### 1. Calibrate Joint Limits
Fine-tune limits in `joint_limits.yaml` based on actual hardware constraints.

### 2. Add Collision Objects
Define workspace obstacles for safer planning:
```python
# Using MoveIt Python API
scene = moveit_commander.PlanningSceneInterface()
scene.add_box("table", pose, size=(1.0, 1.0, 0.05))
```

### 3. Integrate with GUI
Modify `arm_gui.py` to add MoveIt planning button alongside direct IK control.

### 4. Test Complex Trajectories
Try pick-and-place operations, circular paths, or constrained motion.

---

## System Architecture

```
┌──────────────┐
│    RViz      │ ← User plans motions
└──────┬───────┘
       │
┌──────▼───────┐
│  MoveIt      │ ← Plans collision-free paths
│ (move_group) │    OMPL planners (RRTConnect, RRTstar)
└──────┬───────┘
       │ FollowJointTrajectoryAction
       │ (20-50 waypoints, radians, timing)
       │
┌──────▼────────┐
│  Trajectory   │ ← Executes waypoints sequentially
│   Executor    │    Converts radians → degrees
└──────┬────────┘
       │ Float32MultiArray
       │ (single target, degrees)
       │
┌──────▼────────┐
│  Servo Node   │ ← Hardware interface
│  (PCA9685)    │    PWM generation
└──────┬────────┘
       │ I2C commands
       │
┌──────▼────────┐
│  6× MG996R    │ ← Physical servos
│   Servos      │    Move the arm
└───────────────┘
```

---

## Configuration Files Reference

### URDF
- **Path:** `/workspace/robot_urdf/robot_fixed.urdf`
- **Purpose:** 3D model, kinematics, collision geometry

### SRDF
- **Path:** `src/arm_moveit_config/config/robot_arm.srdf`
- **Purpose:** Planning groups, end effector, collision pairs

### Planning Configuration
- **Path:** `src/arm_moveit_config/config/ompl_planning.yaml`
- **Planners:** RRTConnect (fast), RRTstar (optimal)

### Joint Limits
- **Path:** `src/arm_moveit_config/config/joint_limits.yaml`
- **Limits:** Velocity (1.5 rad/s), acceleration (3.0 rad/s²)

### Kinematics
- **Path:** `src/arm_moveit_config/config/kinematics.yaml`
- **Solver:** KDL (Kinematics and Dynamics Library)

---

## 🎉 You're Ready!

The MoveIt integration is complete and ready for motion planning. Start with simple named poses (home, ready) and gradually move to complex trajectories.

**Happy Planning! 🤖**
