# 🔌 Hardware Integration Guide
## Using IK with Your Physical Robot Arm

## Current Setup (Simulation)

### ✅ What's Already Working:
1. **IK Solver**: `ik_solver.py` - Calculates joint angles from X,Y,Z coordinates
2. **GUI**: Real-time or manual control with sliders
3. **RViz**: 3D visualization shows the arm moving
4. **ROS2 Topics**:
   - `/joint_commands` → Float32MultiArray (for hardware)
   - `/joint_states` → JointState (for visualization)
   - `/target_position_marker` → Marker (green dot in RViz)

### 🎯 How IK Currently Works:

```
User Input (X,Y,Z) 
    ↓
IK Solver (ik_solver.py)
    ↓
Joint Angles [J1, J2, J3, J4, J5, J6]
    ↓
GUI Updates Sliders
    ↓
ROS2 Publishers:
    ├─→ /joint_states (for RViz)
    └─→ /joint_commands (for hardware)
```

---

## Hardware Connection

### Step 1: Prepare Hardware

**You Need:**
- Raspberry Pi (with I2C enabled)
- PCA9685 16-Channel Servo Driver (0x40 address)
- 6× MG996R servos (or similar)
- 5-6V, 5A+ power supply
- GPIO 17 for OE (Output Enable) pin

**Servo Connections:**
```
PCA9685 Channel → Joint
─────────────────────────
Channel 0 → Base (J1)
Channel 1 → Shoulder (J2)
Channel 2 → Elbow (J3)
Channel 4 → Wrist Pitch (J4)
Channel 5 → Wrist Roll (J5)
Channel 6 → Gripper (J6)
```

### Step 2: Test Hardware First

Before connecting to ROS2, verify servos work:

```bash
# Run calibration (sets all to 90°)
cd /home/tnt/arm_project/Robotic-Arm---Jarvis/src/arm_controller/arm_controller
python3 motor_calibration.py
```

This will:
- Initialize PCA9685
- Set all servos to 90° (centered)
- Keep them powered so you can attach servo horns

### Step 3: Run Hardware Node

You already have a hardware interface node! Run it **on the Raspberry Pi**:

```bash
# On Raspberry Pi (not in Docker)
cd /home/tnt/arm_project/Robotic-Arm---Jarvis
source install/setup.bash

# Run the servo hardware node
ros2 run arm_controller servo_node
```

This node:
- Subscribes to `/joint_commands`
- Sends angles to PCA9685 servos
- Includes safety limits (0-180°)

### Step 4: Connect GUI to Hardware

**Option A: Run GUI on Pi (Direct Connection)**
```bash
# On Raspberry Pi
ros2 launch /home/tnt/arm_project/Robotic-Arm---Jarvis/view_with_gui.launch.py
```

**Option B: Run GUI on PC, Hardware on Pi (Network)**
```bash
# On Raspberry Pi - Run hardware node
ros2 run arm_controller servo_node

# On PC (Docker) - Run GUI and RViz
# In another terminal, set ROS_DOMAIN_ID to match
export ROS_DOMAIN_ID=0
./ros2-access.sh start "cd /workspace && source install/setup.bash && ros2 launch /workspace/view_with_gui.launch.py"
```

---

## Using IK with Hardware

### Real-time Control:

1. **Launch Everything:**
   ```bash
   # Terminal 1 (Pi): Hardware
   ros2 run arm_controller servo_node
   
   # Terminal 2 (Docker): GUI + RViz
   ./ros2-access.sh start "cd /workspace && source install/setup.bash && ros2 launch /workspace/view_with_gui.launch.py"
   ```

2. **Enable Real-time Mode** in GUI
3. **Move Cartesian Sliders**:
   - X: 200 mm
   - Y: 100 mm
   - Z: 250 mm
4. **IK solves automatically** → Physical arm moves!

### Manual Control:

1. **Real-time OFF**
2. Enter coordinates:
   - X: 150
   - Y: 0
   - Z: 200
3. Click **"Move to Position (IK)"**
4. IK solves, sliders update
5. Click **"Send to Robot"**
6. Physical arm moves to position

---

## Safety Features

### Hardware Safety in servo_node.py:

```python
# Angle clamping (0-180°)
if target_angle < 0:
    target_angle = 0
if target_angle > 180:
    target_angle = 180
```

### GUI Safety:
- Motor calibration: 90° GUI = 0° servo (upright position)
- Emergency stop button
- Real-time mode rate limiting (100ms updates)

### OE Pin Safety:
The `test.py` and `app.py` have OE pin control (GPIO 17):
```python
oe_pin = OutputDevice(OE_PIN, initial_value=True)  # HIGH = disabled
oe_pin.off()  # Enable motors
oe_pin.on()   # Disable motors
```

---

## Testing Workflow

### 1. Test Joint Control First:
```bash
# Publish joint angles directly
ros2 topic pub --once /joint_commands std_msgs/msg/Float32MultiArray "data: [90, 90, 90, 90, 90, 90]"
```

### 2. Test IK in Simulation:
- Use GUI with RViz (no hardware)
- Verify IK solutions look correct
- Check green marker position

### 3. Test IK with Hardware:
- Start hardware node on Pi
- Use GUI to control
- Start with small movements
- Verify arm reaches correct positions

### 4. Test Real-time IK:
- Enable real-time mode
- Slowly move cartesian sliders
- Arm follows in real-time

---

## Advanced: Using MoveIt (Optional)

Your current geometric IK solver works great! But if you want advanced features:

### MoveIt Benefits:
- Collision avoidance
- Path planning
- Multiple IK solutions
- Joint limits checking

### To Add MoveIt:
```bash
# Install MoveIt2
sudo apt install ros-humble-moveit

# Generate MoveIt config
ros2 launch moveit_setup_assistant setup_assistant.launch.py
# Load your robot_arm.urdf
# Configure planning groups, end-effectors, etc.
```

But honestly, **your current IK solver is perfect for this arm!** It's:
- Fast (no planning overhead)
- Simple (geometric solution)
- Reliable (tested and working)

---

## Troubleshooting

### Problem: Hardware node can't find servos
```bash
# Check I2C is enabled
sudo raspi-config
# → Interface Options → I2C → Enable

# Detect I2C devices
i2cdetect -y 1
# Should show 0x40 (PCA9685)
```

### Problem: Servos jitter/buzz
- Check power supply (need 5-6V, 5A minimum)
- Verify PWM frequency: `pca.frequency = 50`
- Adjust pulse width in servo_node.py

### Problem: IK gives unreachable positions
- Check link lengths in ik_solver.py match your arm
- Verify coordinates are in meters (not mm)
- Use cartesian sliders to see range

### Problem: Arm moves to wrong position
- Verify servo channel mapping
- Check motor calibration (90° = upright)
- Adjust joint offsets in ik_solver.py

---

## Next Steps

1. ✅ **Current**: IK working in simulation
2. 🔧 **Next**: Connect hardware node on Pi
3. 🎯 **Then**: Test joint control
4. 🚀 **Finally**: Use IK with real arm!

## Summary

**Your IK is ALREADY integrated with ROS2!** You just need to:
1. Run `servo_node` on the Pi
2. Use the same GUI you're using now
3. The arm will follow your IK commands

The system is **hardware-ready** - the `/joint_commands` topic is published, you just need the Pi to subscribe and control the servos!
