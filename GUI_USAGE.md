# 🎮 Robot Arm GUI Controller

Simple Tkinter-based GUI application for controlling the Jarvis robot arm and testing ROS2 integration.

## Features

### 🎚️ Joint Angle Control
- **6 sliders** for individual joint control (0-180 degrees)
- Real-time angle display for each joint
- Smooth adjustment with 1-degree precision

### 📍 Preset Positions
- **Home** - Neutral position (90° all joints)
- **Zero** - All joints at 0°
- **Stretch** - Extended forward reach
- **Rest** - Relaxed position

### 🎯 Cartesian Control (Coming Soon)
- X, Y, Z coordinate input
- Inverse kinematics integration
- Direct end-effector positioning

### 🛡️ Safety Features
- **Emergency Stop** button - Immediately halts movement
- Real-time joint position feedback
- Visual connection status indicator

## Installation

The GUI is already integrated into your ROS2 package!

### Build the package:
```bash
cd /home/tnt/arm_project/Robotic-Arm---Jarvis
./ros2-access.sh exec
cd /workspace
colcon build --packages-select arm_controller
source install/setup.bash
```

## Usage

### Option 1: Run inside Docker (Recommended)

```bash
# Start Docker and launch GUI
./ros2-access.sh start "ros2 run arm_controller arm_gui"
```

### Option 2: Multiple terminals for full system

```bash
# Terminal 1: Start Docker
./ros2-access.sh start

# Inside Docker Terminal 1: Start servo controller
ros2 run arm_controller servo_node

# Inside Docker Terminal 2: Open new terminal
./ros2-access.sh exec

# Inside Docker Terminal 2: Launch GUI
ros2 run arm_controller arm_gui
```

## GUI Controls

### Joint Sliders
1. Adjust any slider to set desired joint angle
2. Click **"📤 Send to Robot"** to execute movement
3. Watch the arm move in RViz or physically

### Preset Buttons
- Click any preset button to load predefined position
- Click **"Send to Robot"** to execute

### Emergency Stop
- Click **"⛔ EMERGENCY STOP"** to immediately halt all movement
- Holds current position

## Testing ROS2 Integration

### Test 1: Verify Communication
```bash
# Terminal 1: Launch GUI
ros2 run arm_controller arm_gui

# Terminal 2: Monitor commands
ros2 topic echo /joint_commands
```

Move sliders and click "Send" - you should see commands published!

### Test 2: With Servo Node
```bash
# Terminal 1: Start servo controller (connects to hardware)
ros2 run arm_controller servo_node

# Terminal 2: Launch GUI
ros2 run arm_controller arm_gui
```

Now commands will control actual servos!

### Test 3: With RViz Visualization
```bash
# Terminal 1: Launch RViz
ros2 launch /workspace/view_urdf.launch.py

# Terminal 2: Launch GUI
ros2 run arm_controller arm_gui
```

Move sliders and see the arm move in RViz!

## Troubleshooting

### "Cannot connect to display"
Make sure X11 forwarding is working:
```bash
xhost +local:docker
echo $DISPLAY  # Should show :0 or similar
```

### GUI doesn't open
Check if running inside Docker with display forwarding:
```bash
./ros2-access.sh start "ros2 run arm_controller arm_gui"
```

### No response from robot
1. Verify servo_node is running: `ros2 node list`
2. Check topic communication: `ros2 topic list`
3. Monitor joint commands: `ros2 topic echo /joint_commands`

## Architecture

```
┌─────────────────┐
│   Tkinter GUI   │
│   (arm_gui.py)  │
└────────┬────────┘
         │ publishes
         ▼
┌─────────────────────┐
│ /joint_commands     │ (Float32MultiArray)
│ Topic               │
└────────┬────────────┘
         │ subscribes
         ▼
┌─────────────────┐
│  Servo Node     │
│                 │
└────────┬────────┘
         │ controls
         ▼
┌─────────────────┐
│  Robot Hardware │
│  (PCA9685)      │
└─────────────────┘
```

## Keyboard Shortcuts

- **Ctrl+C** - Close GUI (safe shutdown)
- **Esc** - Close window

## Color Coding

- 🟢 **Green** - Connected/Success
- 🔵 **Blue** - Send command button
- 🔴 **Red** - Emergency stop/Error
- 🟡 **Cyan** - Current joint values

## Next Steps

1. **Test basic movement** - Use sliders and presets
2. **Connect to hardware** - Run with servo_node
3. **Add IK integration** - Enable Cartesian control
4. **Create custom presets** - Add your own positions
5. **Record sequences** - Chain multiple movements

## Tips

- Start with small movements to test
- Use "Home" preset as safe starting position
- Keep Emergency Stop readily accessible
- Monitor ROS2 topics to verify communication
- Test in RViz before connecting hardware

---

**Created:** 2026-01-22
**Status:** ✅ Ready to use
**Dependencies:** ROS2 Humble, Python 3, Tkinter
