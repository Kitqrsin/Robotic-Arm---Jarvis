# 🔍 GUI Hardware Control Troubleshooting Checklist

## Issue: Robot not moving when using GUI

### ✅ What I Fixed:
1. **Visualization angles** - RViz now shows correct GUI angles (not calibrated hardware angles)
2. **Better logging** - Now shows both GUI angles and hardware angles in status message

### 🧪 Diagnostic Steps:

#### Step 1: Verify GUI is Publishing Commands

**Start the GUI:**
```bash
/home/tnt/arm_project/Robotic-Arm---Jarvis/ros2-access.sh start \
  "cd /workspace && colcon build --packages-select arm_controller && \
   source install/setup.bash && \
   ros2 launch /workspace/view_with_gui.launch.py"
```

**In another terminal, check topics:**
```bash
./ros2-access.sh exec
cd /workspace
source install/setup.bash

# Run diagnostic script
bash /workspace/test_topics.sh
```

**Expected output:**
- Should see `/joint_commands` and `/joint_states` in topic list
- When you move sliders, should see data on `/joint_commands`

#### Step 2: Test GUI Commands

**Move a joint slider** (e.g., Base to 120°)
**Click "Send to Robot"**

**Check terminal output for:**
```
[INFO] [arm_control_gui]: GUI angles: [120.0, 90.0, 90.0, 90.0, 90.0, 90.0] → Hardware: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**In the diagnostic terminal you should see:**
```
data:
- 30.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
```

#### Step 3: Verify RViz Updates

- Move sliders in GUI
- Click "Send to Robot"
- **Check RViz**: Robot should move in visualization
- If RViz doesn't update → visualization issue (not hardware issue)

#### Step 4: Test Real-time Mode

1. Enable **⚡ Real-time Mode** checkbox
2. Move a joint slider
3. Check info bar: Should say "✓ Sent to robot"
4. RViz should update immediately
5. Terminal should show continuous publishes

#### Step 5: Test IK

1. Enable Real-time Mode
2. Move **Z slider** to 250mm
3. Move **X slider** to 150mm
4. Watch RViz arm move
5. Green marker should appear at target position

#### Step 6: Connect Hardware (Raspberry Pi)

**On Raspberry Pi:**
```bash
cd /home/tnt/arm_project/Robotic-Arm---Jarvis
source install/setup.bash

# Run hardware interface
ros2 run arm_controller servo_node
```

**Expected output:**
```
[INFO] [arm_servo_node]: PCA9685 Servo Driver Connected!
```

**Now move sliders in GUI:**
- Physical servos should move!
- Terminal shows: `[INFO] [arm_servo_node]: Moving to: [30.0, 0.0, ...]`

---

## 🐛 Common Issues & Solutions

### Issue: RViz not updating
**Cause:** Visualization angles were wrong (FIXED)
**Test:** Move slider → Click "Send to Robot" → RViz should move now

### Issue: No topics visible
**Symptom:** `ros2 topic list` is empty or missing topics
**Solution:**
```bash
# Check if GUI node is running
ros2 node list
# Should show: /arm_control_gui

# Check publishers
ros2 topic info /joint_commands
# Should show 1 publisher
```

### Issue: Hardware node can't connect to servos
**Symptom:** `Could not connect to servo board`
**Solutions:**
```bash
# Enable I2C
sudo raspi-config
# → Interface → I2C → Enable

# Check I2C devices
i2cdetect -y 1
# Should show 0x40

# Check wiring:
# SDA → GPIO 2 (Pin 3)
# SCL → GPIO 3 (Pin 5)
# VCC → 3.3V
# GND → GND
```

### Issue: Real-time mode doesn't send commands
**Check:** 
- Is checkbox actually enabled?
- Move a slider slowly - status bar should update
- Check terminal for publish messages

### Issue: Servos move to wrong positions
**Cause:** Calibration offset
**Current:** 90° GUI = 0° hardware (upright)
**To change:** Edit line 448 in arm_gui.py:
```python
calibrated_positions = [pos - 90.0 for pos in self.target_positions]
# Change to different offset if needed
```

### Issue: IK unreachable positions
**Symptoms:** "Position unreachable" error
**Solutions:**
- Use cartesian sliders to see valid range
- Keep within workspace:
  - X: -200 to 200 mm
  - Y: -200 to 200 mm
  - Z: 50 to 350 mm
- Green marker shows target - if it's far from arm, position is invalid

---

## 📊 Expected Behavior

### Manual Mode (Real-time OFF):
1. Move sliders → Nothing publishes
2. Click "Send to Robot" → Publishes once
3. RViz updates
4. Hardware (if connected) moves once

### Real-time Mode (Real-time ON):
1. Move slider → Auto-publishes after 100ms
2. RViz updates continuously
3. Hardware (if connected) moves continuously
4. Status bar shows real-time updates

### IK Control:
1. Enter X,Y,Z coordinates OR use cartesian sliders
2. Click "Move to Position (IK)" OR enable Real-time
3. IK solver calculates angles
4. Joint sliders update automatically
5. Green marker appears in RViz
6. Click "Send to Robot" (manual) OR auto-sends (real-time)
7. Robot moves to position

---

## 🔧 Quick Test Sequence

```bash
# Terminal 1 - Start GUI
./ros2-access.sh start "cd /workspace && colcon build --packages-select arm_controller && source install/setup.bash && ros2 launch /workspace/view_with_gui.launch.py"

# Terminal 2 - Monitor topics
./ros2-access.sh exec "cd /workspace && source install/setup.bash && ros2 topic echo /joint_commands"

# In GUI:
# 1. Move Base slider to 120°
# 2. Click "Send to Robot"
# 3. Check Terminal 2 → should see: data: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 4. Check RViz → arm should move
```

---

## 📝 Summary of Changes

### Fixed in arm_gui.py:
- **Line 464:** Changed to use `self.target_positions` instead of `calibrated_positions` for RViz
- **Line 468:** Updated status message to show both GUI and hardware angles
- **Line 470:** Improved logging to distinguish GUI vs hardware values

### Result:
- ✅ RViz now shows correct visualization
- ✅ Hardware receives correct calibrated angles
- ✅ Clear separation between display angles (GUI/RViz) and hardware angles
- ✅ Better debugging with dual angle display
