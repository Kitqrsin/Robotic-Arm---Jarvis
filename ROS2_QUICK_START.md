# ROS2 Quick Start Guide

## Starting the Docker Environment

```bash
# Start Docker container
sudo docker run -it --rm --privileged --network host \
  -v /home/tnt/arm_project/Robotic-Arm---Jarvis:/workspace \
  -v /dev:/dev -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-arm:latest

# Build workspace (first time or after changes)
cd /workspace
colcon build
source install/setup.bash
```

## Running ROS2 Nodes

### Launch the Full Control System
```bash
# Start all arm control nodes
ros2 launch arm_controller arm_control.launch.py

# With debug logging
ros2 launch arm_controller arm_control.launch.py log_level:=debug
```

### Launch Visualization (RViz)
```bash
# Requires X11 forwarding or GUI environment
ros2 launch arm_controller visualization.launch.py

# Without GUI controls
ros2 launch arm_controller visualization.launch.py use_gui:=false
```

### Run Individual Nodes
```bash
# Servo controller
ros2 run arm_controller servo_node

# Joint state publisher
ros2 run arm_controller joint_state_publisher_node

# Motor calibration
ros2 run arm_controller motor_calibration

# Emergency stop
ros2 run arm_controller stop_servos
```

## Testing Commands

### Send Joint Commands
```bash
# Move all joints to 90 degrees (in radians: ~1.57)
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_gripper'], position: [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]}"

# Or using Float32MultiArray (simpler)
ros2 topic pub /joint_commands std_msgs/msg/Float32MultiArray \
  "data: [90, 90, 90, 90, 90, 90]"
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# See joint states
ros2 topic echo /joint_states

# Monitor servo commands
ros2 topic echo /joint_commands
```

### Inspect Nodes
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /arm_servo_node

# View TF tree
ros2 run tf2_tools view_frames
```

## Development Workflow

### After Code Changes
```bash
# Rebuild only changed packages
colcon build --packages-select arm_controller

# Rebuild everything
colcon build

# Source the workspace
source install/setup.bash
```

### Run Tests
```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select arm_controller
```

## Troubleshooting

### Hardware Not Detected
- Warning about Adafruit library is normal in Docker
- On Raspberry Pi, ensure I2C is enabled: `sudo raspi-config`
- Check connections: `i2cdetect -y 1`

### Launch File Not Found
```bash
# Ensure workspace is built and sourced
colcon build
source install/setup.bash
```

### Permission Denied for /dev
```bash
# Run Docker with --privileged flag (already in command)
# Or add user to necessary groups on host
```

## Next Steps

1. **Test with hardware**: Deploy to Raspberry Pi
2. **Create pose service**: Save/load arm positions
3. **Add trajectory planning**: Smooth motion between poses
4. **Web integration**: Bridge Flask app with ROS2
5. **MoveIt setup**: Advanced motion planning
