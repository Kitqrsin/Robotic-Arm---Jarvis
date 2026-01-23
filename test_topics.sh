#!/bin/bash
# Test script to verify ROS2 topics are working

echo "=========================================="
echo "ROS2 Topic Diagnostics"
echo "=========================================="
echo ""

echo "1. Listing all topics:"
ros2 topic list
echo ""

echo "2. Checking /joint_commands info:"
ros2 topic info /joint_commands
echo ""

echo "3. Checking /joint_states info:"
ros2 topic info /joint_states
echo ""

echo "4. Echo /joint_commands (Ctrl+C to stop):"
echo "   Move sliders or click 'Send to Robot' to see output"
ros2 topic echo /joint_commands
