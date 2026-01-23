#!/bin/bash
# ROS2 Docker Launch Script
# Usage: ./ros2_docker.sh [command]

# Run ROS2 Humble Docker container with access to robot arm workspace
sudo docker run -it --rm \
  --privileged \
  --network host \
  #change thw workspace path according to your project location
  -v /home/tnt/arm_project/Robotic-Arm---Jarvis:/workspace \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name ros2_robot_arm \
  ros:humble \
  bash -c "cd /workspace && ${1:-bash}"
