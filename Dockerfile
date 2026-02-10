FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-moveit \
    python3-tk \
    libcap-dev \
    libatlas-base-dev \
    v4l-utils \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    "Flask>=3.0,<4.0" \
    "Flask-Cors>=4.0" \
    gpiozero==2.0.1 \
    lgpio \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-servokit \
    numpy \
    Pillow \
    "Werkzeug>=2.0" \
    "pytest>=7.0" \
    "flake8>=6.0" \
    "black>=24.0" \
    "isort>=5.13" \
    "ruff>=0.1.0" \
    opencv-python-headless \
    ultralytics

# Install picamera2 (optional — works only on Raspberry Pi OS with libcamera)
# Falls back to OpenCV if libcamera is not available at runtime
RUN pip3 install --no-cache-dir picamera2 || echo "picamera2 install skipped (libcamera not available)"

WORKDIR /workspace

# Source ROS2 on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]
