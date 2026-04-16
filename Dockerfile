FROM ros:humble

# ---- System dependencies (single layer) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# ---- Python dependencies (single layer, no cache) ----
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
    opencv-python-headless \
    ultralytics \
    moveit-commander \
    py_trees \
    && pip3 cache purge 2>/dev/null || true

# Install picamera2 (optional — works only on Raspberry Pi OS with libcamera)
# Falls back to OpenCV if libcamera is not available at runtime
RUN pip3 install --no-cache-dir picamera2 || echo "picamera2 install skipped (libcamera not available)"

WORKDIR /workspace

# Source ROS2 on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]
