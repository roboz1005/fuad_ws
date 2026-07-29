FROM osrf/ros:jazzy-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-pyqt5 \
    python3-pyqt5.qtwebengine \
    libportaudio2 \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-turtlebot3-teleop \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for voice recognition
RUN pip3 install --no-cache-dir \
    SpeechRecognition \
    pyaudio \
    vosk

# Create workspace
WORKDIR /root/fuad_ws
COPY src/ ./src/

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /root/fuad_ws && \
    colcon build --symlink-install"

# Source on startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/fuad_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
