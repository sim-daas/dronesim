# Use ROS 2 Humble Desktop as the base image
FROM osrf/ros:humble-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    sudo \
    wget \
    curl \
    tmux \
    ruby \
    tmuxinator \
    ros-humble-ros-gz

# Install PX4
RUN git clone --depth 1 https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && git submodule update --init --recursive

# PX4 dependencies
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && \
    make px4_sitl

# Setup Micro XRCE-DDS Agent & Client
RUN cd /root && \
    git clone --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib/

# Build ROS 2 Workspace ws_sensor_combined
RUN mkdir -p /root/dronews/src && \
    cd /root/dronews/src && \
#    git clone https://github.com/sim-daas/dronesim.git && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/dronews/install/setup.bash" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/dronews && colcon build" && \
    /bin/bash -c "source /root/.bashrc"

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    opencv-python \
    ultralytics

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 install "numpy<2.0" --force-reinstall

# Set up tmuxinator
RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY models/. /root/.gz/models/
COPY models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY worlds/default_docker.sdf /PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
