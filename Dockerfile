FROM monemati/px4_ros2_gazebo_yolov8

# Setting Working Directory
WORKDIR /root

# Build ROS 2 Workspace ws_sensor_combined
RUN mkdir -p /root/dronews/src && \
    cd /root/dronews/src && \
    #    git clone https://github.com/sim-daas/dronesim.git && \
    echo "source /root/dronews/install/setup.bash" >> /root/.bashrc && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/dronews && colcon build --symlink-install" && \
    /bin/bash -c "source /root/.bashrc"

# Remove all ROS2 repository files and keys
RUN rm -f /etc/apt/sources.list.d/ros*.list && \
    rm -f /usr/share/keyrings/ros*keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt update && \
    apt install -y ros-humble-rmw-cyclonedds-cpp

# Set Cyclone DDS as the RMW implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 install "numpy<2.0" --force-reinstall

# Enter bash
CMD ["/bin/bash"]
