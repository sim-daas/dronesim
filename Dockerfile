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

# Install Python requirements. If you don't have gpu, uncomment next line -torch cpu installation-
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    opencv-python \
    ultralytics

# Related to mismatch between numpy 2.x and numpy 1.x
RUN pip3 install "numpy<2.0" --force-reinstall

# Set up tmuxinator
RUN echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

# Enter bash
CMD ["/bin/bash"]
