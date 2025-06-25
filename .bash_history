cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && python3 uav_camera_det.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && python3 uav_camera_det.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && python3 uav_camera_det.py
cd /root/dronesim/PX4-ROS2-Gazebo-YOLOv8/
cat Dockerfile 
tmuxinator start px4_ros2_gazebo
sleep 8 && python3 mavlinkros.py
sleep 10 && python3 roskeypub.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 8 && python3 mavlinkros.py
python3 mavlinkros.py
sleep 10 && python3 roskeypub.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && python3 uav_camera_det.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 keyboard-mavsdk-test.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 8 && python3 mavlinkros.py
sleep 10 && python3 roskeypub.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
tmuxinator start px4_ros2_gazebo
ign
gz
gz sim
ign gazebo
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
python3 offboard_velocity_ned.py 
python3 offboard_position_ned.py 
python3 offboard_velocity_body.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
tmuxinator start test_mav
sleep 4 && python3 goto.py
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
python3 offboard_position_velocity_ned.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
nano offboard_velocity_ned.py 
python3 offboard_velocity_ned.py 
cat offboard_velocity_ned.py 
nano offboard_velocity_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
python3 offboard_position_velocity_ned.py 
nano offboard_position_velocity_ned.py 
tmuxinator start test_mav
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
sleep 8 && python3 mavlinkros.py
sleep 10 && python3 roskeypub.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
nano offboard_position_velocity_ned.py 
nano offboard_position__ned.py 
nano offboard_position_ned.py 
python3 offboard_position_ned.py 
nano goto.py 
ls
nano mission.py 
nano mission_raw.py.py 
nano mission_raw.py
python3 mission.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 4 && python3 goto.py
python3 offboard_position_ned.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
tmuxinator start test_mav
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 4 && python3 goto.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
tmuxinator start test_mav
sleep 4 && python3 goto.py
python3 offboard_position_velocity_ned.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cp PX4-ROS2-Gazebo-YOLOv8/test_px4.yml ~/.config/tmuxinator/test_mav.yml 
sleep 4 && python3 goto.py
python3 offboard_position_velocity_ned.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
tmuxinator start test_mav
sleep 4 && python3 offboard_position_velocity_ned.py
python3 offboard_position_ned.py 
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 8 && python3 mavlinkros.py
python3 keyboard-mavsdk-test.py 
python3 mavkeyboard-old.py 
python3 keyboard-mavsdk-test.py 
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
python3 uav_camera_det.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 4 && python3 offboard_position_velocity_ned.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
tmuxinator start test_mav
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4 MPC_XY_CRUISE=10 MPC_XY_VEL_MAX=15 
MPC_XY_CRUISE=10 MPC_XY_VEL_MAX=15 PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 6 && python3 uav_camera_det.py
sleep 10 && python3 roskeypub.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 8 && python3 mavlinkros.py
python3 keyboard-mavsdk-test.py 
python3 mavkeyboard-old.py 
python3 mavlinkros.py 
sleep 6 && python3 uav_camera_det.py
sleep 10 && python3 roskeypub.py
python3 roskeypub.py
sleep 8 && python3 mavlinkros.py
python3 mavlinkros.py 
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 8 && python3 mavlinkros.py
python3 mavkeyboard-old.py 
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 10 && python3 roskeypub.py
tmuxinator start ros2_mav
cat ~/.config/tmuxinator/px4_ros2_gazebo.yml 
tmuxinator start px4_ros2_gazebo
mv ~/.config/tmuxinator/px4_ros2_gazebo_new.yml ~/.config/tmuxinator/px4_ros2_gazebo_new.yml.bak
tmuxinator start px4_ros2_gazebo
history
tmuxinator start px4_ros2_gazebo
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 5 && python3 roskeypub.py
sleep 4 && python3 mavlinkros.py
python3 mavlinkros.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 6 && python3 uav_camera_det.py
sleep 4 && python3 offboard_position_velocity_ned.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
history
pwd
ls
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd
cd PX4-Autopilot/
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd MAVSDK-Python/examples/
python3 offboard_position_ned.py 
python3 offboard_position_velocity_ned.py 
python3 offboard_velocity_body.py 
python3 offboard_velocity_ned.py 
sleep 5 && python3 roskeypub.py
sleep 4 && python3 mavlinkros.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 4 && python3 offboard_position_velocity_ned.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 4 && python3 offboard_position_velocity_ned.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 5 && python3 roskeypub.py
sleep 4 && python3 mavlinkros.py
ls
tmuxinator start px4_ros2_gazebo
sleep 5 && python3 roskeypub.py
sleep 4 && python3 mavlinkros.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
sleep 6 && python3 uav_camera_det.py
sleep 6 && ros2 run ros_gz_image image_bridge /camera
sleep 4 && python3 offboard_position_velocity_ned.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
tmuxinator start test_mav
sleep 5 && python3 roskeypub.py
python3 roskeypub.py
sleep 4 && python3 mavlinkros.py
cd /root/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
cd
ls
