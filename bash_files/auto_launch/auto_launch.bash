#!/bin/bash
sleep 10
# 启动Micro XRCE-DDS Agent
cd /home/sunrise/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600 &
sleep 10

# 启动相机
source /opt/ros/humble/setup.bash
source /home/sunrise/Simulation-Precise-Landing/install/setup.bash
ros2 launch usb_cam camera.launch.py &
sleep 10

# 启动aruco_ros
source /opt/ros/humble/setup.bash
source /home/sunrise/Simulation-Precise-Landing/install/setup.bash
ros2 launch aruco_ros single.launch.py &
sleep 10

# 启动offboard_control
source /opt/ros/humble/setup.bash
source /home/sunrise/Simulation-Precise-Landing/install/setup.bash
ros2 launch px4_ros_com visual_precise_landing_launch.py &
sleep 10