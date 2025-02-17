cd Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600		#根据RDK X5的官方文档，RDK X5在40PIN接口上默认使能UART1，物理管脚号为8和10，IO电压为3.3V

ros2 launch usb_cam camera.launch.py # 终端1、启动相机
ros2 launch aruco_ros single.launch.py # 终端2、启动aruco_ros
ros2 topic echo /aruco_single/pose # 终端3、查看topic输出
#查看话题，输出结果图像：/aruco_single/result

ros2 run px4_ros_com offboard_control

colcon build --packages-select px4_ros_com

vim ~/Simulation-Precise-Landing/bash_files/auto_launch.bash
bash ~/Simulation-Precise-Landing/bash_files/auto_launch.bash