#写入,WARNING:在root下,运行ros相关程序，往往会报很多错误，一般与摄像头有关，一个很好的解决办法就是将启动命令移到user下进行执行，可以很好的规避这些问题。
#!/bin/bash

### BEGIN INIT INFO
# Provides:          auto_root_launch.bash.service
# Required-Start:    $all
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start auto_root_launch.bash.service at boot time
# Description:       Enable service provided by auto_root_launch.bash.service
### END INIT INFO

# 设置环境变量（如果需要）
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=0
export ROS_LOG_DIR=/var/log/ros

# 设置ROS 2环境变量
source /opt/ros/humble/setup.bash
source /home/sunrise/Simulation-Precise-Landing/install/setup.bash

# 定义启动脚本路径
LAUNCH_SCRIPT=/home/sunrise/Simulation-Precise-Landing/bash_files/auto_launch.bash

# 检查启动脚本是否存在
if [ ! -f "$LAUNCH_SCRIPT" ]; then
  echo "Launch script not found: $LAUNCH_SCRIPT"
  exit 1
fi

# 以 sunrise 用户身份启动脚本
su - sunrise -c "bash $LAUNCH_SCRIPT"

# 记录启动的进程ID
echo $! > /var/run/auto_root_launch.pid

exit 0