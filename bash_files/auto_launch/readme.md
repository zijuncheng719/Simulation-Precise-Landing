# 脚本说明

## 1. auto_launch_test.bash是测试用的bash脚本
## 2. auto_launch.bash是精准降落工作空间启动的bash脚本
## 3. auto_root_launch.bash是系统自启动服务用的脚本
## 4. RDK X5自启动项

```bash
vim ~/ws_precision_visland/bash_files/auto_launch.bash
bash ~/ws_precision_visland/bash_files/auto_launch.bash	#测试

sudo vim /etc/init.d/auto_root_launch.bash
sudo chmod +x /etc/init.d/auto_root_launch.bash 
sudo update-rc.d auto_root_launch.bash defaults
sudo systemctl enable auto_root_launch.bash
systemctl status auto_root_launch.bash.service

#分析启动服务错误
systemctl status auto_root_launch.bash.service
#查看日志分析
sudo journalctl -u auto_root_launch.bash.service
#停止服务
sudo systemctl stop auto_root_launch.bash.service
#auto_root_launch.bash.service 服务的配置文件已经发生了变化,重新加载 Systemd 的配置
sudo systemctl daemon-reload
#重启服务
sudo systemctl restart auto_root_launch.bash.service
#杀死node
pkill -f ros2.*talker
```

## 5. 问题
在root运行usb_com报错
```bash
[ERROR] [usb_cam_node_exe-1]: process has died [pid 4455, exit code -6, cmd '/opt/ros/humble/lib/usb_cam/usb_cam_node_exe --ros-args -r __node:=camera1 --params-file /opt/ros/humble/share/usb_cam/config/params_1.yaml -r image_raw:=camera1/image_raw -r image_raw/compressed:=camera1/image_compressed -r image_raw/compressedDepth:=camera1/compressedDepth -r image_raw/theora:=camera1/image_raw/theora -r camera_info:=camera1/camera_info'].
```

## 6. 管理节点

ros2 node是一个用于节点管理的命令行工具，它允许用户执行与节点相关的各种操作。

### 列出节点：

要查看当前活动的节点，可以使用：

- `ros2 node list`

这个命令会列出当前系统中所有活跃的节点名称。

### 获取节点信息：

要获取特定节点的更多信息，如订阅的主题、发布的主题、提供的服务等，可以使用：

- `ros2 node info <node_name>`

### 终止节点

在ROS 2中，直接终止节点的命令不像ROS 1中的 `rosnode kill` 那样直观。要终止一个节点，通常需要找到运行该节点的进程，然后使用系统命令（如 `kill` 在Linux上）来终止该进程。可以通过组合使用 `p` 和 `grep` 命令来查找节点的进程ID，然后使用 `kill` 命令终止它。

例如，如果你的节点名称包含 `talker`，你可以使用以下命令查找并终止这个节点：

1. `pgrep -f ros2.*talker`

这将列出所有名称中包含 `talker` 的ROS 2节点的进程ID。然后，你可以使用 `kill` 命令加上进程ID来终止节点：

- `kill -SIGINT <process_id>`

或者，如果想要终止所有匹配的进程，可以使用：

- `pkill -f ros2.*talker`

节点杀死参考：[https://blog.yanghong.dev/ros2-node/](https://blog.yanghong.dev/ros2-node/)