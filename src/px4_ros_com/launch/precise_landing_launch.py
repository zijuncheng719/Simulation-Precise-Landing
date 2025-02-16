from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
            parameters=[
                {'local_home': [1.5, 0.3, 5.0]},    #在ENU坐标系下,依赖GPS，local_home的坐标为
                {'local_home_yaw': 0.0},            #在ENU坐标系下，依赖GPS，local_home的坐标为
                {'land_home': [0.2, 0.2, 1.7]},     #标定位置时，距离aruco码的X，Y坐标以及X轴欧拉角
                {'land_height': [5.0, 3.0, 0.8]},
                {'allow_err_xyz': 1.0},
                {'allow_err_yaw': 0.2},
                {'allow_visual_err_xyz': 0.1},
                {'allow_visual_err_yaw': 0.1},
                {'param_x': 0.6},
                {'param_y': -0.6},
                {'param_z': -1.0},
                {'param_yaw': 0.6}
            ]
        )
    ])