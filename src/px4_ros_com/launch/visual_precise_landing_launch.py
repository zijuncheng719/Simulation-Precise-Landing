from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simu_flag',
            default_value='False',
            description='Flag to indicate if simulation mode is enabled'
        ),
        DeclareLaunchArgument(
            'offboard_position_test',
            default_value='False',
            description='Flag to indicate if offboard position test is enabled'
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
            parameters=[
                {'simu_flag': LaunchConfiguration('simu_flag')},
                {'offboard_position_test': LaunchConfiguration('offboard_position_test')},
                {'local_home': [0, 0, -3.5]},
                {'land_height': [-3.5, -2.0, -1.0]},
                {'offset_aruco_position_frd': [0.0, 0.0, 0.0]},
                {'offset_aruco_yaw_frd': 0.0},
                {'param_yaw': 0.3},
                {'allow_err_xyz': 1.0},
                {'allow_err_yaw': 0.2},
                {'allow_visual_err_xyz': 0.06},
                {'allow_visual_err_yaw': 0.10},
                {'local_home_yaw': 0.0},
                {'loss_of_aruco_protection_time': 1},    # 1 seconds
                {'limit_area': [10.0, 10.0, 10.0]}    
            ]
        )
    ])