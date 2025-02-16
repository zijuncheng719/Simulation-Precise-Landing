from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('roll', default_value='0.0', description='Roll value'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch value'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw value'),
        DeclareLaunchArgument('throttle', default_value='0.0', description='Throttle value'),
        DeclareLaunchArgument('aux1', default_value='0.0', description='Aux1 value'),
        DeclareLaunchArgument('aux2', default_value='0.0', description='Aux2 value'),

        Node(
            package='px4_ros_com',
            executable='manual_control_publisher',
            name='manual_control_publisher',
            output='screen',
            parameters=[
                {'roll': LaunchConfiguration('roll')},
                {'pitch': LaunchConfiguration('pitch')},
                {'yaw': LaunchConfiguration('yaw')},
                {'throttle': LaunchConfiguration('throttle')},
                {'aux1': LaunchConfiguration('aux1')},
                {'aux2': LaunchConfiguration('aux2')}
            ]
        )
    ])