from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false'),
        Node(
            package='ros_adapter',
            executable='server',
            namespace='ros_adapter',
            name='ros_adapter',
            parameters=[
            {
                "server_ip" : "0.0.0.0",
                "port": "30053"
            }]
        )
    ])