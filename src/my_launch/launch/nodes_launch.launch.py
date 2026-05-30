from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='car_control',
        executable='key_control',
        output='screen'
    )
    return LaunchDescription([node])
