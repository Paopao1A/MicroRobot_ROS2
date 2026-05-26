from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='Car_Control',
        executable='Key_Control',
        output='screen'
    )
    return LaunchDescription([node])
