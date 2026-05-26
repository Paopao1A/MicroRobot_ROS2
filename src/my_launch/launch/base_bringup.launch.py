from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    odom_tf_broadcaster = Node(
        package='micro_robot_bringup',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'input_odom_topic': 'odom_raw',
            'output_odom_topic': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_odom': True,
            'publish_tf': True,
        }],
    )

    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.05',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link',
        ],
        output='screen',
    )

    return LaunchDescription([
        odom_tf_broadcaster,
        base_footprint_to_base_link,
    ])
