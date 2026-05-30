import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 定义导航启动描述
def generate_launch_description():
    my_launch_share = get_package_share_directory('my_launch')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_launch_share, 'launch', 'base_bringup.launch.py')
        )
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        arguments=['-d', os.path.join(my_launch_share, 'rviz', 'navigation.rviz')],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/MicroRos/maps/micro_robot_map.yaml'),
            description='Full path to the saved map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(my_launch_share, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Start RViz with the navigation display configuration.',
        ),
        base_bringup,
        nav2_bringup,
        rviz,
    ])
