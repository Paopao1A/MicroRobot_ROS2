import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 定义地图建模启动描述
def generate_launch_description():
    my_launch_share = get_package_share_directory('my_launch')
    use_rviz = LaunchConfiguration('use_rviz')# 是否启动 rviz 映射显示配置
    slam_params_file = LaunchConfiguration('slam_params_file')# slam_toolbox 映射参数文件路径

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_launch_share, 'launch', 'base_bringup.launch.py')
        )
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    rviz = Node(
        package='rviz2',#功能包名称
        executable='rviz2',#可执行文件名称
        name='rviz2_mapping',#节点名称
        arguments=['-d', os.path.join(my_launch_share, 'rviz', 'mapping.rviz')],# 运行参数：指定加载的 rviz 配置文件路径
        condition=IfCondition(use_rviz),# 条件启动：只有当 use_rviz 为 true 时才启动
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(# 返回启动描述，用于指定是否启动 rviz 映射显示配置
            'use_rviz',# 参数名称
            default_value='true',# 默认值为 true
            description='Start RViz with the mapping display configuration.',# 描述：是否启动 rviz 映射显示配置
        ),
        DeclareLaunchArgument(# 返回启动描述，用于指定 slam_toolbox 映射参数文件路径
            'slam_params_file',# 参数名称
            default_value=os.path.join(my_launch_share, 'config', 'slam_toolbox_mapping.yaml'),# 默认值为 slam_toolbox_mapping.yaml
            description='Full path to the slam_toolbox mapping parameters file.',# 描述：slam_toolbox 映射参数文件路径
        ),
        base_bringup,
        slam_toolbox,
        rviz,
    ])
