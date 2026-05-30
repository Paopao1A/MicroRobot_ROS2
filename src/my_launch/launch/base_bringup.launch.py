from launch import LaunchDescription
from launch_ros.actions import Node

# 定义基础启动描述
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
            'publish_tf': False,
            'use_system_time_stamp': True,
            'stamp_offset_sec': 0.0,
        }],
    )

    imu_attitude_filter = Node(
        package='micro_robot_bringup',
        executable='imu_attitude_filter',
        name='imu_attitude_filter',
        output='screen',
        parameters=[{
            'input_imu_topic': 'imu_raw',
            'output_imu_topic': 'imu/data',
            'frame_id': 'imu_frame',
            'alpha': 0.98,
            'gravity': 9.80665,
            'accel_gate': 3.0,
            'yaw_gyro_bias': 0.0,
            'orientation_covariance': 0.05,
            'auto_calibrate_gyro': True,
            'gyro_calibration_time': 2.0,
            'gyro_calibration_min_samples': 50,
            'gyro_deadband': 0.01,
            'stationary_gyro_threshold': 0.03,
            'bias_adaptation_alpha': 0.002,
            'use_system_time_stamp': True,
            'stamp_offset_sec': 0.0,
        }],
    )

    odom_imu_fusion = Node(
        package='micro_robot_bringup',
        executable='odom_imu_fusion',
        name='odom_imu_fusion',
        output='screen',
        parameters=[{
            'input_odom_topic': 'odom',
            'input_imu_topic': 'imu/data',
            'output_odom_topic': 'odometry/filtered',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'imu_timeout_sec': 0.5,
            'publish_tf': True,
            'use_imu_orientation': True,
            'use_system_time_stamp': True,
            'stamp_offset_sec': 0.0,
        }],
    )

    laser_scan_processor = Node(
        package='micro_robot_bringup',
        executable='laser_scan_processor',
        name='laser_scan_processor',
        output='screen',
        parameters=[{
            'input_scan_topic': 'scan',
            'output_scan_topic': 'scan_filtered',
            'frame_id': 'laser_frame',
            'range_min': 0.12,
            'range_max': 8.0,
            'replace_invalid_with_inf': True,
            'warn_timeout_sec': 3.0,
            'use_system_time_stamp': True,
            'stamp_offset_sec': 0.0,
        }],
    )
    
    # 定义 base_footprint 到 base_link 的静态变换
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

    # 定义 base_link 到 imu_frame 的静态变换
    base_link_to_imu_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_frame',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.03',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_frame',
        ],
        output='screen',
    )

    # 定义 base_link 到 laser_frame 的静态变换
    base_link_to_laser_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=[
            '--x', '-0.0046412',
            '--y', '0',
            '--z', '0.094079',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
        output='screen',
    )

    # 定义 base_link 到 front_bumper_frame 的静态变换
    base_link_to_front_bumper_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_front_bumper_frame',
        arguments=[
            '--x', '0.09',
            '--y', '0',
            '--z', '0.05',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_bumper_frame',
        ],
        output='screen',
    )

    # 定义 base_link 到 camera_frame 的静态变换
    base_link_to_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_frame',
        arguments=[
            '--x', '0.11',
            '--y', '0',
            '--z', '0.12',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_frame',
        ],
        output='screen',
    )

    return LaunchDescription([
        odom_tf_broadcaster,
        imu_attitude_filter,
        odom_imu_fusion,
        laser_scan_processor,
        base_footprint_to_base_link,
        base_link_to_imu_frame,
        base_link_to_laser_frame,
        base_link_to_front_bumper_frame,
        base_link_to_camera_frame,
    ])
