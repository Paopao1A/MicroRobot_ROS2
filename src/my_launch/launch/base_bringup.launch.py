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
            'publish_tf': False,
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
        }],
    )
    
    # 静态发布坐标系，把base_link也发布到tf中，base_link固定在base_footprint坐标系上0.05m处，跟随base_footprint坐标系移动
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
        imu_attitude_filter,
        odom_imu_fusion,
        base_footprint_to_base_link,
    ])
