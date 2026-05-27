from setuptools import find_packages, setup

package_name = 'micro_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='MicroRobot host-side bringup helpers.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = micro_robot_bringup.odom_tf_broadcaster:main',
            'imu_attitude_filter = micro_robot_bringup.imu_attitude_filter:main',
            'odom_imu_fusion = micro_robot_bringup.odom_imu_fusion:main',
        ],
    },
)
