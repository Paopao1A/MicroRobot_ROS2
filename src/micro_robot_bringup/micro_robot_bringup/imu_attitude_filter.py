#!/usr/bin/env python3

import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuAttitudeFilter(Node):
    def __init__(self):
        super().__init__('imu_attitude_filter')

        self.declare_parameter('input_imu_topic', 'imu_raw')
        self.declare_parameter('output_imu_topic', 'imu/data')
        self.declare_parameter('frame_id', 'imu_frame')
        self.declare_parameter('alpha', 0.98) # 滤波系数，0.98是常用的值
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('accel_gate', 3.0) # 加速度阈值，用于过滤异常值
        self.declare_parameter('yaw_gyro_bias', 0.0) # 陀螺仪偏置，用于校准，根据实际情况调整，后续可以在在launch文件中设置
        self.declare_parameter('orientation_covariance', 0.05) # 姿态方差，用于计算置信区间                     

        # 获取参数
        self.input_imu_topic = self.get_parameter('input_imu_topic').value
        self.output_imu_topic = self.get_parameter('output_imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.alpha = clamp(float(self.get_parameter('alpha').value), 0.0, 1.0)
        self.gravity = float(self.get_parameter('gravity').value)
        self.accel_gate = float(self.get_parameter('accel_gate').value)
        self.yaw_gyro_bias = float(self.get_parameter('yaw_gyro_bias').value)
        self.orientation_covariance = float(self.get_parameter('orientation_covariance').value)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_stamp_sec = None
        self.initialized = False

        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, 10) # 创建处理之后的imu话题发布者，10是队列大小，根据实际情况调整；
        self.imu_sub = self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, 10)# 创建原始话题订阅者，10是队列大小，根据实际情况调整；

        self.get_logger().info(
            'filtering %s -> %s, alpha %.3f'
            % (self.input_imu_topic, self.output_imu_topic, self.alpha)
        )

    def stamp_to_sec(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def accel_to_roll_pitch(self, ax, ay, az):# 根据加速度计算roll和pitch
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    def imu_callback(self, msg):# imu订阅回调，处理原始imu话题，发布处理之后的imu话题
        now_sec = self.stamp_to_sec(msg.header.stamp)# 转换为秒级时间戳
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z - self.yaw_gyro_bias

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)# 计算合加速度
        accel_valid = abs(accel_norm - self.gravity) <= self.accel_gate# 判断加速度是否有效

        if not self.initialized:# 初始化时，也就是第一次收到imu话题时，根据合加速度计算roll和pitch
            if accel_norm > 0.001:# 确保合加速度不为0，避免除0错误
                self.roll, self.pitch = self.accel_to_roll_pitch(ax, ay, az)
            self.last_stamp_sec = now_sec
            self.initialized = True
            return

        dt = now_sec - self.last_stamp_sec# 计算时间间隔
        self.last_stamp_sec = now_sec# 更新上次时间戳
        if dt <= 0.0 or dt > 1.0:
            return

        gyro_roll = self.roll + gx * dt # 计算陀螺仪roll，侧倾角度，这个是根据陀螺仪原始数据计算得来
        gyro_pitch = self.pitch + gy * dt # 计算陀螺仪pitch，俯仰角度
        gyro_yaw = normalize_angle(self.yaw + gz * dt)# 计算陀螺仪yaw，偏航角度

        if accel_valid and accel_norm > 0.001:
            accel_roll, accel_pitch = self.accel_to_roll_pitch(ax, ay, az)  # 根据加速度计算roll和pitch
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll     # 加速度计算结果和角速度计算结果互补滤波，这样可以限制陀螺仪的零飘以及加速度计的受惯性影响
            self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch 
        else:
            self.roll = gyro_roll
            self.pitch = gyro_pitch
        self.yaw = gyro_yaw

        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)# 计算四元数

        out = deepcopy(msg)# 复制原始话题话题
        out.header.frame_id = self.frame_id
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw

        out.orientation_covariance[0] = self.orientation_covariance
        out.orientation_covariance[4] = self.orientation_covariance
        out.orientation_covariance[8] = self.orientation_covariance

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAttitudeFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
