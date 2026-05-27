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
        self.declare_parameter('alpha', 0.98) # 滤波系数
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('accel_gate', 3.0) # 加速度门限
        self.declare_parameter('yaw_gyro_bias', 0.0) # 校准的yaw陀螺仪零飘系数
        self.declare_parameter('orientation_covariance', 0.05) # 姿态方差
        self.declare_parameter('auto_calibrate_gyro', True)
        self.declare_parameter('gyro_calibration_time', 2.0)
        self.declare_parameter('gyro_calibration_min_samples', 50)

        # ????
        self.input_imu_topic = self.get_parameter('input_imu_topic').value
        self.output_imu_topic = self.get_parameter('output_imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.alpha = clamp(float(self.get_parameter('alpha').value), 0.0, 1.0)
        self.gravity = float(self.get_parameter('gravity').value)
        self.accel_gate = float(self.get_parameter('accel_gate').value)
        self.manual_yaw_gyro_bias = float(self.get_parameter('yaw_gyro_bias').value)
        self.orientation_covariance = float(self.get_parameter('orientation_covariance').value)
        self.auto_calibrate_gyro = bool(self.get_parameter('auto_calibrate_gyro').value)
        self.gyro_calibration_time = float(self.get_parameter('gyro_calibration_time').value)
        self.gyro_calibration_min_samples = int(self.get_parameter('gyro_calibration_min_samples').value)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_stamp_sec = None
        self.initialized = False

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = self.manual_yaw_gyro_bias
        self.calibration_start_sec = None
        self.calibration_count = 0
        self.calibration_sum_x = 0.0
        self.calibration_sum_y = 0.0
        self.calibration_sum_z = 0.0
        self.calibration_done = not self.auto_calibrate_gyro

        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, 10) # 发布过滤后的imu数据
        self.imu_sub = self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, 10)# 订阅原始imu数据

        self.get_logger().info(
            'filtering %s -> %s, alpha %.3f, gyro calibration %s'
            % (self.input_imu_topic, self.output_imu_topic, self.alpha,
               'enabled' if self.auto_calibrate_gyro else 'disabled')
        )

    def stamp_to_sec(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def accel_to_roll_pitch(self, ax, ay, az):# 加速度转换为roll和pitch
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    def update_gyro_calibration(self, now_sec, gx, gy, gz):
        if self.calibration_done:
            return True

        if self.calibration_start_sec is None:
            self.calibration_start_sec = now_sec
            self.get_logger().info(
                'keep robot still, calibrating gyro bias for %.1f s'
                % self.gyro_calibration_time
            )

        self.calibration_count += 1
        self.calibration_sum_x += gx
        self.calibration_sum_y += gy
        self.calibration_sum_z += gz

        elapsed = now_sec - self.calibration_start_sec
        enough_time = elapsed >= self.gyro_calibration_time
        enough_samples = self.calibration_count >= self.gyro_calibration_min_samples
        if not (enough_time and enough_samples):
            return False

        self.gyro_bias_x = self.calibration_sum_x / self.calibration_count
        self.gyro_bias_y = self.calibration_sum_y / self.calibration_count
        self.gyro_bias_z = self.calibration_sum_z / self.calibration_count + self.manual_yaw_gyro_bias
        self.calibration_done = True
        self.last_stamp_sec = now_sec
        self.get_logger().info(
            'gyro bias calibrated: x %.6f, y %.6f, z %.6f rad/s'
            % (self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z)
        )
        return True

    def imu_callback(self, msg):# imu数据回调函数
        now_sec = self.stamp_to_sec(msg.header.stamp)# 获取当前时间戳
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        raw_gx = msg.angular_velocity.x
        raw_gy = msg.angular_velocity.y
        raw_gz = msg.angular_velocity.z

        if not self.update_gyro_calibration(now_sec, raw_gx, raw_gy, raw_gz):
            return

        gx = raw_gx - self.gyro_bias_x
        gy = raw_gy - self.gyro_bias_y
        gz = raw_gz - self.gyro_bias_z

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)# 计算加速度模
        accel_valid = abs(accel_norm - self.gravity) <= self.accel_gate# 加速度是否有效

        if not self.initialized:# 初始化时，使用加速度计算roll和pitch
            if accel_norm > 0.001:# 加速度模大于0.001，才计算roll和pitch
                self.roll, self.pitch = self.accel_to_roll_pitch(ax, ay, az)
            self.last_stamp_sec = now_sec
            self.initialized = True
            return

        dt = now_sec - self.last_stamp_sec# 计算时间间隔
        self.last_stamp_sec = now_sec# 更新上次时间戳
        if dt <= 0.0 or dt > 1.0:
            return

        gyro_roll = self.roll + gx * dt # 计算陀螺仪的roll
        gyro_pitch = self.pitch + gy * dt # 计算陀螺仪的pitch
        gyro_yaw = normalize_angle(self.yaw + gz * dt)# 计算陀螺仪的yaw

        if accel_valid and accel_norm > 0.001:
            accel_roll, accel_pitch = self.accel_to_roll_pitch(ax, ay, az)  # 计算加速度的roll和pitch
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll     # 计算roll
            self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
        else:
            self.roll = gyro_roll
            self.pitch = gyro_pitch
        self.yaw = gyro_yaw

        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)# 计算四元数

        out = deepcopy(msg)# 复制原始imu数据
        out.header.frame_id = self.frame_id
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz

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
