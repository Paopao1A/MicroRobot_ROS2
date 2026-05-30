#!/usr/bin/env python3

import math
from copy import deepcopy

import rclpy
from rclpy.duration import Duration
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
        self.declare_parameter('auto_calibrate_gyro', True) # 是否自动校准陀螺仪
        self.declare_parameter('gyro_calibration_time', 2.0) # 校准陀螺仪的时间
        self.declare_parameter('gyro_calibration_min_samples', 50) # 校准陀螺仪的样本数
        self.declare_parameter('gyro_deadband', 0.01)# 陀螺仪死区,防止静止时还存在零飘
        self.declare_parameter('stationary_gyro_threshold', 0.03)
        self.declare_parameter('bias_adaptation_alpha', 0.002)
        self.declare_parameter('use_system_time_stamp', True)
        self.declare_parameter('stamp_offset_sec', 0.05)

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
        self.gyro_deadband = max(0.0, float(self.get_parameter('gyro_deadband').value))# 陀螺仪死区,防止静止时还存在零飘
        self.stationary_gyro_threshold = max(0.0, float(self.get_parameter('stationary_gyro_threshold').value))# 静止时陀螺仪零飘系数的阈值
        self.bias_adaptation_alpha = clamp(float(self.get_parameter('bias_adaptation_alpha').value), 0.0, 1.0)# 自适应更新零飘系数的alpha值
        self.use_system_time_stamp = bool(self.get_parameter('use_system_time_stamp').value)
        self.stamp_offset_sec = float(self.get_parameter('stamp_offset_sec').value)

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
        self.last_input_time = self.get_clock().now()
        self.input_warned = False

        self.imu_pub = self.create_publisher(Imu, self.output_imu_topic, 10) # 发布过滤后的imu数据
        self.imu_sub = self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, 10)# 订阅原始imu数据
        self.input_warn_timer = self.create_timer(1.0, self.warn_if_no_input)

        self.get_logger().info(
            'filtering %s -> %s, alpha %.3f, gyro calibration %s'
            % (self.input_imu_topic, self.output_imu_topic, self.alpha,
               'enabled' if self.auto_calibrate_gyro else 'disabled')
        )

    def warn_if_no_input(self):
        if self.input_warned:
            return
        elapsed = (self.get_clock().now() - self.last_input_time).nanoseconds * 1e-9
        if elapsed >= 3.0:
            self.input_warned = True
            self.get_logger().warn('no imu_raw messages received; check ESP32 /imu_raw publisher')

    def stamp_to_sec(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def accel_to_roll_pitch(self, ax, ay, az):# 加速度转换为roll和pitch
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch

    def apply_gyro_deadband(self, value):
        if self.gyro_deadband <= 0.0:
            return value
        abs_value = abs(value)
        if abs_value <= self.gyro_deadband:
            return 0.0
        return math.copysign(abs_value - self.gyro_deadband, value)

    def update_adaptive_gyro_bias(self, raw_gx, raw_gy, raw_gz, gx, gy, gz, accel_valid):# 更新自适应陀螺仪零飘系数
        if self.bias_adaptation_alpha <= 0.0 or self.stationary_gyro_threshold <= 0.0:
            return gx, gy, gz
        if not accel_valid:
            return gx, gy, gz
        if max(abs(gx), abs(gy), abs(gz)) > self.stationary_gyro_threshold:
            return gx, gy, gz

        # 零飘系数更新的条件：静止且加速度可信时，才自适应更新零飘系数
        alpha = self.bias_adaptation_alpha
        self.gyro_bias_x = (1.0 - alpha) * self.gyro_bias_x + alpha * raw_gx
        self.gyro_bias_y = (1.0 - alpha) * self.gyro_bias_y + alpha * raw_gy
        self.gyro_bias_z = (1.0 - alpha) * self.gyro_bias_z + alpha * (raw_gz + self.manual_yaw_gyro_bias)
        return (
            raw_gx - self.gyro_bias_x,
            raw_gy - self.gyro_bias_y,
            raw_gz - self.gyro_bias_z,
        )

    def update_gyro_calibration(self, now_sec, gx, gy, gz):# 更新陀螺仪校准
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
        enough_time = elapsed >= self.gyro_calibration_time# 校准时间是否足够
        enough_samples = self.calibration_count >= self.gyro_calibration_min_samples
        if not (enough_time and enough_samples):
            return False

        self.gyro_bias_x = self.calibration_sum_x / self.calibration_count # 计算陀螺仪的校准值后的roll和pitch
        self.gyro_bias_y = self.calibration_sum_y / self.calibration_count # 计算陀螺仪的校准值后的roll和pitch
        self.gyro_bias_z = self.calibration_sum_z / self.calibration_count + self.manual_yaw_gyro_bias # 计算陀螺仪的校准值后的roll和pitch
        self.calibration_done = True # 校准完成
        self.last_stamp_sec = now_sec
        self.get_logger().info(
            'gyro bias calibrated: x %.6f, y %.6f, z %.6f rad/s'
            % (self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z)
        )
        return True

    def imu_callback(self, msg):# imu数据回调函数
        self.last_input_time = self.get_clock().now()
        self.input_warned = False
        now_sec = self.stamp_to_sec(msg.header.stamp)# 获取当前时间戳
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        raw_gx = msg.angular_velocity.x
        raw_gy = msg.angular_velocity.y
        raw_gz = msg.angular_velocity.z

        if not self.update_gyro_calibration(now_sec, raw_gx, raw_gy, raw_gz):# 更新陀螺仪校准
            return

        gx = raw_gx - self.gyro_bias_x # 得到校准之后的陀螺仪相应轴的角速度
        gy = raw_gy - self.gyro_bias_y # 
        gz = raw_gz - self.gyro_bias_z # 

        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)# 计算加速度模
        accel_valid = abs(accel_norm - self.gravity) <= self.accel_gate# 加速度是否有效
        gx, gy, gz = self.update_adaptive_gyro_bias(raw_gx, raw_gy, raw_gz, gx, gy, gz, accel_valid)# 更新自适应陀螺仪零飘系数
        gx = self.apply_gyro_deadband(gx)# 应用陀螺仪死区
        gy = self.apply_gyro_deadband(gy)# 应用陀螺仪死区
        gz = self.apply_gyro_deadband(gz)# 应用陀螺仪死区

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

        gyro_roll = self.roll + gx * dt # 计算陀螺仪的roll角度
        gyro_pitch = self.pitch + gy * dt # 计算陀螺仪的pitch角度
        gyro_yaw = normalize_angle(self.yaw + gz * dt)# 计算陀螺仪的yaw角度

        if accel_valid and accel_norm > 0.001:
            accel_roll, accel_pitch = self.accel_to_roll_pitch(ax, ay, az)  # 计算加速度的roll和pitch
            self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll     # 计算roll
            self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
        else:
            self.roll = gyro_roll
            self.pitch = gyro_pitch
        self.yaw = gyro_yaw

        if abs(self.yaw) < self.gyro_deadband:# 陀螺仪死区，防止静止时还存在零飘
            self.yaw = 0.0
        if abs(self.roll) < self.gyro_deadband:
            self.roll = 0.0
        if abs(self.pitch) < self.gyro_deadband:
            self.pitch = 0.0

        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)# 计算四元数

        out = deepcopy(msg)# 复制原始imu数据
        if self.use_system_time_stamp:
            out.header.stamp = (self.get_clock().now() + Duration(seconds=self.stamp_offset_sec)).to_msg()
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
