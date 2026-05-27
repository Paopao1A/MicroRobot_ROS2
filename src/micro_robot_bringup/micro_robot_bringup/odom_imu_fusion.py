#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class OdomImuFusion(Node):
    def __init__(self):
        super().__init__('odom_imu_fusion')

        self.declare_parameter('input_odom_topic', 'odom')
        self.declare_parameter('input_imu_topic', 'imu/data')
        self.declare_parameter('output_odom_topic', 'odometry/filtered')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu_orientation', True)

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.input_imu_topic = self.get_parameter('input_imu_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu_orientation = bool(self.get_parameter('use_imu_orientation').value)

        self.latest_imu = None
        self.latest_imu_time = None

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)# 发布融合之后的odom话题
        self.tf_broadcaster = TransformBroadcaster(self)# 发布tf话题
        self.imu_sub = self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, 10)# 订阅imu/data话题
        self.odom_sub = self.create_subscription(Odometry, self.input_odom_topic, self.odom_callback, 10)# 订阅odom话题

        self.get_logger().info(
            'fusing %s + %s -> %s'
            % (self.input_odom_topic, self.input_imu_topic, self.output_odom_topic)
        )

    def stamp_to_sec(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds * 1e-9
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def imu_callback(self, msg):# imu/data话题回调函数，主要是进行imu数据快照，更新时间
        self.latest_imu = msg
        self.latest_imu_time = self.stamp_to_sec(msg.header.stamp)

    def has_fresh_imu(self, odom_msg):# 检查是否有新鲜的imu数据
        if self.latest_imu is None or self.latest_imu_time is None:
            return False
        odom_time = self.stamp_to_sec(odom_msg.header.stamp)
        return abs(odom_time - self.latest_imu_time) <= self.imu_timeout_sec

    def odom_callback(self, msg):
        fused = deepcopy(msg)
        fused.header.frame_id = self.odom_frame
        fused.child_frame_id = self.base_frame

        #把odom的四元数姿态orientation替换为imu的，横摆角速度也更新替换为imu的数据
        if self.use_imu_orientation and self.has_fresh_imu(msg):# 如果使用imu数据，且有新鲜的imu数据
            fused.pose.pose.orientation = self.latest_imu.orientation # 更新odom的orientation
            fused.pose.covariance[21] = self.latest_imu.orientation_covariance[0]
            fused.pose.covariance[28] = self.latest_imu.orientation_covariance[4]
            fused.pose.covariance[35] = self.latest_imu.orientation_covariance[8]
            fused.twist.twist.angular = self.latest_imu.angular_velocity # 更新odom的angular_velocity
            fused.twist.covariance[21] = self.latest_imu.angular_velocity_covariance[0]
            fused.twist.covariance[28] = self.latest_imu.angular_velocity_covariance[4]
            fused.twist.covariance[35] = self.latest_imu.angular_velocity_covariance[8]

        self.odom_pub.publish(fused)# 发布融合之后的odom话题

        if self.publish_tf:# 发布tf话题
            transform = TransformStamped()
            transform.header.stamp = fused.header.stamp # 更新tf的时间戳
            transform.header.frame_id = self.odom_frame # 更新tf的父坐标系，也就是odom坐标系
            transform.child_frame_id = self.base_frame # 更新tf的子坐标系，也就是base_footprint坐标系
            transform.transform.translation.x = fused.pose.pose.position.x
            transform.transform.translation.y = fused.pose.pose.position.y
            transform.transform.translation.z = fused.pose.pose.position.z
            transform.transform.rotation = fused.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
