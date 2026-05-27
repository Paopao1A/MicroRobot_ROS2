#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('input_odom_topic', 'odom_raw') # 输入的odom话题，也就是ESP32传过来的原始odom数据
        self.declare_parameter('output_odom_topic', 'odom') # 输出的odom话题，也就是处理之后的odom数据
        self.declare_parameter('odom_frame', 'odom') # odom话题的frame_id
        self.declare_parameter('base_frame', 'base_footprint') # base话题的frame_id
        self.declare_parameter('publish_odom', True) # 是否发布odom话题
        self.declare_parameter('publish_tf', True) # 是否发布tf话题

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.odom_pub = None
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)# 创建处理之后的odom话题发布者，10是队列大小，根据实际情况调整；

        self.tf_broadcaster = TransformBroadcaster(self) # 创建tf广播器
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            10,
        ) # 创建odom_raw话题订阅者

        tf_state = 'enabled' if self.publish_tf else 'disabled'
        self.get_logger().info(
            'bridging %s -> %s, tf %s'
            % (self.input_odom_topic, self.output_odom_topic, tf_state)
        ) # 打印日志

    def odom_callback(self, msg):# odom_raw话题回调函数
        odom_msg = deepcopy(msg) # 复制原始odom数据
        odom_msg.header.frame_id = self.odom_frame # 设置odom话题的frame_id，传过来的是odom_raw话题的frame_id，转换成odom话题的frame_id
        odom_msg.child_frame_id = self.base_frame # 设置base话题的frame_id

        if self.publish_odom and self.odom_pub is not None: # 发布处理之后的odom数据
            self.odom_pub.publish(odom_msg)

        if not self.publish_tf: # 如果不发布tf话题，直接返回，后续IMU和odom融合之后发布更准确的tf，这里只做验证
            return

        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
