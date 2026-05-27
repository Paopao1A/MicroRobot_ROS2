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

        self.declare_parameter('input_odom_topic', 'odom_raw')
        self.declare_parameter('output_odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.odom_pub = None
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            10,
        )

        tf_state = 'enabled' if self.publish_tf else 'disabled'
        self.get_logger().info(
            'bridging %s -> %s, tf %s'
            % (self.input_odom_topic, self.output_odom_topic, tf_state)
        )

    def odom_callback(self, msg):
        odom_msg = deepcopy(msg)
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        if self.publish_odom and self.odom_pub is not None:
            self.odom_pub.publish(odom_msg)

        if not self.publish_tf:
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
