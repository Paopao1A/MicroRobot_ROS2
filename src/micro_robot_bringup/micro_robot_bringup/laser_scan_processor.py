#!/usr/bin/env python3

import math
from copy import deepcopy

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanProcessor(Node):
    def __init__(self):
        super().__init__('laser_scan_processor')

        self.declare_parameter('input_scan_topic', 'scan')
        self.declare_parameter('output_scan_topic', 'scan_filtered')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('replace_invalid_with_inf', True)
        self.declare_parameter('warn_timeout_sec', 3.0)
        self.declare_parameter('use_system_time_stamp', True)
        self.declare_parameter('stamp_offset_sec', 0.05)

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.replace_invalid_with_inf = bool(self.get_parameter('replace_invalid_with_inf').value)
        self.warn_timeout_sec = float(self.get_parameter('warn_timeout_sec').value)
        self.use_system_time_stamp = bool(self.get_parameter('use_system_time_stamp').value)
        self.stamp_offset_sec = float(self.get_parameter('stamp_offset_sec').value)

        self.last_input_time = self.get_clock().now()
        self.input_warned = False

        self.scan_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            10,
        )
        self.input_warn_timer = self.create_timer(1.0, self.warn_if_no_input)

        self.get_logger().info(
            'processing %s -> %s, frame %s, range %.2f..%.2f m'
            % (self.input_scan_topic, self.output_scan_topic, self.frame_id,
               self.range_min, self.range_max)
        )

    def warn_if_no_input(self):
        if self.input_warned:
            return
        elapsed = (self.get_clock().now() - self.last_input_time).nanoseconds * 1e-9
        if elapsed >= self.warn_timeout_sec:
            self.input_warned = True
            self.get_logger().warn('no scan messages received; check ESP32 /scan publisher')

    def normalize_range(self, value):
        if math.isfinite(value) and self.range_min <= value <= self.range_max:# 确保距离在指定范围内
            return float(value)
        if self.replace_invalid_with_inf:# 替换无效值为无穷大
            return math.inf
        return 0.0

    def scan_callback(self, msg):
        self.last_input_time = self.get_clock().now()
        self.input_warned = False

        out = deepcopy(msg)# 深拷贝原始消息，避免修改原始消息
        if self.use_system_time_stamp:
            out.header.stamp = (self.get_clock().now() + Duration(seconds=self.stamp_offset_sec)).to_msg()
        out.header.frame_id = self.frame_id# 设置新的帧ID
        out.range_min = self.range_min# 设置新的距离最小值
        out.range_max = self.range_max# 设置新的距离最大值
        out.ranges = [self.normalize_range(value) for value in msg.ranges]# 对每个距离值进行归一化处理,for ... in ...含义就是遍历msg.ranges列表中的每个元素，然后每个值放进新的列表给out.ranges
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
