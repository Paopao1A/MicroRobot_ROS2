#!/usr/bin/env python3
# encoding: utf-8

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

msg = """
Control Your MicroRos Car!  key_control_2
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
    'Q': (1.1, 1.1),
    'Z': (0.9, 0.9),
    'W': (1.1, 1.0),
    'X': (0.9, 1.0),
    'E': (1.0, 1.1),
    'C': (1.0, 0.9),
}


class KeyControl2(Node):
    def __init__(self):
        super().__init__('key_control_2')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_vels(self, speed, turn):
        print('currently:\tspeed %.3f\tturn %.3f' % (speed, turn))

    def publish_stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = KeyControl2()
    speed = 0.2
    turn = 1.0
    x = 0
    th = 0
    status = 0
    stop = False
    count = 0

    try:
        print(msg)
        node.print_vels(speed, turn)
        while True:
            key = node.get_key()
            if key in ('s', 'S'):
                stop = not stop
                x = 0
                th = 0
                count = 0
                print('stop keyboard control: {}'.format(stop))
            elif key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings:
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                if speed > node.linear_speed_limit:
                    speed = node.linear_speed_limit
                    print('Linear speed limit reached!')
                if turn > node.angular_speed_limit:
                    turn = node.angular_speed_limit
                    print('Angular speed limit reached!')
                node.print_vels(speed, turn)
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key in (' ', 'k', 'K'):
                x = 0
                th = 0
                count = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = speed * x
            twist.angular.z = turn * th
            if stop:
                node.publish_stop()
            else:
                node.pub.publish(twist)
    except Exception as exc:
        print(exc)
    finally:
        node.publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()
