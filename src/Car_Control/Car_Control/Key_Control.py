#!/usr/bin/env python
# encoding: utf-8
#import public lib
import sys, select, termios, tty

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your MicroRos Car!
---------------------------
Moving around:
        w
   a    s    d

w/s : forward/backward
a/d : turn left/right
wa/wd : forward left/right
sa/sd : backward left/right
space key : force stop
x/X : stop keyboard control
q/z : increase/decrease max speeds by 10%
r/f : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

linearBindings = {
    'w': 1,
    's': -1,
    'W': 1,
    'S': -1,
}

angularBindings = {
    'a': 1,
    'd': -1,
    'A': 1,
    'D': -1,
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'r': (1.1, 1),
    'f': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'R': (1.1, 1),
    'F': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
}


class Yahboom_Keybord(Node):
	def __init__(self,name):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'cmd_vel',1000)
		self.declare_parameter("linear_speed_limit",1.0)
		self.declare_parameter("angular_speed_limit",5.0)
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.settings = termios.tcgetattr(sys.stdin)
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist: key = sys.stdin.read(1)
		else: key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)


def main():
	rclpy.init()
	yahboom_keyboard = Yahboom_Keybord("yahboom_keyboard_ctrl")
	(speed, turn) = (0.2, 1.0)
	(x, th) = (0, 0)
	status = 0
	stop = False
	count = 0
	try:
		print(msg)
		print(yahboom_keyboard.vels(speed, turn))
		while (1):
			key = yahboom_keyboard.getKey()
			if key in linearBindings.keys():
				x = linearBindings[key]
				count = 0
			elif key in angularBindings.keys():
				th = angularBindings[key]
				count = 0
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				count = 0
				if speed > yahboom_keyboard.linenar_speed_limit:
					speed = yahboom_keyboard.linenar_speed_limit
					print("Linear speed limit reached!")
				if turn > yahboom_keyboard.angular_speed_limit:
					turn = yahboom_keyboard.angular_speed_limit
					print("Angular speed limit reached!")
				print(yahboom_keyboard.vels(speed, turn))
				if (status == 14): print(msg)
				status = (status + 1) % 15
			elif key == ' ':
				(x, th) = (0, 0)
				count = 0
			elif key == "x" or key == "X":
				print ("stop keyboard control: {}".format(not stop))
				stop = not stop
				(x, th) = (0, 0)
				count = 0
			else:
				count = count + 1
				if count > 4: (x, th) = (0, 0)
				if (key == '\x03'): break

			twist = Twist()
			twist.linear.x = speed * x
			twist.angular.z = turn * th
			if not stop: yahboom_keyboard.pub.publish(twist)
			if stop: yahboom_keyboard.pub.publish(Twist())
	except Exception as e: print(e)
	finally: yahboom_keyboard.pub.publish(Twist())
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, yahboom_keyboard.settings)
	yahboom_keyboard.destroy_node()
	rclpy.shutdown()
