#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Sensor(Node):
	limo=limo.LIMO()
	limo.EnableCommand()
	def __init__(self):
		super().__init__("Sensor")	
		#self.cmd_vel_pub=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
		self.timer = self.create_timer(1.0,self.sensor_call)
	def sensor_call(self):
		self.get_logger().info("sa")
		#msg=Twist()
		#msg.linear.x=limo.GetLinearVelocity()
		#msg.angular.z=limo.GetAngularVelocity()
		#self.cmd_vel_pub.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node=Sensor()
	rclpy.spin(node)
	rclpu.shutdown()


