#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time



class mouvement(Node):

	def __init__(self):
		super().__init__("mouvement")	
		self.limo=limo.LIMO()
		self.limo.EnableCommand()
		self.subsc=self.create_subscription(Pose,"/turtle1/pose",self.callback,10)
	def callback(self,twist:Pose):
		self.limo.SetMotionCommand(linear_vel=twist.linear_velocity,steering_angle=twist.theta)
		self.get_logger().info(str(self.limo.GetLinearVelocity()))

def main(args=None):
	rclpy.init(args=args)
	node=mouvement()
	rclpy.spin(node)
	rclpu.shutdown()

main()
