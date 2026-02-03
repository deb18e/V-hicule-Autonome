#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import TeleopCommand  # Import the TeleopCommand message type
import time

class Mouvement(Node):

    def __init__(self):
        super().__init__("mouvement") 
        self.limo = limo.LIMO()
        self.limo.EnableCommand()
        
        # Create a subscriber for the turtlesim_teleop_key topic
        self.teleop_sub = self.create_subscription(
            TeleopCommand, "/turtlesim_teleop_key", self.teleop_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.2, self.send_vel_cmd)

    def teleop_callback(self, msg):
        # Callback function to handle teleop commands
        linear_vel = msg.linear
        angular_vel = msg.angular
        self.limo.SetLinearVelocity(linear_vel)
        self.limo.SetAngularVelocity(angular_vel)

    def send_vel_cmd(self):
        msg = Twist()
        msg.linear.x = self.limo.GetLinearVelocity()
        msg.angular.z = self.limo.GetAngularVelocity()
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mouvement()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
