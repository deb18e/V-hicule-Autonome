#!/usr/bin/env python3
# coding=UTF-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
import math
from pylimo import limo
import time

class MouvementNode(Node):
    def __init__(self):
        super().__init__('mouvement_node')
        self.limo = limo.LIMO()
        self.limo.EnableCommand()

        self.x_init_sub = self.create_subscription(Float64, '/xinit', self.x_init_callback, 10)
        self.y_init_sub = self.create_subscription(Float64, '/yinit', self.y_init_callback, 10)
        self.pose_pub = self.create_publisher(Pose2D, '/robot_pose', 10)

        self.L = 0.2
        self.Kphi = 0.9
        self.Kbeta = 0.2
        self.s = 10.5
        self.Kalpha = self.s - self.Kphi

        self.odopresleft = self.limo.GetLeftWheelOdeom()
        self.odopresright = self.limo.GetRightWheelOdom()

        self.x_init = -1
        self.y_init = -0.5
        self.theta_init = math.pi
        self.x = self.x_init
        self.y = self.y_init
        self.theta = self.theta_init

        self.limo.SetMotionCommand(linear_vel=0, steering_angle=0)
        time.sleep(1)

    def x_init_callback(self, msg):
        self.x_init = msg.data

    def y_init_callback(self, msg):
        self.y_init = msg.data

    def publish_pose(self):
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

    def run(self):
        while self.x < 0.05 or self.y < 0.05:
            rho = math.sqrt(pow(self.x, 2) + pow(self.y, 2))
            beta = -math.atan(self.y / self.x)
            alpha = -beta - self.theta

            w = self.Kbeta * beta + self.Kalpha * alpha
            v = self.Kphi * rho
            gamma = math.atan((self.L * w) / v)

            print(f"{self.x:.3f} {self.y:.3f} {gamma:.3f} {beta:.3f}")

            if gamma > 0.488:
                gamma = 0.488
            elif gamma < -0.488:
                gamma = -0.488
            if v > 0.5:
                v = 0.5

            dl = (self.limo.GetLeftWheelOdeom() - self.odopresleft) / 1000.0
            dr = (self.limo.GetRightWheelOdom() - self.odopresright) / 1000.0
            self.odopresleft = self.limo.GetLeftWheelOdeom()
            self.odopresright = self.limo.GetRightWheelOdom()

            self.limo.SetMotionCommand(linear_vel=v, steering_angle=gamma)

            delta_theta = (dr - dl) / 0.17079
            d = (dl + dr) / 2
            x_prime = d * math.cos(self.theta)
            y_prime = d * math.sin(self.theta)
            theta_prime = delta_theta

            self.x = self.x + x_prime
            self.y = self.y + y_prime
            self.theta = self.theta + theta_prime

            self.publish_pose()


def main(args=None):
    rclpy.init(args=args)
    node = MouvementNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
