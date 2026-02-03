#!/usr/bin/env python3
# coding=UTF-8

import math
import time
import threading
from queue import Queue
from tkinter import messagebox

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quaternion(q):
    # q: geometry_msgs/Quaternion
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotController:
    """
    Contrôleur point-goal ROS2 :
    - lit la pose réelle depuis /odom
    - publie (v,w) sur /cmd_vel
    """

    def __init__(self, ros_node):
        self.node = ros_node

        # Publisher cmd_vel
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber odom
        self.sub_odom = self.node.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # Thread / état
        self.robot_status = "Stopped"
        self.thread = None
        self.queue = Queue()

        # Pose réelle (mise à jour par /odom)
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0
        self.odom_ok = False

        # Goal
        self.x_goal = 1.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # Gains
        self.dt = 0.05
        self.K_rho = 0.8
        self.K_alpha = 2.0
        self.K_beta = -0.7

        # Saturations
        self.v_max = 0.25
        self.w_max = 1.2

        self.print_debug = True

    # =========================
    # Odom callback
    # =========================
    def odom_cb(self, msg: Odometry):
        self.x_g = float(msg.pose.pose.position.x)
        self.y_g = float(msg.pose.pose.position.y)
        self.theta_g = float(yaw_from_quaternion(msg.pose.pose.orientation))
        self.odom_ok = True

    # =========================
    # Goal
    # =========================
    def set_goal(self, x, y, theta=0.0):
        self.x_goal = float(x)
        self.y_goal = float(y)
        self.theta_goal = float(theta)
        print(f"[GOAL] x={self.x_goal} y={self.y_goal} theta={self.theta_goal}")

    # =========================
    # Publish cmd_vel
    # =========================
    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd_vel.publish(msg)

    # =========================
    # Start / Stop
    # =========================
    def start_robot(self):
        if self.thread is not None and self.thread.is_alive():
            print("Robot already running.")
            return

        self.robot_status = "Started"
        print("Robot started (needs /odom and publishes /cmd_vel)")

        def robot_thread():
            try:
                # Attendre /odom (sinon contrôleur aveugle)
                t0 = time.time()
                while not self.odom_ok and (time.time() - t0) < 3.0 and self.robot_status == "Started":
                    time.sleep(0.05)

                if not self.odom_ok:
                    print("ERROR: No /odom received. Controller cannot work.")
                    self.queue.put("NoOdom")
                    self.robot_status = "Stopped"
                    self.publish_cmd(0.0, 0.0)
                    return

                while self.robot_status == "Started":
                    # lire pose réelle (mise à jour par callback)
                    x = self.x_g
                    y = self.y_g
                    theta = self.theta_g

                    dx = self.x_goal - x
                    dy = self.y_goal - y
                    rho = math.hypot(dx, dy)

                    if rho < 0.05:
                        break

                    alpha = wrap_to_pi(math.atan2(dy, dx) - theta)
                    beta = wrap_to_pi(self.theta_goal - theta - alpha)

                    v = self.K_rho * rho
                    w = self.K_alpha * alpha + self.K_beta * beta

                    # saturations
                    v = max(min(v, self.v_max), -self.v_max)
                    w = max(min(w, self.w_max), -self.w_max)

                    # publier
                    self.publish_cmd(v, w)

                    if self.print_debug:
                        print(
                            f"odom: x={x:.3f} y={y:.3f} th={theta:.3f} | "
                            f"rho={rho:.3f} alpha={alpha:.3f} beta={beta:.3f} | "
                            f"cmd: v={v:.2f} w={w:.2f}"
                        )

                    time.sleep(self.dt)

                # stop
                self.publish_cmd(0.0, 0.0)
                self.robot_status = "Stopped"
                print("Robot stopped / target reached")

            except Exception as e:
                print("ERROR in robot thread:", e)
                self.queue.put("Error")
                self.robot_status = "Stopped"
                self.publish_cmd(0.0, 0.0)

        self.thread = threading.Thread(target=robot_thread, daemon=True)
        self.thread.start()

    def stop_robot(self):
        if self.thread is not None and self.thread.is_alive():
            self.robot_status = "Stopped"
            self.thread.join(timeout=1.0)
            self.publish_cmd(0.0, 0.0)
            print("Robot stopped by user")

    # =========================
    # ROS tools (inchangés)
    # =========================
    def rvizMode(self):
        self.start_ros_nodes()

    def turtlesimMode(self):
        self.start_turtlesim_nodes()

    def rqtGraph(self):
        self.start_rqtGraph()

    def start_ros_nodes(self):
        import subprocess
        cmd = "cd ~/agx_ws && source install/setup.bash && ros2 launch limo_bringup limo_start.launch.py"
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd])

    def start_turtlesim_nodes(self):
        import subprocess
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', "ros2 run turtlesim turtlesim_node"])

    def start_rqtGraph(self):
        import subprocess
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', "rqt_graph"])

    # =========================
    # Errors
    # =========================
    def check_errors(self):
        while not self.queue.empty():
            err = self.queue.get()
            if err == "NoOdom":
                messagebox.showerror("ROS Error", "Aucun /odom reçu. Lance le bringup et vérifie le topic /odom.")
            elif err == "Error":
                messagebox.showerror("Robot Error", "Erreur thread / ROS.")
