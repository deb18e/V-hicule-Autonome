#!/usr/bin/env python3
# coding=UTF-8

import math
import time
import threading
from queue import Queue
from tkinter import messagebox

from geometry_msgs.msg import Twist


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RobotController:
    """
    Contrôleur point-goal (x*, y*, theta*) -> commande (v, w) publiée sur /cmd_vel
    - v : linear.x (m/s)
    - w : angular.z (rad/s)
    """

    def __init__(self, ros_node):
        self.node = ros_node

        # Publisher cmd_vel
        self.pub_cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)

        # Thread / état
        self.robot_status = "Stopped"
        self.thread = None
        self.queue = Queue()

        # Pose affichée (estimation interne pour UI)
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

        # Goal
        self.x_goal = 1.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # Paramètres / gains
        self.dt = 0.05
        self.K_rho = 0.8
        self.K_alpha = 2.0
        self.K_beta = -0.7

        # Saturations
        self.v_max = 0.30     # m/s
        self.w_max = 1.50     # rad/s

        self.print_debug = True

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
    # Start
    # =========================
    def start_robot(self):
        if self.thread is not None and self.thread.is_alive():
            print("Robot already running.")
            return

        self.robot_status = "Started"
        print("Robot started (v,w) -> /cmd_vel")

        def robot_thread():
            try:
                x = self.x_g
                y = self.y_g
                theta = self.theta_g

                while self.robot_status == "Started":

                    dx = self.x_goal - x
                    dy = self.y_goal - y
                    rho = math.hypot(dx, dy)

                    # stop proche du but
                    if rho < 0.05:
                        break

                    alpha = wrap_to_pi(math.atan2(dy, dx) - theta)
                    beta = wrap_to_pi(self.theta_goal - theta - alpha)

                    # Commande (v,w)
                    v = self.K_rho * rho
                    w = self.K_alpha * alpha + self.K_beta * beta

                    # Saturations
                    v = max(min(v, self.v_max), -self.v_max)
                    w = max(min(w, self.w_max), -self.w_max)

                    # Publier cmd_vel
                    self.publish_cmd(v, w)

                    # Mise à jour d'état (pour UI seulement)
                    x += v * math.cos(theta) * self.dt
                    y += v * math.sin(theta) * self.dt
                    theta = wrap_to_pi(theta + w * self.dt)

                    self.x_g = x
                    self.y_g = y
                    self.theta_g = theta

                    if self.print_debug:
                        print(
                            f"x={x:.3f} y={y:.3f} th={theta:.3f} | "
                            f"rho={rho:.3f} alpha={alpha:.3f} beta={beta:.3f} | "
                            f"v={v:.2f} w={w:.2f}"
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

    # =========================
    # Stop
    # =========================
    def stop_robot(self):
        if self.thread is not None and self.thread.is_alive():
            self.robot_status = "Stopped"
            self.thread.join(timeout=1.0)
            self.publish_cmd(0.0, 0.0)
            print("Robot stopped by user")

    # =========================
    # Outils (inchangés)
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
    # Erreurs
    # =========================
    def check_errors(self):
        while not self.queue.empty():
            err = self.queue.get()
            if err == "Error":
                messagebox.showerror("Robot Error", "Erreur thread / commande / communication.")
