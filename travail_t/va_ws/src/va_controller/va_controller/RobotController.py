#!/usr/bin/env python3
# coding=UTF-8

import math
import time
import threading
from queue import Queue
from tkinter import messagebox

import rclpy
from std_msgs.msg import Float32


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RobotController:

    def __init__(self, ros_node):
        """
        ros_node : un Node ROS2 déjà créé dans le main.
        On publie gamma et omega dessus.
        """
        self.node = ros_node

        # publishers gamma et omega
        self.pub_gamma = self.node.create_publisher(Float32, "/cmd_gamma", 10)
        self.pub_omega = self.node.create_publisher(Float32, "/cmd_omega", 10)

        # état robot
        self.robot_status = "Stopped"
        self.thread = None
        self.queue = Queue()

        # pose estimée (pour affichage UI)
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

        # objectif
        self.x_goal = 1.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # paramètres
        self.L = 0.2         # empattement
        self.dt = 0.05

        # gains
        self.K_rho = 0.6
        self.K_alpha = 1.5
        self.K_beta = -0.6

        # saturations
        self.gamma_max = 0.5
        self.omega_max = 1.5

        # debug
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
    # Publisher gamma/omega
    # =========================
    def publish_cmd(self, gamma, omega):
        msg_g = Float32()
        msg_w = Float32()
        msg_g.data = float(gamma)
        msg_w.data = float(omega)

        self.pub_gamma.publish(msg_g)
        self.pub_omega.publish(msg_w)

    # =========================
    # Start / Stop
    # =========================
    def start_robot(self):
        if self.thread is not None and self.thread.is_alive():
            print("Robot already running.")
            return

        self.robot_status = "Started"
        print("Robot started")

        def robot_thread():
            try:
                # état local
                x = self.x_g
                y = self.y_g
                theta = self.theta_g

                while self.robot_status == "Started":

                    dx = self.x_goal - x
                    dy = self.y_goal - y
                    rho = math.hypot(dx, dy)

                    if rho < 0.05:
                        break

                    # erreurs d’angle
                    alpha = wrap_to_pi(math.atan2(dy, dx) - theta)
                    beta = wrap_to_pi(self.theta_goal - theta - alpha)

                    # commande angulaire (omega)
                    omega = self.K_alpha * alpha + self.K_beta * beta
                    omega = max(min(omega, self.omega_max), -self.omega_max)

                    # on choisit une vitesse "virtuel" v pour définir gamma
                    # (car sur Ackermann : gamma = atan(L*omega / v))
                    v_virtual = min(self.K_rho * rho, 0.3)
                    if abs(v_virtual) < 1e-3:
                        gamma = 0.0
                    else:
                        gamma = math.atan((self.L * omega) / v_virtual)

                    gamma = max(min(gamma, self.gamma_max), -self.gamma_max)

                    # Publier la commande (gamma, omega)
                    self.publish_cmd(gamma, omega)

                    # Mise à jour "simulation" juste pour affichage UI
                    # On utilise v_virtual pour intégrer (affichage seulement)
                    x += v_virtual * math.cos(theta) * self.dt
                    y += v_virtual * math.sin(theta) * self.dt
                    theta += omega * self.dt
                    theta = wrap_to_pi(theta)

                    self.x_g = x
                    self.y_g = y
                    self.theta_g = theta

                    if self.print_debug:
                        print(f"x={x:.3f} y={y:.3f} th={theta:.3f} | rho={rho:.3f} alpha={alpha:.3f} beta={beta:.3f} | omega={omega:.3f} gamma={gamma:.3f}")

                    time.sleep(self.dt)

                # stop robot
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
    # Modes (tes fonctions)
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
                messagebox.showerror("Robot Error", "Erreur communication / thread robot.")
