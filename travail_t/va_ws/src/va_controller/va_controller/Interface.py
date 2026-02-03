#!/usr/bin/env python3
# coding=UTF-8

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node

from pylimo import limo

from RobotController import RobotController
from RobotControlUI import RobotControlUI


# =========================
# ROS2 Node pour RViz/RQt + Obstacles (Markers)
# =========================
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class VisualizationNode(Node):
    """
    Node ROS2 qui publie des obstacles (markers) visibles dans RViz.
    Tu peux les placer autour du robot pour "simuler" des obstacles visuels.
    """
    def __init__(self):
        super().__init__('robot_visualization_node')
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles_markers', 10)

    def publish_obstacles(self, obstacles):
        """
        obstacles: list of dict:
          [{"x":1.0,"y":0.0,"r":0.2}, ...]
        """
        msg = MarkerArray()

        for i, obs in enumerate(obstacles):
            m = Marker()
            m.header.frame_id = "map"     # ou "odom" selon ta config TF
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = "obstacles"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = float(obs["x"])
            m.pose.position.y = float(obs["y"])
            m.pose.position.z = 0.1

            m.pose.orientation.w = 1.0

            r = float(obs.get("r", 0.2))
            m.scale.x = r * 2.0
            m.scale.y = r * 2.0
            m.scale.z = 0.3

            # couleur (rouge)
            m.color.a = 0.9
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0

            msg.markers.append(m)

        self.marker_pub.publish(msg)


def main():
    rclpy.init()

    # ---- Robot controller (ton contrôle LIMO)
    robot_controller = RobotController()

    # ---- Node ROS2 pour markers/visualisation
    viz_node = VisualizationNode()

    # ---- Tkinter UI
    root = tk.Tk()
    root.title("Robot Control Interface")
    root.geometry('600x400')

    # Style Start/Stop
    style = ttk.Style()
    style.configure('Start.TButton', font=('Helvetica', 12), padding=5)
    style.configure('Stop.TButton', font=('Helvetica', 12), padding=5)

    robot_control_ui = RobotControlUI(root, robot_controller)

    # =========================
    # Ajout d’obstacles (simple)
    # =========================
    # Exemple : obstacles autour du robot (visuels RViz)
    # -> tu peux changer les coordonnées comme tu veux
    default_obstacles = [
        {"x": 0.8, "y": 0.0, "r": 0.2},
        {"x": 0.8, "y": 0.5, "r": 0.2},
        {"x": 0.8, "y": -0.5, "r": 0.2},
        {"x": 1.2, "y": 0.2, "r": 0.25},
    ]

    def publish_obstacles_button():
        viz_node.publish_obstacles(default_obstacles)
        messagebox.showinfo("Obstacles", "Obstacles publiés sur /obstacles_markers (RViz).")

    # Bouton obstacles (dans l’UI, en bas)
    obstacles_btn = ttk.Button(root, text="Publish Obstacles (RViz)", command=publish_obstacles_button)
    obstacles_btn.place(x=380, y=350)


    # =========================
    # Fermeture propre
    # =========================
    def on_closing():
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            try:
                robot_controller.stop_robot()
            except Exception:
                pass
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    # =========================
    # Boucle périodique :
    # - update GUI (x,y,theta)
    # - check erreurs
    # - spin ROS2
    # =========================
    def update_coordinates_periodic():
        # Mise à jour des coordonnées dans l'UI
        robot_control_ui.update_coordinates()

        # Check erreurs LIMO
        robot_controller.check_errors()

        # Spin ROS2 (non-bloquant)
        rclpy.spin_once(viz_node, timeout_sec=0.0)

        # Replanifier
        root.after(100, update_coordinates_periodic)

    root.after(100, update_coordinates_periodic)
    root.mainloop()

    # Cleanup ROS
    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # Connexion LIMO
    limo = limo.LIMO()

    # Objectif par défaut (si tu l’utilises ailleurs)
    x_g = 1
    y_g = 0
    theta_g = 0

    main()