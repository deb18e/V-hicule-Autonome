#!/usr/bin/env python3
# coding=UTF-8

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node

from RobotController import RobotController
from RobotControlUI import RobotControlUI

from visualization_msgs.msg import Marker, MarkerArray


# ======================================================
# Node ROS2 pour visualisation RViz (obstacles)
# ======================================================
class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacles_markers',
            10
        )

    def publish_obstacles(self):
        markers = MarkerArray()

        obstacles = [
            (0.8, 0.0, 0.2),
            (0.8, 0.5, 0.2),
            (0.8, -0.5, 0.2),
        ]

        for i, (x, y, r) in enumerate(obstacles):
            m = Marker()
            m.header.frame_id = "odom"   # IMPORTANT
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0

            m.scale.x = r * 2
            m.scale.y = r * 2
            m.scale.z = 0.3

            m.color.a = 0.9
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0

            markers.markers.append(m)

        self.marker_pub.publish(markers)
        self.get_logger().info("Obstacles published")


# ======================================================
# MAIN
# ======================================================
def main():

    # ---------- Init ROS2 ----------
    rclpy.init()

    # ---------- Robot Controller ----------
    robot_controller = RobotController()

    # objectif initial (modifiable plus tard)
    robot_controller.set_goal(1.0, 0.0, 0.0)

    # ---------- Node RViz ----------
    viz_node = VisualizationNode()

    # ---------- Tkinter ----------
    root = tk.Tk()
    root.title("Robot Control Interface")
    root.geometry("600x400")

    # styles boutons
    style = ttk.Style()
    style.configure('Start.TButton', font=('Helvetica', 12))
    style.configure('Stop.TButton', font=('Helvetica', 12))

    ui = RobotControlUI(root, robot_controller)

    # ---------- Bouton obstacles RViz ----------
    def publish_obstacles():
        viz_node.publish_obstacles()
        messagebox.showinfo(
            "RViz",
            "Obstacles publiés sur /obstacles_markers\n"
            "RViz Fixed Frame = odom"
        )

    btn_obs = ttk.Button(
        root,
        text="Publish Obstacles (RViz)",
        command=publish_obstacles
    )
    btn_obs.place(x=360, y=350)

    # ---------- Fermeture propre ----------
    def on_close():
        if messagebox.askokcancel("Quit", "Quit application ?"):
            try:
                robot_controller.stop_robot()
            except Exception:
                pass
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # ---------- Boucle périodique ----------
    def periodic_update():
        # MAJ interface
        ui.update_coordinates()

        # erreurs robot
        robot_controller.check_errors()

        # spin ROS non bloquant
        rclpy.spin_once(viz_node, timeout_sec=0.0)

        root.after(100, periodic_update)

    root.after(100, periodic_update)

    # ---------- Lancement GUI ----------
    root.mainloop()

    # ---------- Cleanup ----------
    viz_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
