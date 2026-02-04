#!/usr/bin/env python3
# coding=UTF-8

import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node

from .RobotController import RobotController
from .RobotControlUI import RobotControlUI


class AppNode(Node):
    def __init__(self):
        super().__init__("app_node")


def main():
    rclpy.init()
    node = AppNode()

    robot_controller = RobotController(node)
    robot_controller.set_goal(1.0, 0.0, 0.0)

    root = tk.Tk()
    root.title("Robot Control Interface")
    root.geometry("600x400")

    style = ttk.Style()
    style.configure("Start.TButton", font=("Helvetica", 12))
    style.configure("Stop.TButton", font=("Helvetica", 12))

    ui = RobotControlUI(root, robot_controller)

    def on_close():
        if messagebox.askokcancel("Quit", "Quit application ?"):
            try:
                robot_controller.stop_robot()
            except Exception:
                pass
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    def periodic():
        ui.update_coordinates()
        robot_controller.check_errors()
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(100, periodic)

    root.after(100, periodic)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
