#!/usr/bin/env python3
# coding=UTF-8
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
from pylimo import limo
import time
import threading
from queue import Queue
from tkinter import StringVar
from RobotController import RobotController
from RobotControlUI import RobotControlUI



def main():
    rclpy.init()
    robot_controller = RobotController()
    root = tk.Tk()
    root.title("Robot Control Interface")
    root.geometry('600x400')

    # Style pour les boutons Start et Stop
    style = ttk.Style()
    style.configure('Start.TButton', font=('Helvetica', 12), padding=5, background='green', foreground='white')
    style.configure('Stop.TButton', font=('Helvetica', 12), padding=5, background='red', foreground='white')

    robot_control_ui = RobotControlUI(root, robot_controller)

    def on_closing():
        if messagebox.askokcancel("Quit", "Do you really want to quit?"):
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    # Fonction de mise à jour périodique des coordonnées
    def update_coordinates_periodic():
        robot_control_ui.update_coordinates()
        robot_controller.check_errors()
        root.after(100, update_coordinates_periodic)

    # Lancement de la mise à jour périodique des coordonnées
    root.after(100, update_coordinates_periodic)

    root.mainloop()


if __name__ == '__main__':
    limo=limo.LIMO()
    x_g=1
    y_g=0
    theta_g=0
    main()

