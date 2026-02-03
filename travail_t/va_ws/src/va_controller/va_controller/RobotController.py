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

class RobotController():
    
    def __init__(self):
        self.robot_status = "Stopped"
        self.thread = None 
        self.queue = Queue()
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0


	

    def start_robot(self):
	
        if self.thread is not None and self.thread.is_alive():
            print("Robot is already running.")
            return

        self.robot_status = "Started"
        print("Robot started")


        def robot_thread():
            try:
                L = 0.2
                Kphi = 0.9
                Kbeta = -2
                Kalpha = 5
                odopresleft = limo.GetLeftWheelOdeom()
                odopresright = limo.GetRightWheelOdom()
                x_init = -1   
                y_init = -1
                theta_init = 0
                x = x_init
                y = y_init
                theta = theta_init
                limo.SetMotionCommand(linear_vel=0, steering_angle=0)

                time.sleep(1)

                while x < 0.05 or y < 0.05 or self.robot_status != "Stopped":
                    if self.robot_status == "Stopped":
                        break
                    rho = math.sqrt(pow(x, 2) + pow(y, 2))
                    beta = -math.atan(y / x)
                    alpha = -beta - theta

                    w = Kbeta * beta + Kalpha * alpha
                    v = Kphi * rho
                    gamma = math.atan((L * w) / v)

                    if gamma > 0.488:
                        gamma = 0.488
                    elif gamma < -0.488:
                        gamma = -0.488
                    if v > 0.5:
                        v = 0.3
                
                    dl = (limo.GetLeftWheelOdeom() - odopresleft) / 1000.0
                    dr = (limo.GetRightWheelOdom() - odopresright) / 1000.0
                    odopresleft = limo.GetLeftWheelOdeom()
                    odopresright = limo.GetRightWheelOdom()
                
                    limo.SetMotionCommand(linear_vel=v, steering_angle=gamma)

                    delta_theta = (dr - dl) / 0.16879
                    d = (dl + dr) / 2
                    x_prime = d * math.cos(theta)
                    y_prime = d * math.sin(theta)
                    theta_prime = delta_theta
                    x = x + x_prime
                    y = y + y_prime
                    theta = theta + theta_prime
                    self.x_g = x_init-x
                    self.y_g = y_init-y
                    self.theta_g = theta_init-theta

            except Exception as e:
                print(f"Error in robot_thread: {e}")
                self.queue.put("Error")  # Placez un indicateur d'erreur dans la file d'attente
                raise  # Ajoutez cette ligne pour afficher la trace complète de l'erreur

        self.thread = threading.Thread(target=robot_thread)
        self.thread.start()

    def stop_robot(self):

        if self.thread is not None and self.thread.is_alive():
            self.robot_status = "Stopped"
            self.thread.join()  # Attendez la fin du thread
            print("Robot stopped")
        else:
            print("No running robot to stop.")


    def rvizMode(self):
        self.start_ros_nodes()

    def turtlesimMode(self):
        self.start_turtlesim_nodes()

    def rqtGraph(self):
        self.start_rqtGraph()

    def start_ros_nodes(self):
    
        commande1 = "cd ~/agx_ws && source install/setup.bash && ros2 launch limo_bringup limo_start.launch.py && read -p 'Appuyez sur Enter pour fermer le terminal'"
        commande2 = "cd ~/agx_ws && source install/setup.bash && ros2 launch turtlebot3_cartographer cartographer.launch.py && read -p 'Appuyez sur Enter pour fermer le terminal'"
        commande3 = "cd ~/agx_ws && source install/setup.bash && ros2 run turtlebot3_teleop teleop_keyboard && read -p 'Appuyez sur Enter pour fermer le terminal'"
	
	
	
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande1], check=True)
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande2], check=True)
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande3], check=True)


    def start_turtlesim_nodes(self):
    
        commande1 = "ros2 run turtlesim turtlesim_node && read -p 'Appuyez sur Enter pour fermer le terminal'"
        commande2 = "ros2 run turtlesim turtle_teleop_key && read -p 'Appuyez sur Enter pour fermer le terminal'"

        commande3 = "cd ~/va_ws && source install/setup.bash && ros2 run va_controller mouvement && read -p 'Appuyez sur Enter pour fermer le terminal'"     
	
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande1], check=True)
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande2], check=True)
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande3], check=True)
		
		
    def start_rqtGraph(self):
    
        commande1 = "rqt_graph && read -p 'Appuyez sur Enter pour fermer le terminal'"
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande1], check=True)  


    def check_errors(self):
        while not self.queue.empty():
            error = self.queue.get()
            if error == "Error":
                # Affichez un message d'erreur dans l'interface graphique ou effectuez toute autre action nécessaire
                messagebox.showerror("Error", "Communication error with LIMO. Please check the connection.")
