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

       # commande1 = "cd ~/agx_ws && source install/setup.bash && ros2 run turtlesim turtlesim_node && read -p 'Appuyez sur Enter pour fermer le terminal'"
        commande2 = "cd ~/va_ws && source install/setup.bash && ros2 run va_controller mouvement && read -p 'Appuyez sur Enter pour fermer le terminal'"

    #    subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande1], check=True)
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande2], check=True)



        def robot_thread():
            try:
                limo.EnableCommand()
                L = 0.2
                Kphi = 0.9
                Kbeta = 1
                s = 10.5
                Kalpha = s - Kphi
                odopresleft = limo.GetLeftWheelOdeom()
                odopresright = limo.GetRightWheelOdom()
                x_init = 1   
                y_init = 1
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
                        v = 0.5
                
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
    
    def check_errors(self):
        while not self.queue.empty():
            error = self.queue.get()
            if error == "Error":
                # Affichez un message d'erreur dans l'interface graphique ou effectuez toute autre action nécessaire
                messagebox.showerror("Error", "Communication error with LIMO. Please check the connection.")


class RobotControlUI:
    def __init__(self, root, robot_controller):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry('600x400')  # Ajustez la taille de la fenêtre

        self.robot_controller = robot_controller
        self.x_var = tk.StringVar()
        self.y_var = tk.StringVar()
        self.theta_var = tk.StringVar()
        self.x_t = 0
        self.y_t = 0

        # Chargez l'image de fond
        background_image = Image.open("background_image.jpeg")  # Remplacez "background_image.jpg" par le chemin de votre propre image
        background_image = background_image.resize((600, 400), Image.ANTIALIAS)
        self.background_image = ImageTk.PhotoImage(background_image)

        self.create_widgets()

    def create_widgets(self):
        # Afficher l'image de fond
        background_label = tk.Label(self.root, image=self.background_image)
        background_label.place(relwidth=1, relheight=1)

        # Style pour les boutons
        style = ttk.Style()
        style.configure('TButton', font=('Helvetica', 12), padding=5)

        # Label
        label = ttk.Label(self.root, text="Navigation Interface", font=('Helvetica', 16, 'bold'), background='white')
        label.grid(row=1, column=0, columnspan=3, pady=10, sticky=tk.W)

        # Button to send command - Start (en vert)
        start_button = ttk.Button(self.root, text="Start", command=self.send_start_command, style='Start.TButton')
        start_button.grid(row=2, column=0, pady=10, padx=(0, 10), sticky=tk.W)

        # Button to send command - Stop (en rouge)
        stop_button = ttk.Button(self.root, text="Stop", command=self.send_stop_command, style='Stop.TButton')
        stop_button.grid(row=3, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)


        # Labels pour afficher les coordonnées en temps réel
        x_label = ttk.Label(self.root, text="X:", font=('Helvetica', 12), background='white')
        x_label.grid(row=4, column=0, pady=5, sticky=tk.W)
        ttk.Label(self.root, textvariable=self.x_var, font=('Helvetica', 12), background='white').grid(row=4, column=1, pady=5, sticky=tk.W)

        y_label = ttk.Label(self.root, text="Y:", font=('Helvetica', 12), background='white')
        y_label.grid(row=5, column=0, pady=5, sticky=tk.W)
        ttk.Label(self.root, textvariable=self.y_var, font=('Helvetica', 12), background='white').grid(row=5, column=1, pady=5, sticky=tk.W)

        theta_label = ttk.Label(self.root, text="Theta:", font=('Helvetica', 12), background='white')
        theta_label.grid(row=6, column=0, pady=5, sticky=tk.W)
        ttk.Label(self.root, textvariable=self.theta_var, font=('Helvetica', 12), background='white').grid(row=6, column=1, pady=5, sticky=tk.W)

    def update_coordinates(self):
        # Mettre à jour les variables Tkinter avec les coordonnées en temps réel
        self.x_var.set(f'{self.robot_controller.x_g:.2f}')
        self.y_var.set(f'{self.robot_controller.y_g:.2f}')
        self.theta_var.set(f'{self.robot_controller.theta_g:.2f}')

    def send_start_command(self):
        self.execute_command('start')

    def send_stop_command(self):
        self.execute_command('stop')

    def execute_command(self, command):
        if command == 'start':
            self.robot_controller.start_robot()
        elif command == 'stop':
            self.robot_controller.stop_robot()
        else:
            messagebox.showinfo("Info", "Unknown command")


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
    x_g=0
    y_g=0
    theta_g=0
    main()

