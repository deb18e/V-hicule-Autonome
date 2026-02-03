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




class RobotController():
	
    def __init__(self):
        self.robot_status = "Stopped"
        self.thread = None  # Ajout d'une variable de thread
        
      #  self.x_init_publisher = self.create_publisher(Float64, '/xinit', 10)
      #  self.y_init_publisher = self.create_publisher(Float64, '/yinit', 10)

    def start_robot(self):
        if self.thread is not None and self.thread.is_alive():
            print("Robot is already running.")
            return

        self.robot_status = "Started"
        print("Robot started")
        
        def robot_thread():
            

            # ... (le reste du code de la boucle while)
######
        
            limo.EnableCommand()
            L=0.2
    
            Kphi=0.9
            Kbeta=0.2
            s=10.5
            Kalpha=s-Kphi
            odopresleft=limo.GetLeftWheelOdeom()
            odopresright=limo.GetRightWheelOdom()
            x_init=-1
            y_init=-0.5
            theta_init=math.pi
            x=x_init
            y=y_init
            theta=theta_init
            limo.SetMotionCommand(linear_vel=0,steering_angle=0)

            time.sleep(1)

            while x<0.05 or y<0.05 or self.robot_status != "Stopped":
                if self.robot_status == "Stopped":
                    break
                rho=math.sqrt(pow(x,2)+pow(y,2))
                beta= - math.atan(y/x)
                alpha= - beta-theta
        
                w=Kbeta*beta+Kalpha*alpha
                v=Kphi*rho
                gamma=math.atan((L*w)/v)
                print(f"{x:.3f} {y:.3f} {gamma:.3f} {beta:.3f}")
                if gamma>0.488:
                    gamma=0.488
                elif gamma<-0.488:
                    gamma=-0.488
                if v>0.5:
                    v=0.5
        
                dl=(limo.GetLeftWheelOdeom()-odopresleft)/1000.0
                dr=(limo.GetRightWheelOdom()-odopresright)/1000.0
                odopresleft=limo.GetLeftWheelOdeom()
                odopresright=limo.GetRightWheelOdom()
            
                limo.SetMotionCommand(linear_vel=v,steering_angle=gamma)
    
                delta_theta=(dr-dl)/0.17079
                d=(dl+dr)/2
                x_prime=d*math.cos(theta)
                y_prime=d*math.sin(theta)
                theta_prime=delta_theta
                x=x+x_prime
                y=y+y_prime
                theta=theta+theta_prime 

        self.thread = threading.Thread(target=robot_thread)
        self.thread.start()


     
 


    def stop_robot(self):
        if self.thread is not None and self.thread.is_alive():
            self.robot_status = "Stopped"
            self.thread.join()  # Attendez la fin du thread
            print("Robot stopped")
        else:
            print("No running robot to stop.")
        

    def perform_action1(self):
        print("Performing action 1")
    def rvizMode(self):
        self.start_ros_nodes()

    def turtlesimMode(self):
        self.start_turtlesim_nodes()

    def turn_gauche(self):
    	self.run_custom_script()



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
		
		


    def run_custom_script(self):
        # Replace 'python custom_script.py' with the command to run your specific Python script
        try:
            subprocess.Popen(['python', 'custom_script.py'])
        except Exception as e:
            print(f"Error running custom script: {e}")


class RobotControlUI:
    def __init__(self, root, robot_controller):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry('600x400')  # Ajustez la taille de la fenêtre

        self.robot_controller = robot_controller

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

        # Image d'un robot
        #robot_image = Image.open("monrobot.jpg")  # Remplacez "robot.png" par le chemin de votre propre image
        #robot_image = robot_image.resize((100, 100), Image.ANTIALIAS)
        #robot_image = ImageTk.PhotoImage(robot_image)
        #robot_label = ttk.Label(self.root, image=robot_image, background='white')
        #robot_label.image = robot_image
        #robot_label.grid(row=0, column=0, columnspan=3, pady=10, sticky=tk.W)

        # Label
        label = ttk.Label(self.root, text="Navigation Interface", font=('Helvetica', 16, 'bold'), background='white')
        label.grid(row=1, column=0, columnspan=3, pady=10, sticky=tk.W)

        # Entry for command
        #self.command_entry = ttk.Entry(self.root)
        #self.command_entry.grid(row=2, column=0, pady=5, columnspan=2, sticky=tk.W)

        # Button to send command - Start (en vert)
        start_button = ttk.Button(self.root, text="Start", command=self.send_start_command, style='Start.TButton')
        start_button.grid(row=2, column=0, pady=10, padx=(0, 10), sticky=tk.W)

        # Button to send command - Stop (en rouge)
        stop_button = ttk.Button(self.root, text="Stop", command=self.send_stop_command, style='Stop.TButton')
        stop_button.grid(row=3, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)

        # Buttons with style and colors
        commands1 = ['rviz Mode']
        for i, command in enumerate(commands1, start=1):
            send_button1 = ttk.Button(self.root, text=command, command=self.send_command, style='TButton')
            send_button1.grid(row=i+3, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)
        commands2 = ['Turtlesimmode']
        for i, command in enumerate(commands2, start=1):
            send_button2 = ttk.Button(self.root, text=command, command=self.send_turtlesim_command, style='TButton')
            send_button2.grid(row=i+4, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)
	
        # Entry for xinit
        self.x_init_entry = ttk.Entry(self.root)
        self.x_init_entry.grid(row=2, column=5, pady=5, columnspan=2, sticky=tk.W)

        # Entry for yinit
        self.y_init_entry = ttk.Entry(self.root)
        self.y_init_entry.grid(row=3, column=5, pady=5, columnspan=2, sticky=tk.W)

        # Labels for displaying robot pose
        ttk.Label(self.root, text="Robot Position", font=('Helvetica', 14, 'bold'), background='white').grid(row=4, column=3, columnspan=3, pady=5, sticky=tk.W)
        ttk.Label(self.root, text="X:").grid(row=5, column=2, pady=5, sticky=tk.W)
        self.x_label = ttk.Label(self.root, text="", font=('Helvetica', 12), background='white')
        self.x_label.grid(row=5, column=3, pady=5, sticky=tk.W)
        ttk.Label(self.root, text="Y:").grid(row=6, column=2, pady=5, sticky=tk.W)
        self.y_label = ttk.Label(self.root, text="", font=('Helvetica', 12), background='white')
        self.y_label.grid(row=6, column=3, pady=5, sticky=tk.W)
        ttk.Label(self.root, text="Theta:").grid(row=7, column=2, pady=5, sticky=tk.W)
        self.theta_label = ttk.Label(self.root, text="", font=('Helvetica', 12), background='white')
        self.theta_label.grid(row=7, column=3, pady=5, sticky=tk.W)
	
    def send_start_command(self):
        self.execute_command('start')

    def send_stop_command(self):
        self.execute_command('stop')

    def send_command(self):
        self.robot_controller.rvizMode()
        

    def send_turtlesim_command(self):
        self.robot_controller.turtlesimMode()



    def execute_command(self, command):
        if command == 'start':
            self.robot_controller.start_robot()
        elif command == 'stop':
            self.robot_controller.stop_robot()
        elif command == 'Rviz Mode':
            self.robot_controller.rvizMode()
        else:
            messagebox.showinfo("Info", "Unknown command")

def main():
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
    root.mainloop()

if __name__ == '__main__':
    limo=limo.LIMO()
    main()

