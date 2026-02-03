from tkinter import messagebox
import subprocess
import math
from pylimo import limo
import time
import threading
from queue import Queue

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
                K_rho = 0.5
                K_alpha = 1.5
                K_beta = -0.6

                # position initiale
                x = -1   
                y = -1
                theta = 0

                # objectif
                x_goal = 0
                y_goal = 0
                theta_goal = 0

                limo.SetMotionCommand(linear_vel=0, steering_angle=0)
                time.sleep(1)

                odopresleft = limo.GetLeftWheelOdeom()
                odopresright = limo.GetRightWheelOdom()

                while self.robot_status != "Stopped":
                    # Distance vers l'objectif
                    rho = math.sqrt((x_goal - x)**2 + (y_goal - y)**2)
                    if rho < 0.05:
                        break  # proche du but

                    alpha = math.atan2(y_goal - y, x_goal - x) - theta
                    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
                    beta = -theta - alpha
                    beta = (beta + math.pi) % (2 * math.pi) - math.pi

                    v = K_rho * rho
                    w = K_alpha * alpha + K_beta * beta

                    if abs(v) < 1e-3:
                        gamma = 0.0
                    else:
                        gamma = math.atan((L * w) / v)

                    # Saturations
                    gamma = max(min(gamma, 0.5), -0.5)
                    v = min(v, 0.3)

                    # Odométrie
                    dl = (limo.GetLeftWheelOdeom() - odopresleft) / 1000.0
                    dr = (limo.GetRightWheelOdom() - odopresright) / 1000.0
                    odopresleft = limo.GetLeftWheelOdeom()
                    odopresright = limo.GetRightWheelOdom()

                    delta_theta = (dr - dl) / 0.16879
                    d = (dl + dr) / 2

                    x += d * math.cos(theta)
                    y += d * math.sin(theta)
                    theta += delta_theta
                    theta = (theta + math.pi) % (2 * math.pi) - math.pi

                    limo.SetMotionCommand(linear_vel=v, steering_angle=gamma)

                    self.x_g = x
                    self.y_g = y
                    self.theta_g = theta

                    time.sleep(0.05)

                limo.SetMotionCommand(linear_vel=0, steering_angle=0)
                self.robot_status = "Stopped"
                print("Robot reached target!")

            except Exception as e:
                print(f"Error in robot_thread: {e}")
                self.queue.put("Error")

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
