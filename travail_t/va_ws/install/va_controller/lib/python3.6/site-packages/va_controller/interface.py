import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk
import subprocess

class RobotController:
    def __init__(self):
        self.robot_status = "Stopped"

    def start_robot(self):
        self.robot_status = "Started"
        print("Robot started")

    def stop_robot(self):
        self.robot_status = "Stopped"
        print("Robot stopped")

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
        subprocess.run(['gnome-terminal', '--', 'bash', '-c', commande4], check=True)

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
        robot_image = Image.open("monrobot.jpg")  # Remplacez "robot.png" par le chemin de votre propre image
        robot_image = robot_image.resize((100, 100), Image.ANTIALIAS)
        robot_image = ImageTk.PhotoImage(robot_image)
        robot_label = ttk.Label(self.root, image=robot_image, background='white')
        robot_label.image = robot_image
        robot_label.grid(row=0, column=0, columnspan=3, pady=10, sticky=tk.W)

        # Label
        label = ttk.Label(self.root, text="Navigation Interface", font=('Helvetica', 16, 'bold'), background='white')
        label.grid(row=1, column=0, columnspan=3, pady=10, sticky=tk.W)

        # Entry for command
        self.command_entry = ttk.Entry(self.root)
        self.command_entry.grid(row=2, column=0, pady=5, columnspan=2, sticky=tk.W)

        # Button to send command - Start (en vert)
        start_button = ttk.Button(self.root, text="Start", command=self.send_start_command, style='Start.TButton')
        start_button.grid(row=2, column=2, pady=10, padx=(0, 10), sticky=tk.W)

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
    main()


