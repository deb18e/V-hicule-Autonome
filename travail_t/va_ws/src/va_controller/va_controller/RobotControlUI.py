import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk


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
        background_image = Image.open("Interface.jpeg")  # Remplacez "background_image.jpg" par le chemin de votre propre image
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



        # Buttons with style and colors
        commands1 = ['Lidar with Rviz']
        for i, command in enumerate(commands1, start=1):
            send_button1 = ttk.Button(self.root, text=command, command=self.send_command, style='TButton')
            send_button1.grid(row=i+6, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)
        commands2 = ['Ros application (with turtlesim)']
        for i, command in enumerate(commands2, start=1):
            send_button2 = ttk.Button(self.root, text=command, command=self.send_turtlesim_command, style='TButton')
            send_button2.grid(row=i+7, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)

        commands3 = ['Rqt graph']
        for i, command in enumerate(commands3, start=1):
            send_button3 = ttk.Button(self.root, text=command, command=self.send_rqtgraph_command, style='TButton')
            send_button3.grid(row=i+8, column=0, pady=5, columnspan=3, padx=(0, 10), sticky=tk.W)	


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

    def send_command(self):
        self.robot_controller.rvizMode()
        

    def send_turtlesim_command(self):
        self.robot_controller.turtlesimMode()


    def send_rqtgraph_command(self):
        self.robot_controller.rqtGraph()


    def execute_command(self, command):
        if command == 'start':
            self.robot_controller.start_robot()
        elif command == 'stop':
            self.robot_controller.stop_robot()
        elif command == 'Rviz Mode':
            self.robot_controller.rvizMode()
        elif command == 'Rqt Graph':
            self.robot_controller.rqtGraph()
        elif command == 'Ros application (with turtlesim)':
            self.robot_controller.turtlesimMode()
        else:
            messagebox.showinfo("Info", "Unknown command")