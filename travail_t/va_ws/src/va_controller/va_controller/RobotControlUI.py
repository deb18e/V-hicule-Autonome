import tkinter as tk
from tkinter import ttk, messagebox

try:
    from PIL import Image, ImageTk
    PIL_OK = True
except Exception:
    PIL_OK = False


class RobotControlUI:
    def __init__(self, root, robot_controller):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry('600x400')

        self.robot_controller = robot_controller

        # variables d’affichage pose
        self.x_var = tk.StringVar(value="0.00")
        self.y_var = tk.StringVar(value="0.00")
        self.theta_var = tk.StringVar(value="0.00")

        # champs goal
        self.goal_x_var = tk.StringVar(value="1.0")
        self.goal_y_var = tk.StringVar(value="0.0")
        self.goal_theta_var = tk.StringVar(value="0.0")

        # background (optionnel)
        self.background_image = None
        if PIL_OK:
            try:
                bg = Image.open("Interface.jpeg")  # doit être dans le même dossier que le script
                bg = bg.resize((600, 400))
                self.background_image = ImageTk.PhotoImage(bg)
            except Exception as e:
                print("WARNING: cannot load Interface.jpeg:", e)
                self.background_image = None
        else:
            print("WARNING: Pillow not available, no background image")

        self.create_widgets()

    def create_widgets(self):
        # background (si dispo)
        if self.background_image is not None:
            background_label = tk.Label(self.root, image=self.background_image)
            background_label.place(relwidth=1, relheight=1)

        # styles
        style = ttk.Style()
        style.configure('TButton', font=('Helvetica', 12), padding=5)

        # titre
        title = ttk.Label(
            self.root,
            text="Navigation Interface",
            font=('Helvetica', 16, 'bold'),
            background='white'
        )
        title.grid(row=0, column=0, columnspan=3, pady=10, sticky=tk.W, padx=10)

        # Start / Stop
        start_button = ttk.Button(
            self.root, text="Start", command=self.send_start_command, style='Start.TButton'
        )
        start_button.grid(row=1, column=0, pady=5, padx=10, sticky=tk.W)

        stop_button = ttk.Button(
            self.root, text="Stop", command=self.send_stop_command, style='Stop.TButton'
        )
        stop_button.grid(row=2, column=0, pady=5, padx=10, sticky=tk.W)

        # RViz / Turtlesim / RQt
        rviz_btn = ttk.Button(self.root, text="Lidar with RViz", command=self.send_rviz_command)
        rviz_btn.grid(row=3, column=0, pady=5, padx=10, sticky=tk.W)

        turtlesim_btn = ttk.Button(self.root, text="ROS app (Turtlesim)", command=self.send_turtlesim_command)
        turtlesim_btn.grid(row=4, column=0, pady=5, padx=10, sticky=tk.W)

        rqt_btn = ttk.Button(self.root, text="RQt Graph", command=self.send_rqtgraph_command)
        rqt_btn.grid(row=5, column=0, pady=5, padx=10, sticky=tk.W)

        # =========================
        # Affichage position (x,y,theta)
        # =========================
        ttk.Label(self.root, text="Position:", font=('Helvetica', 12, 'bold'), background='white')\
            .grid(row=1, column=1, sticky=tk.W, padx=10)

        ttk.Label(self.root, text="X:", font=('Helvetica', 12), background='white')\
            .grid(row=2, column=1, sticky=tk.W, padx=10)
        ttk.Label(self.root, textvariable=self.x_var, font=('Helvetica', 12), background='white')\
            .grid(row=2, column=2, sticky=tk.W)

        ttk.Label(self.root, text="Y:", font=('Helvetica', 12), background='white')\
            .grid(row=3, column=1, sticky=tk.W, padx=10)
        ttk.Label(self.root, textvariable=self.y_var, font=('Helvetica', 12), background='white')\
            .grid(row=3, column=2, sticky=tk.W)

        ttk.Label(self.root, text="Theta:", font=('Helvetica', 12), background='white')\
            .grid(row=4, column=1, sticky=tk.W, padx=10)
        ttk.Label(self.root, textvariable=self.theta_var, font=('Helvetica', 12), background='white')\
            .grid(row=4, column=2, sticky=tk.W)

        # =========================
        # Objectif (x_goal, y_goal, theta_goal)
        # =========================
        ttk.Label(self.root, text="Goal:", font=('Helvetica', 12, 'bold'), background='white')\
            .grid(row=6, column=0, sticky=tk.W, padx=10, pady=(15, 0))

        ttk.Label(self.root, text="X goal:", background='white')\
            .grid(row=7, column=0, sticky=tk.W, padx=10)
        ttk.Entry(self.root, textvariable=self.goal_x_var, width=10)\
            .grid(row=7, column=1, sticky=tk.W)

        ttk.Label(self.root, text="Y goal:", background='white')\
            .grid(row=8, column=0, sticky=tk.W, padx=10)
        ttk.Entry(self.root, textvariable=self.goal_y_var, width=10)\
            .grid(row=8, column=1, sticky=tk.W)

        ttk.Label(self.root, text="Theta goal:", background='white')\
            .grid(row=9, column=0, sticky=tk.W, padx=10)
        ttk.Entry(self.root, textvariable=self.goal_theta_var, width=10)\
            .grid(row=9, column=1, sticky=tk.W)

        set_goal_btn = ttk.Button(self.root, text="Set Goal", command=self.set_goal_from_ui)
        set_goal_btn.grid(row=9, column=2, sticky=tk.W, padx=10)

        # petit hint
        ttk.Label(
            self.root,
            text="(Astuce) Lance ce programme avec: python3 main_gui.py (pas ros2 run)",
            background='white'
        ).grid(row=10, column=0, columnspan=3, sticky=tk.W, padx=10, pady=(10, 0))


