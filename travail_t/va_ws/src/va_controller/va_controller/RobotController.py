import math
import time
import threading
from queue import Queue
from tkinter import messagebox

from pylimo import limo


class RobotController:

    def __init__(self):
        # état robot
        self.robot_status = "Stopped"
        self.thread = None
        self.queue = Queue()

        # état estimé
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

        # objectif
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # robot
        self.limo = limo.LIMO()

    # =========================
    # Définir un objectif
    # =========================
    def set_goal(self, x, y, theta=0.0):
        self.x_goal = float(x)
        self.y_goal = float(y)
        self.theta_goal = float(theta)
        print(f"New goal set: x={x}, y={y}, theta={theta}")

    # =========================
    # Démarrage du robot
    # =========================
    def start_robot(self):

        if self.thread is not None and self.thread.is_alive():
            print("Robot already running.")
            return

        self.robot_status = "Started"
        print("Robot started")

        def robot_thread():
            try:
                # paramètres robot (Ackermann)
                L = 0.2        # empattement
                dt = 0.05

                # gains
                K_rho = 0.6
                K_alpha = 1.5
                K_beta = -0.6

                # état initial (local)
                x = self.x_g
                y = self.y_g
                theta = self.theta_g

                self.limo.SetMotionCommand(0.0, 0.0)
                time.sleep(1.0)

                while self.robot_status == "Started":

                    # erreur position
                    dx = self.x_goal - x
                    dy = self.y_goal - y
                    rho = math.hypot(dx, dy)

                    if rho < 0.05:
                        break

                    # angles
                    alpha = math.atan2(dy, dx) - theta
                    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

                    beta = self.theta_goal - theta - alpha
                    beta = (beta + math.pi) % (2 * math.pi) - math.pi

                    # commandes
                    v = K_rho * rho
                    w = K_alpha * alpha + K_beta * beta

                    # Ackermann
                    if abs(v) < 1e-3:
                        gamma = 0.0
                    else:
                        gamma = math.atan((L * w) / v)

                    # saturations
                    v = min(v, 0.3)
                    gamma = max(min(gamma, 0.5), -0.5)

                    # intégration cinématique Ackermann
                    x += v * math.cos(theta) * dt
                    y += v * math.sin(theta) * dt
                    theta += (v / L) * math.tan(gamma) * dt
                    theta = (theta + math.pi) % (2 * math.pi) - math.pi

                    # envoyer commande
                    self.limo.SetMotionCommand(v, gamma)

                    # publier pour l'interface
                    self.x_g = x
                    self.y_g = y
                    self.theta_g = theta

                    # debug clair
                    print(f"x={x:.3f} y={y:.3f} theta={theta:.3f} | v={v:.2f} gamma={gamma:.2f}")

                    time.sleep(dt)

                # arrêt
                self.limo.SetMotionCommand(0.0, 0.0)
                self.robot_status = "Stopped"
                print("Robot stopped / target reached")

            except Exception as e:
                print("ERROR in robot thread:", e)
                self.queue.put("Error")
                self.robot_status = "Stopped"

        # lancement du thread
        self.thread = threading.Thread(target=robot_thread, daemon=True)
        self.thread.start()

    # =========================
    # Arrêt du robot
    # =========================
    def stop_robot(self):
        if self.thread is not None and self.thread.is_alive():
            self.robot_status = "Stopped"
            self.thread.join(timeout=1.0)
            self.limo.SetMotionCommand(0.0, 0.0)
            print("Robot stopped by user")

    # =========================
    # ROS / outils (inchangés)
    # =========================
    def rvizMode(self):
        self.start_ros_nodes()

    def turtlesimMode(self):
        self.start_turtlesim_nodes()

    def rqtGraph(self):
        self.start_rqtGraph()

    def start_ros_nodes(self):
        import subprocess
        cmd = "cd ~/agx_ws && source install/setup.bash && ros2 launch limo_bringup limo_start.launch.py"
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd])

    def start_turtlesim_nodes(self):
        import subprocess
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', "ros2 run turtlesim turtlesim_node"])

    def start_rqtGraph(self):
        import subprocess
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', "rqt_graph"])

    # =========================
    # Gestion erreurs
    # =========================
    def check_errors(self):
        while not self.queue.empty():
            err = self.queue.get()
            if err == "Error":
                messagebox.showerror(
                    "Robot Error",
                    "Communication error with LIMO.\nCheck USB / power / permissions."
                )
