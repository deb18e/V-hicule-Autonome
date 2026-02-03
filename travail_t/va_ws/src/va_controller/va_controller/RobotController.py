from tkinter import messagebox
import subprocess
import math
from pylimo import limo
import time
import threading
from queue import Queue

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
            t0 = time.time()
            while self.robot_status != "Stopped":
                if time.time() - t0 > 3.0:
                    self.x_goal = 2.0
                    self.y_goal = 0.0

            limo.SetMotionCommand(linear_vel=0, steering_angle=0)
            time.sleep(1)

            odopresleft = limo.GetLeftWheelOdeom()
            odopresright = limo.GetRightWheelOdom()

            while self.robot_status != "Stopped":
                rho = math.sqrt((x_goal - x)**2 + (y_goal - y)**2)
                if rho < 0.05:
                    break

                alpha = math.atan2(y_goal - y, x_goal - x) - theta
                alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

                beta = theta_goal - theta - alpha
                beta = (beta + math.pi) % (2 * math.pi) - math.pi

                v = K_rho * rho
                w = K_alpha * alpha + K_beta * beta

                if abs(v) < 1e-3:
                    gamma = 0.0
                else:
                    gamma = math.atan((L * w) / v)

                gamma = max(min(gamma, 0.5), -0.5)
                v = min(v, 0.3)

                dl = (limo.GetLeftWheelOdeom() - odopresleft) / 1000.0
                dr = (limo.GetRightWheelOdom() - odopresright) / 1000.0
                odopresleft = limo.GetLeftWheelOdeom()
                odopresright = limo.GetRightWheelOdom()
                
                dt = 0.05

                x += v * math.cos(theta) * dt
                y += v * math.sin(theta) * dt
                theta += (v / L) * math.tan(gamma) * dt
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
            self.robot_status = "Stopped"

    self.thread = threading.Thread(target=robot_thread, daemon=True)
    self.thread.start()
