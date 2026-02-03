import tkinter as tk
from tkinter import ttk
import subprocess

class InterfaceSwitcherApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Interface Switcher")

        # Create two buttons
        button1 = ttk.Button(self.root, text="Show Interface 1", command=self.run_interface1_script)
        button1.pack(pady=20)

        button2 = ttk.Button(self.root, text="Show Interface 2", command=self.run_interface2_script)
        button2.pack(pady=20)

    def run_interface1_script(self):
        # Run the Python script for Interface 1
        subprocess.run(['python', 'vehiculeautonome.py'])

    def run_interface2_script(self):
        # Run the Python script for Interface 2
        subprocess.run(['python', 'interface2_script.py'])

if __name__ == "__main__":
    root = tk.Tk()
    app = InterfaceSwitcherApp(root)
    root.mainloop()
