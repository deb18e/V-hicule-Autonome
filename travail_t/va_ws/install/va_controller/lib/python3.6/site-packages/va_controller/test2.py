import tkinter as tk

class CommandInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Command Interface")

        self.x_value = tk.StringVar()
        self.create_widgets()

    def create_widgets(self):
        label = tk.Label(self.root, text="Enter X:")
        label.pack(pady=10)

        entry = tk.Entry(self.root, textvariable=self.x_value)
        entry.pack(pady=10)

        button = tk.Button(self.root, text="Send Command", command=self.show_x_value)
        button.pack(pady=20)

        result_label = tk.Label(self.root, text="Result:")
        result_label.pack()

    def show_x_value(self):
        x = self.x_value.get()
        try:
            x_float = float(x)
            result_label = tk.Label(self.root, text=f"Result: {x_float}")
            result_label.pack()
        except ValueError:
            result_label = tk.Label(self.root, text="Invalid input. Please enter a numerical value.")
            result_label.pack()

def main():
    root = tk.Tk()
    command_interface = CommandInterface(root)
    root.mainloop()

if __name__ == "__main__":
    main()

