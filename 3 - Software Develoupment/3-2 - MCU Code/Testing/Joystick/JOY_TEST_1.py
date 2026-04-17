import tkinter as tk
from tkinter import ttk, scrolledtext
import pygame
import serial

# --- CONFIGURATION ---
SERIAL_PORT = 'COM9'  # Double check this in Device Manager!
BAUD_RATE = 115200

class JoystickSerialGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Joystick to KL25Z Bridge")
        self.root.geometry("450x700")

        # Initialize Serial with a check
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.status_text = f"Connected to {SERIAL_PORT}"
            self.status_color = "green"
        except Exception as e:
            self.ser = None
            self.status_text = f"Error: Could not open {SERIAL_PORT}"
            self.status_color = "red"

        # Initialize Joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()
        else:
            self.joy = None
            self.status_text = "No Joystick Found!"
            self.status_color = "red"

        self.setup_ui()
        self.update_loop()

    def setup_ui(self):
        # Connection Status
        self.status_lbl = tk.Label(self.root, text=self.status_text, fg=self.status_color, font=('Arial', 10, 'bold'))
        self.status_lbl.pack(pady=5)

        # Analog Gauges
        axis_frame = ttk.LabelFrame(self.root, text=" Analog Data ")
        axis_frame.pack(padx=10, pady=5, fill="x")

        self.axis_vars = []
        self.axis_labels = []
        names = ["X-Axis", "Y-Axis", "Twist", "Throttle"]
        
        for i in range(4):
            ttk.Label(axis_frame, text=names[i]).grid(row=i, column=0, padx=5, sticky="w")
            var = tk.DoubleVar()
            bar = ttk.Progressbar(axis_frame, variable=var, maximum=1.0, length=200)
            bar.grid(row=i, column=1, padx=5, pady=5)
            lbl = ttk.Label(axis_frame, text="0.00")
            lbl.grid(row=i, column=2, padx=5)
            self.axis_vars.append(var)
            self.axis_labels.append(lbl)

        # Buttons
        btn_frame = ttk.LabelFrame(self.root, text=" Buttons ")
        btn_frame.pack(padx=10, pady=5, fill="x")
        self.btn_indicators = []
        for i in range(12):
            lbl = tk.Label(btn_frame, text=f"B{i+1}", width=5, bg="gray", relief="raised")
            lbl.grid(row=i//4, column=i%4, padx=4, pady=4)
            self.btn_indicators.append(lbl)

        # Terminal Output for KL25Z Echo
        term_frame = ttk.LabelFrame(self.root, text=" Echo from KL25Z ")
        term_frame.pack(padx=10, pady=10, fill="both", expand=True)
        self.terminal = scrolledtext.ScrolledText(term_frame, height=8, state='disabled', font=('Consolas', 9))
        self.terminal.pack(fill="both", expand=True)

    def log_to_terminal(self, msg):
        self.terminal.config(state='normal')
        self.terminal.insert(tk.END, msg + "\n")
        self.terminal.see(tk.END)
        self.terminal.config(state='disabled')
        # Keep only last 50 lines
        if int(self.terminal.index('end-1c').split('.')[0]) > 50:
             self.terminal.config(state='normal')
             self.terminal.delete('1.0', '2.0')
             self.terminal.config(state='disabled')

    def update_loop(self):
        if self.joy:
            pygame.event.pump()
            
            # Read Values
            x = round(self.joy.get_axis(0), 2)
            y = round(self.joy.get_axis(1), 2)
            z = round(self.joy.get_axis(2), 2)
            thr = round(self.joy.get_axis(3), 2)
            btn1 = self.joy.get_button(0)

            # Update GUI Elements
            vals = [x, y, z, thr]
            for i in range(4):
                self.axis_vars[i].set((vals[i] + 1) / 2)
                self.axis_labels[i].config(text=f"{vals[i]:.2f}")
            
            for i in range(12):
                color = "#00FF00" if self.joy.get_button(i) else "gray"
                self.btn_indicators[i].config(bg=color)

            # --- SERIAL TRANSMISSION ---
            if self.ser and self.ser.is_open:
                try:
                    packet = f"{x},{y},{z},{thr},{btn1}\n"
                    self.ser.write(packet.encode())

                    # Check for Echo from Board (FIXED NULL CHECK HERE)
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if response:
                            self.log_to_terminal(response)
                except Exception as e:
                    self.status_lbl.config(text="Connection Lost", fg="red")
                    self.ser = None

        self.root.after(20, self.update_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = JoystickSerialGui(root)
    root.mainloop()