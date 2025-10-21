import tkinter as tk
from tkinter import filedialog
import serial
import threading
import time
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Initialize variables
serial_port = "/dev/ttyACM0"  # Update your serial port
baud_rate = 115200
is_running = False
log_file = None
start_time = None
data = {
    "time": [],
    "value1": [],
    "value2": [],
    "value3": [],
    "value4": [],
}

# Function to handle serial data acquisition
def read_serial_data():
    global is_running, log_file, start_time
    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
        while is_running:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("SENSOR0"):
                    # Parse the data
                    parts = line.split()
                    if len(parts) == 7:
                        timestamp = time.time() - start_time  # Seconds since start
                        value1 = float(parts[3])
                        value2 = float(parts[4])
                        value3 = float(parts[5][:-1])  # Remove 'V'
                        value4 = float(parts[6][:-1])  # Remove 'V'

                        # Append data
                        data["time"].append(timestamp)
                        data["value1"].append(value1)
                        data["value2"].append(value2)
                        data["value3"].append(value3)
                        data["value4"].append(value4)

                        # Log data to file if enabled
                        if log_file:
                            with open(log_file, "a") as f:
                                f.write(f"{timestamp},{value1},{value2},{value3},{value4}\n")
            except Exception as e:
                print(f"Error reading serial data: {e}")
# Start data acquisition
def start_acquisition():
    global is_running, start_time, data
    if not is_running:
        # Reset all data
        data = {
            "time": [],
            "value1": [],
            "value2": [],
            "value3": [],
            "value4": [],
        }

        # Clear all plots
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

        # Redraw empty plots
        # ax1.set_title("Value 1")
        # ax2.set_title("Value 2")
        # ax3.set_title("Value 3")
        # ax4.set_title("Value 4")

#        ax1.set_xlabel("Seconds Since Start")
#        ax2.set_xlabel("Seconds Since Start")
#        ax3.set_xlabel("Seconds Since Start")
        ax4.set_xlabel("Seconds Since Start")

        ax1.set_ylabel("Value 1")
        ax2.set_ylabel("Value 2")
        ax3.set_ylabel("Value 3")
        ax4.set_ylabel("Value 4")

        canvas.draw()

        # Start acquisition
        is_running = True
        start_time = time.time()  # Record the start time
        threading.Thread(target=read_serial_data, daemon=True).start()

# Stop data acquisition and show the additional window
def stop_acquisition():
    global is_running
    is_running = False
    show_additional_plots()

# Reset serial port
def send_reset():
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
            ser.write(b'R')
            print("Sent Shift-R to serial port")
    except Exception as e:
        print(f"Error sending reset command: {e}")

# Select log file
def select_log_file():
    global log_file
    log_file = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
    if log_file:
        file_name_label.config(text=f"Log File: {log_file}")

# Real-time plot update function
def update_plot():
    if data["time"]:
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

        ax1.plot(data["time"], data["value1"], label="Digital pressure")
        ax2.plot(data["time"], data["value2"], label="Temp")
        ax3.plot(data["time"], data["value3"], label="Diff sensor")
        ax4.plot(data["time"], data["value4"], label="Absolute sensor")

        # ax1.legend(loc="upper left")
        # ax2.legend(loc="upper left")
        # ax3.legend(loc="upper left")
        # ax4.legend(loc="upper left")

        # ax1.set_title("Value 1")
        # ax2.set_title("Value 2")
        # ax3.set_title("Value 3")
        # ax4.set_title("Value 4")

        # ax1.set_xlabel("Seconds Since Start")
        # ax2.set_xlabel("Seconds Since Start")
        # ax3.set_xlabel("Seconds Since Start")
        ax4.set_xlabel("Seconds Since Start")

        ax1.set_ylabel("Digital pressure")
        ax2.set_ylabel("Temp")
        ax3.set_ylabel("Diff sensor")
        ax4.set_ylabel("Absolute sensor")

        canvas.draw()

# Show additional plots with linear fits
def show_additional_plots():
    if len(data["value2"]) > 1:  # Ensure there is data to plot
        # Calculate linear fits
        coeffs_3_vs_2 = np.polyfit(data["value2"], data["value3"], 1)
        coeffs_4_vs_2 = np.polyfit(data["value2"], data["value4"], 1)

        # Create a new window for additional plots
        additional_window = tk.Toplevel(root)
        additional_window.title("Additional Plots")

        fig_additional = Figure(figsize=(8, 6), dpi=100)

        # Plot value3 vs value2
        ax_3_vs_2 = fig_additional.add_subplot(211)
        ax_3_vs_2.scatter(data["value2"], data["value3"], label="Value3 vs Value2", color="blue")
        ax_3_vs_2.plot(data["value2"], np.polyval(coeffs_3_vs_2, data["value2"]), color="red", label="Linear Fit")
        ax_3_vs_2.set_title("Value3 vs Value2")
        ax_3_vs_2.set_xlabel("Value2")
        ax_3_vs_2.set_ylabel("Value3")
        ax_3_vs_2.legend()

        # Plot value4 vs value2
        ax_4_vs_2 = fig_additional.add_subplot(212)
        ax_4_vs_2.scatter(data["value2"], data["value4"], label="Value4 vs Value2", color="green")
        ax_4_vs_2.plot(data["value2"], np.polyval(coeffs_4_vs_2, data["value2"]), color="red", label="Linear Fit")
        ax_4_vs_2.set_title("Value4 vs Value2")
        ax_4_vs_2.set_xlabel("Value2")
        ax_4_vs_2.set_ylabel("Value4")
        ax_4_vs_2.legend()

        canvas_additional = FigureCanvasTkAgg(fig_additional, master=additional_window)
        canvas_additional.get_tk_widget().pack()

# GUI setup
root = tk.Tk()
root.title("Serial Data Acquisition")

# Layout frames
control_frame = tk.Frame(root)
control_frame.pack(side=tk.LEFT, padx=10, pady=10)

plot_frame = tk.Frame(root)
plot_frame.pack(side=tk.RIGHT, padx=10, pady=10)

# Control buttons
start_button = tk.Button(control_frame, text="Start", command=start_acquisition)
start_button.pack(pady=5)

stop_button = tk.Button(control_frame, text="Stop", command=stop_acquisition)
stop_button.pack(pady=5)

reset_button = tk.Button(control_frame, text="Reset", command=send_reset)
reset_button.pack(pady=5)

log_button = tk.Button(control_frame, text="Select Log File", command=select_log_file)
log_button.pack(pady=5)

# Display file name
file_name_label = tk.Label(control_frame, text="Log File: None", wraplength=200)
file_name_label.pack(pady=5)

# Matplotlib setup for four subplots
fig = Figure(figsize=(10, 8), dpi=100)
ax1 = fig.add_subplot(411)
ax2 = fig.add_subplot(412)
ax3 = fig.add_subplot(413)
ax4 = fig.add_subplot(414)

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.get_tk_widget().pack()

# Update plot periodically
def periodic_update():
    update_plot()
    root.after(1000, periodic_update)

periodic_update()

# Run the GUI event loop
root.mainloop()
