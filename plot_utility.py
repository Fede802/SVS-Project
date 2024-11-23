import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import threading
import time

class RealTimePlotApp:
    def __init__(self):
        # Initialize data lists
        self.x_data = []
        self.y_data = []

        # Create a figure and axes for the plot
        self.figure, self.ax = plt.subplots()
        self.ax.set_title("Real-Time Data Plot")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Value")

        # Start the Tkinter GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        # Start the data generation in the main thread
        self.running = True
        #self.start_data_generation()

    def update_plot(self):
        """Update the plot with new data."""
        if len(self.x_data) > 0 and len(self.y_data) > 0:
            # Clear the plot, redraw the data, and set labels
            self.ax.clear()
            self.ax.plot(self.x_data, self.y_data, marker='o', linestyle='-', color='b')
            self.ax.set_title("Real-Time Data Plot")
            self.ax.set_xlabel("Time")
            self.ax.set_ylabel("Value")

            # Redraw the canvas with the updated plot
            self.canvas.draw()

    def add_value(self, y):
        self.x_data.append(len(self.x_data))
        self.y_data.append(y)
        self.root.after(0, self.update_plot)
        print( 'UPDATE')

    def start_data_generation(self):
        """Start generating new data points every second."""
        def generate_data():
            while self.running:
                new_x = len(self.x_data) + 1
                new_y = random.uniform(0, 10)

                self.x_data.append(new_x)
                self.y_data.append(new_y)

                # Schedule the GUI update
                self.root.after(0, self.update_plot)

                time.sleep(1)  # Simulate new data every second

        # Start the data generation in the main thread
        threading.Thread(target=generate_data, daemon=True).start()

    def run_gui(self):
        """Run the Tkinter main loop in a separate thread."""
        self.root = tk.Tk()
        self.root.title("Real-Time Plot with Tkinter and Matplotlib")

        # Create a canvas to embed the plot in the Tkinter window
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack()

        # Start the Tkinter main loop in the GUI thread
        self.root.mainloop()

    def stop(self):
        """Stop the data generation thread."""
        self.running = False

# Create the application instance and run it
#app = RealTimePlotApp()
