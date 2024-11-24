import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import threading
import time

class RealTimePlotApp:
    def __init__(self, title: str):
        self.x_data = []
        self.y_data = []
        self.title = title
        threading.Thread(target=self.run_gui, daemon=True).start()
        
    def __update_plot(self):
        self.ax.clear()
        self.ax.set_title(self.title)
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Value")
        self.ax.plot(self.x_data, self.y_data, marker='o', linestyle='-', color='b')
        self.canvas.draw()

    def add_value(self, y):
        self.x_data.append(len(self.x_data))
        self.y_data.append(y)
        self.root.after(0, self.__update_plot)

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Real-Time Plot")
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack()
        self.__update_plot()
        self.root.mainloop()
