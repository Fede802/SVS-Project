from matplotlib.animation import FuncAnimation
from bokeh.plotting import figure, output_file, show
import matplotlib.pyplot as plt
import numpy as np
import random
import time
import pygal
import random
import time
import threading

class Plot:

    def __init__(self):
        self.x = [0] 
        self.y = [10]
        self.chart = self.create_chart(self.x, self.y)
        
        

    def create_chart(self, x_data, y_data):
        chart = pygal.Line()
        chart.title = 'Real-Time Data Plot'
        chart.x_labels = map(str, x_data)  # Convert x_data to string for labeling
        chart.add('Value', y_data)
        chart.render_in_browser()

        # Save the chart as an SVG file
        chart.render_to_file('live_data_chart.svg')

        return chart
        
        
    
    

    def add_value(self, y):
        self.x.append(len(self.x))
        self.y.append(y)
        self.create_chart(self.x, self.y)
        print( 'UPDATE')
        
        




# Initialize data
x_data = []
y_data = []

# Create a Pygal line chart
def create_chart(x_data, y_data):
    chart = pygal.Line()
    chart.title = 'Real-Time Data Plot'
    chart.x_labels = map(str, x_data)  # Convert x_data to string for labeling
    chart.add('Value', y_data)
    chart.render_in_browser()

    # Save the chart as an SVG file
    chart.render_to_file('live_data_chart.svg')

    return chart

# Function to simulate real-time data generation
def update_chart():
    global x_data, y_data
    while True:
        # Simulate new data points
        new_x = len(x_data) + 1
        new_y = random.uniform(0, 10)

        # Append new data to the list
        x_data.append(new_x)
        y_data.append(new_y)

        # Create and display the updated chart
        create_chart(x_data, y_data)

        # Simulate a delay between incoming data
        time.sleep(1)

# Start the data update in a separate thread
def start_real_time_plotting():
    data_thread = threading.Thread(target=update_chart, daemon=True)
    data_thread.start()

# Run the plotting function
start_real_time_plotting()

# Keep the notebook running to simulate continuous updates (if using Jupyter notebook)
while True:
    time.sleep(1)
