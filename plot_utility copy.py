from matplotlib.animation import FuncAnimation
from bokeh.plotting import figure, output_file, show
import matplotlib.pyplot as plt
import numpy as np
import random
import time

class Plot:

    def __init__(self):
        self.x = [0] 
        self.y = [10]
        self.source = ColumnDataSource(data=dict(x=self.x, y=self.y))
        
# Create the figure
        self.p = figure(title="Live Data Plot", x_axis_label='Time', y_axis_label='Value')
        self.p.line('x', 'y', source=self.source, line_width=2, color="red", alpha=0.6)


        curdoc().add_root(column(p))
        curdoc().title = "Live Data Plot"

        
        
    
    

    def add_value(self, y):
        self.x.append(len(self.x))
        self.y.append(y)
        source.stream(dict(x=self.x, y=self.y), rollover=100)
        print( 'UPDATE')
        
        

import numpy as np

from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, Slider, TextInput
from bokeh.plotting import figure

# Set up data
N = 200
x = np.linspace(0, 4*np.pi, N)
y = np.sin(x)
source = ColumnDataSource(data=dict(x=x, y=y))


# Set up plot
plot = figure(height=400, width=400, title="my sine wave",
              tools="crosshair,pan,reset,save,wheel_zoom",
              x_range=[0, 4*np.pi], y_range=[-2.5, 2.5])

plot.line('x', 'y', source=source, line_width=3, line_alpha=0.6)


# Set up widgets
text = TextInput(title="title", value='my sine wave')
offset = Slider(title="offset", value=0.0, start=-5.0, end=5.0, step=0.1)
amplitude = Slider(title="amplitude", value=1.0, start=-5.0, end=5.0, step=0.1)
phase = Slider(title="phase", value=0.0, start=0.0, end=2*np.pi)
freq = Slider(title="frequency", value=1.0, start=0.1, end=5.1, step=0.1)


# Set up callbacks
def update_title(attrname, old, new):
    plot.title.text = text.value

text.on_change('value', update_title)

def update_data(attrname, old, new):

    # Get the current slider values
    a = amplitude.value
    b = offset.value
    w = phase.value
    k = freq.value

    # Generate the new curve
    x = np.linspace(0, 4*np.pi, N)
    y = a*np.sin(k*x + w) + b

    source.data = dict(x=x, y=y)

for w in [offset, amplitude, phase, freq]:
    w.on_change('value', update_data)


# Set up layouts and add to document
inputs = column(text, offset, amplitude, phase, freq)

curdoc().add_root(row(inputs, plot, width=800))
curdoc().title = "Sliders"
show(plot)        
        


 
  

import random
from bokeh.plotting import figure, curdoc, output_notebook, show
from bokeh.models import ColumnDataSource
from bokeh.layouts import column
import time
from threading import Thread

# Create a ColumnDataSource to hold the plot data
source = ColumnDataSource(data=dict(x=[], y=[]))

# Create the figure
p = figure(title="Live Data Plot", x_axis_label='Time', y_axis_label='Value')
p.line('x', 'y', source=source, line_width=2, color="red", alpha=0.6)

# Define a function to update the data
def update():
    # Simulate real-time data (can be replaced with real sensor/API data)
    new_x = source.data['x'][-1] + 1 if len(source.data['x']) > 0 else 0
    new_y = random.uniform(0, 10)
    
    # Add the new data to the ColumnDataSource
    source.stream(dict(x=[new_x], y=[new_y]), rollover=100)

    # The plot will be updated automatically in the Bokeh server

# Start a thread to simulate real-time data updates
def update_data():
    while True:
        update()
        time.sleep(1)  # Simulate data arrival every second

# Start the data update in a separate thread
thread = Thread(target=update_data, daemon=True)
thread.start()

# Set up the Bokeh server to display the plot
curdoc().add_root(column(p))
curdoc().title = "Live Data Plot"
