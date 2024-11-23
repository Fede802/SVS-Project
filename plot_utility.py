from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import random
import time

class Plot:

    def __init__(self):
        self.x = np.linspace(0, 10*np.pi, 100) 
        self.y = np.sin(self.x)
        plt.ion() 
        self.fig = plt.figure() 
        self.ax = self.fig.add_subplot() 
        self.line1, = self.ax.plot(self.x, self.y, 'b-')
    # updates the data and graph
    
    

    def add_value(self, y):
        np.append(self.x, 1)
    
        print(self.x)
        print(self.y)
        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.y) 
        self.fig.canvas.draw() 
        self.fig.canvas.flush_events()      
        
        #plt.ylim(0,10)

        


def pltsin(ax, colors=['b']):
    x = np.linspace(0,1,100)
    if ax.lines:
        for line in ax.lines:
            line.set_xdata(x)
            y = np.random.random(size=(100,1))
            line.set_ydata(y)
    else:
        for color in colors:
            y = np.random.random(size=(100,1))
            ax.plot(x, y, color)
    fig.canvas.draw()

fig,ax = plt.subplots(1,1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_xlim(0,1)
ax.set_ylim(0,1)
plt.show()

# run this cell to dynamically update plot
for f in range(5):
    pltsin(ax, ['b', 'r'])
    time.sleep(1)



    