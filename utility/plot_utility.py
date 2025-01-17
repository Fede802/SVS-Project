import matplotlib.pyplot as plt
import log_utility, random

def __parse_log_file(log_file, log_dir = "logs"):
    data = open(log_dir+"/"+log_file, "r").readlines()
    velocities = []
    accelerations = []
    throttles = []
    brakes = []
    for line in data:
        values = line.split(",")
        velocities.append(float(values[0]))
        accelerations.append(float(values[1]))
        throttles.append(float(values[2]))
        brakes.append(float(values[3]))
    return velocities, accelerations, throttles, brakes

def plot_last_run():
    velocities, accelerations, throttles, brakes = __parse_log_file(log_utility.get_last_log_file())
    velocities = velocities[drop:]
    accelerations = accelerations[drop:]
    throttles = throttles[drop:]
    brakes = brakes[drop:]
    x = range(0, len(velocities))
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))
    axs[0,0].plot(x, velocities, 'r', label='Velocity')
    axs[0,0].legend()
    axs[0,0].grid(True)
    axs[0,1].plot(x, accelerations, 'r', label='Acceleration')
    axs[0,1].legend()
    axs[0,1].grid(True)
    axs[1,0].plot(x, throttles, 'r', label='Throttle', marker='o')
    axs[1,0].legend()
    axs[1,0].grid(True)
    axs[1,1].plot(x, brakes, 'r', label='Brake', marker='o')
    axs[1,1].legend()
    axs[1,1].grid(True)
    plt.tight_layout()
    plt.show() 

def custom_plot(*plot_files, names, log_dir = "logs"):
    titles = ["Velocity", "Acceleration", "Throttle", "Brake"]
    velocities = []
    accelerations = []
    throttles = []
    brakes = []

    for plot_file in plot_files:
        velocity, acceleration, throttle, brake = __parse_log_file(plot_file, log_dir)
        velocity = velocity[drop:]
        acceleration = acceleration[drop:]
        throttle = throttle[drop:]
        brake = brake[drop:]
        velocities.append(velocity)
        accelerations.append(acceleration)
        throttles.append(throttle)
        brakes.append(brake)

    size = min([len(velocity) for velocity in velocities])  
    x = range(0, size)  

    for i in range(len(velocities)):
        velocities[i] = velocities[i][:size]
        accelerations[i] = accelerations[i][:size]
        throttles[i] = throttles[i][:size]
        brakes[i] = brakes[i][:size]

    
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))
    for i in range(len(velocities)):
        random_color = (random.random(), random.random(), random.random())
        axs[0,0].plot(x, velocities[i], label=names[i], color=random_color) #, ax =
        axs[0,1].plot(x, accelerations[i], label=names[i], color=random_color)
        axs[1,0].plot(x, throttles[i], label=names[i], marker='o', color=random_color, )
        axs[1,1].plot(x, brakes[i], label=names[i], marker='o', color=random_color)

    axs[0,0].set_title(titles[0])
    axs[0,0].legend()
    axs[0,0].grid(True)
    axs[0,1].set_title(titles[1])
    axs[0,1].legend()
    axs[0,1].grid(True)
    axs[1,0].set_title(titles[2])
    axs[1,0].legend()
    axs[1,0].grid(True)
    axs[1,1].set_title(titles[3])
    axs[1,1].legend()
    axs[1,1].grid(True)

    fig.suptitle("Switching PID vs Switching RL - Complex Scenario")
    plt.tight_layout()
    plt.show()
drop = 50
if __name__ == "__main__":
    random.seed(14)
    custom_plot("log_17.txt", "log_10.txt", "log_14.txt", "log_20.txt", names=["other", "rl", "5.1", "5.2"])    