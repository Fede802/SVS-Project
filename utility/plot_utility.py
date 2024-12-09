import matplotlib.pyplot as plt

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

velocities1, accelerations1, throttles1, brakes1 = __parse_log_file("log_1.txt")
# velocities2, accelerations2, throttles2, brakes2 = __parse_log_file("log_2.txt")
# velocities3, accelerations3, throttles3, brakes3 = __parse_log_file("log_3.txt")

size = len(velocities1)#min(len(velocities1), len(velocities2), len(velocities3))
velocities1 = velocities1[:size]
accelerations1 = accelerations1[:size]
throttles1= throttles1[:size]
brakes1 = brakes1[:size]
# velocities2 = velocities2[:size]
# accelerations2 = accelerations2[:size]
# throttles2 = throttles2[:size]
# brakes2 = brakes2[:size]
# velocities3 = velocities3[:size]
# accelerations3 = accelerations3[:size]
# throttles3 = throttles3[:size]
# brakes3 = brakes3[:size]


x = range(0, size)

fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs[0,0].plot(x, velocities1, 'r', label='log_1')
# axs[0,0].plot(x, velocities2, 'b', label='log_2')
# axs[0,0].plot(x, velocities3, 'g', label='log_3')
axs[0,0].legend()
axs[0,0].grid(True)

axs[0,1].plot(x, accelerations1, 'r', label='log_1')
# axs[0,1].plot(x, accelerations2, 'b', label='log_2')
# axs[0,1].plot(x, accelerations3, 'g', label='log_3')
axs[0,1].legend()
axs[0,1].grid(True)

axs[1,0].plot(x, throttles1, 'r', label='log_1', marker='o')
# axs[1,0].plot(x, throttles2, 'b', label='log_2', marker='o')
# axs[1,0].plot(x, throttles3, 'g', label='log_3', marker='o')
axs[1,0].legend()
axs[1,0].grid(True)

axs[1,1].plot(x, brakes1, 'r', label='log_1', marker='o')
# axs[1,1].plot(x, brakes2, 'b', label='log_2', marker='o')
# axs[1,1].plot(x, brakes3, 'g', label='log_3', marker='o')
axs[1,1].legend()
axs[1,1].grid(True)

plt.tight_layout()
plt.show()

# plt.plot(x, y, color='r')  # Red line for sin(x)
# plt.plot(x, y, color='b')  # Blue line for cos(x)

# # Add labels and title
# plt.xlabel('x values')
# plt.ylabel('y values')
# plt.title('Plot of logs')

# # Display the legend
# plt.legend()

# plt.grid(True)

# plt.show()