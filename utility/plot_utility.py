import matplotlib.pyplot as plt

def __parse_log_file(log_file, log_dir = "logs"):
    data = open(log_dir+"/"+log_file, "r").readlines()
    velocities = []
    accelerations = []
    for line in data:
        values = line.split(",")
        velocities.append(float(values[0]))
        accelerations.append(float(values[1]))
    return velocities, accelerations    

velocities1, accelerations1 = __parse_log_file("log_1.txt")
velocities2, accelerations2 = __parse_log_file("log_3.txt")

size = min(len(velocities1), len(velocities2))
velocities1 = velocities1[:size]
accelerations1 = accelerations1[:size]
velocities2 = velocities2[:size]
accelerations2 = accelerations2[:size]

x = range(0, size)

fig, axs = plt.subplots(1, 2, figsize=(10, 8))
axs[0].plot(x, velocities1, 'r')
axs[0].plot(x, velocities2, 'b')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(x, accelerations1, 'r')
axs[1].plot(x, accelerations2, 'b')
axs[1].legend()
axs[1].grid(True)

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