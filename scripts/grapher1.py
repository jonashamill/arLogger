import matplotlib.pyplot as plt

# Data
time = [2.65, 65.4, 98.4, 98.65, 98.78, 114.9, 127.9, 135.9, 178.9, 220.4, 230.78, 262.9, 307.9]
min_velocity = [0.0500000007450581, 0.0500000007450581, 1187.84997558594, 1196.44995117188, 1194.65002441406, 617.650024414063,
                1092.84997558594, 1375.44995117188, 2901.64990234375, 4365.85009765625, 4723.0498046875, 5843.4501953125, 7404.0498046875]
max_velocity = [0.200000002980232, 0.200000002980232, 1188.0, 1196.59997558594, 1194.80004882813, 617.799987792969,
                1093.0, 1375.59997558594, 2901.80004882813, 4366.0, 4723.2001953125, 5843.60009765625, 7404.2001953125]
ids = [0, 6, 164, 5, 4, 2, 11, 12, 14, 13, 15, 8, 1]

# Create the line graph for max and min velocities
plt.plot(time, min_velocity, label='Min Velocity', color='blue', marker='o')
plt.plot(time, max_velocity, label='Max Velocity', color='red', marker='x')

# Add labeled dots/crosses for each ID
for i, id_val in enumerate(ids):
    plt.scatter(time[i], min_velocity[i], color='blue', marker='o')
    plt.scatter(time[i], max_velocity[i], color='red', marker='x')
    plt.text(time[i], min_velocity[i], f"ID: {id_val}", ha='right', va='bottom', fontsize=8)
    plt.text(time[i], max_velocity[i], f"ID: {id_val}", ha='right', va='bottom', fontsize=8)

# Set axis labels and title
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs. Time with IDs')

# Add legend
plt.legend()

# Show the plot
plt.grid(True)
plt.tight_layout()
plt.show()
