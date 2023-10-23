import numpy as np
import matplotlib.pyplot as plt

# Define a mapping for activity modes
activity_mode_mapping = {0: 'patrol - neophobic', 1: 'explore - neophilic'}

# Replace activity modes with their corresponding labels
def map_activity_mode(mode):
    return activity_mode_mapping.get(mode, mode)

# Read the CSV file and extract data
file_path = 'logs/6,10/t_5/20231020235747_TTH_6_10_metricslog.csv'  # Replace with your file path

data = np.genfromtxt(file_path, delimiter=',', skip_header=1)

time = data[:, 0]
activity_mode = data[:, 1].astype(int)
probability_value = data[:, 2]
activity_level = data[:, 3]

# Create a figure with two y-axes
fig, ax1 = plt.subplots()

# Plot probability value and activity level on the left axis
ax1.plot(time, probability_value, color='tab:red', label='Probability Value')
ax1.plot(time, activity_level, color='tab:blue', label='Activity Level')
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Probability/Level', color='black')
ax1.tick_params(axis='y', labelcolor='black')
ax1.legend(loc='upper left')

# Create a second y-axis for activity mode
ax2 = ax1.twinx()
activity_mode_labels = [map_activity_mode(mode) for mode in activity_mode]
ax2.plot(time, activity_mode, color='tab:green', label='Activity Mode')
ax2.set_yticks([0, 1])
ax2.set_yticklabels(['Patrol - (neophobic)', 'Explore - (neophilic)'])
ax2.set_ylabel('Activity Mode', color='black')
ax2.tick_params(axis='y', labelcolor='black')
ax2.legend(loc='upper right')

# Display the plot
plt.title('Single Trial TTH 6,10')
plt.show()
