import numpy as np
import matplotlib.pyplot as plt

# Load the CSV data as numpy arrays
data1 = np.genfromtxt("logs/0926/arlog_20230926170524.csv", delimiter=',', skip_header=1, usecols=(0, 1, 2, 3, 4, 5, 6), dtype=None)
data2 = np.genfromtxt("logs/0926/patlog_20230926170521.csv", delimiter=',', skip_header=1, usecols=(0, 1), dtype=None, converters={1: lambda x: 'S' if x.strip() == 'Spinning' else 'P'})

rostime1 = [row[3].decode("utf-8") for row in data1]  # Decode bytes to strings
rostime2 = [row[0].decode("utf-8") for row in data2]  # Decode bytes to strings

# Remove spaces from 'rostime' values
rostime1 = [rt.replace(" ", "") for rt in rostime1]
rostime2 = [rt.replace(" ", "") for rt in rostime2]

state = [row[6] for row in data1]
random_no = [row[5] for row in data1]
behave_val = [row[4] for row in data1]
behavior = [''] * len(rostime1)  # Ensure behavior list matches the length of rostime1
ID = [row[0] for row in data1]

# Update behavior based on available data
for i, row in enumerate(data2):
    if len(row) > 1:
        if i < len(behavior):
            behavior[i] = row[1]

# Determine which 'Ros Time' array to use based on the longer one
if len(rostime1) >= len(rostime2):
    rostime = rostime1[:len(rostime2)]
else:
    rostime = rostime2[:len(rostime1)]

# Convert 'rostime' to datetime objects for better x-axis representation
import datetime
rostime = [datetime.datetime.strptime(rt, "%H:%M:%S") for rt in rostime]

# Create a figure and axes
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot ID as dots with numbers next to them (ignore the axis)
ax1.plot(rostime, ID, color='black', marker='o', label='ID', zorder=5)
for i, txt in enumerate(ID):
    ax1.annotate(txt, (rostime[i], ID[i]), textcoords="offset points", xytext=(0, 5), ha='center')

# Plot State as a line graph on the right-hand side y-axis with custom labels
state_labels = ['Patrol' if s == 1 else 'Explore' for s in state]
ax2 = ax1.twinx()
ax2.plot(rostime, state, color='green', label='State')
ax2.set_yticks([1, 2])
ax2.set_yticklabels(state_labels)

# Plot Behavior on the left-hand side y-axis as dots with "S" or "P" labels
behavior_labels = ['S' if b == 'Spinning' else 'P' for b in behavior]
ax1.scatter(rostime, [0] * len(rostime), color='red', marker='s', label='Behaviour', s=30)
for i, b_label in enumerate(behavior_labels):
    ax1.annotate(b_label, (rostime[i], 0), textcoords="offset points", xytext=(0, 5), ha='center', color='red')

# Plot Random No on the left-hand side y-axis as a line graph
ax1.plot(rostime, random_no, color='blue', label='Random No')

# Plot Behave Val on the left-hand side y-axis as a line graph
ax1.plot(rostime, behave_val, color='purple', label='Behave Val')

# Set the labels for the left and right y-axes
ax1.set_ylabel('Random No / Behave Val')
ax2.set_ylabel('State (Patrol/Explore)')

# Set the title and legend
ax1.set_title('Data Over Time')
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Show the plot
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
