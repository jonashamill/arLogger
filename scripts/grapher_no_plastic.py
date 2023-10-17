import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from matplotlib.widgets import CheckButtons

def convert_time(rostime):
    cleaned_time = [t.replace(' ', '').strip() for t in rostime]
    return [datetime.strptime(t, '%H:%M:%S') for t in cleaned_time]

# Load data from the first CSV
arlog = 'logs/20231013212935_idlog.csv'
data1 = np.genfromtxt(arlog, delimiter=',', skip_header=1, dtype=str)

rostime1, ID1, state1, behave_val1, random_no1 = data1[:, 3], data1[:, 0], data1[:, 6], data1[:, 4], data1[:, 5]

rostime1 = convert_time(rostime1)

# Convert to numeric types
state1 = state1.astype(int)
behave_val1 = behave_val1.astype(float)
random_no1 = random_no1.astype(float)

# Create figure and axis
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

plt.title('ID by QTY', pad=30)

# Plot data
line1, = ax1.plot(rostime1, random_no1, label='Probability Value')
line2, = ax1.plot(rostime1, behave_val1, label='Activity Level')

# Plot ID against y-axis
scatter1 = ax1.scatter(rostime1, ID1.astype(int), c='black', label='ID')

# State plotting
state_map = {0: 'Neophobic', 1: 'Neophilic'}
state_labels = [state_map[s] for s in state1]
line3, = ax2.step(rostime1, state1, where='post', linestyle='-', color='blue', label='State')

# Annotations for ID
id_annotations = []
for i, txt in enumerate(ID1):
    annot = ax1.annotate(txt, (rostime1[i], int(ID1[i])), textcoords="offset points", xytext=(0,10), ha='center')
    id_annotations.append(annot)

# Create checkboxes to toggle lines
rax = plt.axes([0.8, 0.5, 0.1, 0.2])
check = CheckButtons(rax, ('Probability Value', 'Activity Level', 'ID', 'Activity Mode'), (True, True, True, True))

def toggle(label):
    if label == 'Probability Value':
        line1.set_visible(not line1.get_visible())
    elif label == 'Activity Level':
        line2.set_visible(not line2.get_visible())
    elif label == 'ID':
        scatter1.set_visible(not scatter1.get_visible())
        for annot in id_annotations:
            annot.set_visible(not annot.get_visible())
    elif label == 'Activity Mode':
        line3.set_visible(not line3.get_visible())
    plt.draw()

check.on_clicked(toggle)

# Labels and title
ax1.set_xlabel('ROS Time')
ax1.set_ylabel('ID')
ax2.set_ylabel('Activity Mode')
ax2.set_yticks([0, 1])
ax2.set_yticklabels(['Neophobic', 'Neophilic'])

# Move legend above the graph
ax1.legend(loc='upper center', bbox_to_anchor=(0.2, 1.15), ncol=2)
ax2.legend(loc='upper center', bbox_to_anchor=(0.8, 1.25), ncol=2)

plt.show()
