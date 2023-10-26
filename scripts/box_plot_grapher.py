import numpy as np
import matplotlib.pyplot as plt
from matplotlib.cbook import boxplot_stats

# Define the CSV files for each condition
conditions = [
    ['6-10', [
        'logs/6,10/t_1/20231020230808_TTH_np_6_arlog.csv',
        'logs/6,10/t_2/20231020231849_TTH_np_6_arlog.csv',
        'logs/6,10/t_3/20231020232915_TTH_6_10_arlog.csv',
        'logs/6,10/t_4/20231020235149_TTH_6_10_arlog.csv',
        'logs/6,10/t_5/20231020235747_TTH_6_10_arlog.csv'
    ]],
    ['100-200', [
        'logs/100,200/t_1/20231021001925_TTH_100_200_arlog.csv',
        'logs/100,200/t_2/20231021005245_TTH_100_200_arlog.csv',
        'logs/100,200/t_3/20231021010305_TTH_100_200_arlog.csv',
        'logs/100,200/t_4/20231021011359_TTH_100_200_arlog.csv'
    ]],
    ['NP', [
        'logs/np/t_1/20231020223249_TTH_np_5_arlog.csv',
        'logs/np/t_2/20231020223947_TTH_np_5_arlog.csv',
        'logs/np/t_3/20231020224653_TTH_np_5_arlog.csv',
        'logs/np/t_4/20231020225356_TTH_np_5_arlog.csv'
    ]],
    ['2-6', [
        'logs/idk/6,7/trial_1/20231020193608_TTH_2_6_arlog.csv',
        'logs/idk/6,7/trial_2/20231020204230_TTH_2_6_arlog.csv',
        'logs/idk/6,7/trial_3/20231020210611_TTH_2_6_arlog.csv',
        'logs/idk/20231020215734_TTH_2_6_arlog.csv'
    ]]
]

# Create a figure for box plots
fig, ax = plt.subplots(figsize=(10, 6))
fig.suptitle("Box Plots of AR Tags Detected")

box_data = []
box_labels = []

# Iterate through each condition and collect the data
for i, (condition, trial_files) in enumerate(conditions):
    cumulative_tags_all_trials = []
    
    # Initialize a variable to store the minimum length
    min_length = float('inf')

    # Iterate through the trials to find the minimum length
    for trial_file in trial_files:
        with open(trial_file, 'r') as file:
            next(file)  # Skip the header row
            min_length = min(min_length, sum(1 for _ in file))

    # Iterate through the trials again to get the data
    for trial_file in trial_files:
        time_data = []
        id_data = []

        with open(trial_file, 'r') as file:
            next(file)  # Skip the header row
            for line in file:
                values = line.strip().split(',')
                if values[0].isdigit():  # Check if the value in the first column is a digit
                    id_data.append(int(values[0]))
                else:
                    # Handle non-integer values in the first column, e.g., by skipping
                    continue
                time_data.append(float(values[1]))  # Assuming time is in the second column

        cumulative_tags = np.zeros(min_length)

        for j in range(min_length):
            cumulative_tags[j] = np.sum(np.unique(id_data[:j + 1]))

        cumulative_tags_all_trials.append(np.max(cumulative_tags))  # Use the last value as it's cumulative

    box_data.append(cumulative_tags_all_trials)
    if condition != 'NP':
        box_labels.append(f"TTH: {condition}")
    else:
        box_labels.append("No Plasticity")

savepath = 'graphs/w3_single_formal_trials/boxplotssvg.svg'

# Create box plots
ax.boxplot(box_data, labels=box_labels)
ax.set_ylabel("Total AR Tags Detected")

plt.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(savepath, bbox_inches='tight', format='svg')
plt.show()
