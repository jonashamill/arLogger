import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import simps

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

# Create a figure with subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Number of AR Tags Detected Across All Trials")

integral_values = []  # Store the integral values

# Iterate through each condition and plot it on a subplot
for i, (condition, trial_files) in enumerate(conditions):
    ax = axs[i // 2, i % 2]
    cumulative_tags_all_trials = []

    # Initialize a variable to store the minimum length
    min_length = float('inf')

    # Iterate through the trials
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

        time_data = np.array(time_data)
        id_data = np.array(id_data)

        min_length = min(min_length, len(time_data))

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

        time_data = np.array(time_data)
        id_data = np.array(id_data)

        time_data = time_data[:min_length]
        id_data = id_data[:min_length]

        cumulative_tags = np.zeros(min_length)

        for j in range(min_length):
            cumulative_tags[j] = np.sum(np.unique(id_data[:j + 1]))

        cumulative_tags_all_trials.append(cumulative_tags)

    cumulative_tags_all_trials = np.array(cumulative_tags_all_trials)

    mean_tags = np.mean(cumulative_tags_all_trials, axis=0)
    std_tags = np.std(cumulative_tags_all_trials, axis=0)

    ax.errorbar(time_data - time_data[0], mean_tags, yerr=std_tags, fmt='-o')
    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("AR tags Detected")
    if condition != 'NP':
        ax.set_title(f"TTH: {condition}")
    else:
        ax.set_title(f"No Plasticity (Constant Patrol)")
    ax.grid(True)

    # Calculate the derivative of the curve
    derivative = np.diff(mean_tags) / np.diff(time_data)
    time_derivative = time_data[:-1]

    ax2 = ax.twinx()
    ax2.plot(time_derivative - time_derivative[0], derivative, color='red', linestyle='dashed')
    ax2.set_ylabel("Derivative", color='red')
    ax2.tick_params(axis='y', labelcolor='red')

    # Calculate the integral of the curve (area under the curve)
    integral = simps(mean_tags, time_data - time_data[0])

    # Store the integral values
    integral_values.append(integral)

    # Replace the derivative label with the integral value
    ax2.text(0.5, -0.35, f"Integral: {integral:.2f}", transform=ax.transAxes, fontsize=10, ha='center')

plt.tight_layout(rect=[0, 0, 1, 0.96])

# Create a bar chart for integral values only
labels = [f'No Plasticity' if i == 2 else f'TTH {conditions[i][0]}' for i in range(len(conditions))]
values_integral = integral_values

# Create a new figure for the bar chart
plt.figure(figsize=(10, 6))
plt.bar(labels, values_integral, width=0.2, align='edge', label='Integral')
plt.ylabel("Integral")
plt.legend(loc='upper left')
plt.title('Integrals of the Respective Curves')  # Add the title

plt.tight_layout()

plt.show()

# Create a new figure for the box plots
fig, ax = plt.subplots(figsize=(10, 6))

# Prepare data for box plots
box_data = []
box_labels = []

for i, (condition, _) in enumerate(conditions):
    cumulative_tags_all_trials = np.array(cumulative_tags_all_trials)
    box_data.append(np.max(cumulative_tags_all_trials, axis=1))  # Use the last value as it's cumulative
    if condition != 'NP':
        box_labels.append(f"TTH: {condition}")
    else:
        box_labels.append("No Plasticity")

# Create box plots
ax.boxplot(box_data, labels=box_labels)

ax.set_ylabel("Total AR Tags Detected")
ax.set_title("Box Plots of Total AR Tags Detected for Each Condition")

plt.tight_layout()
plt.show()
