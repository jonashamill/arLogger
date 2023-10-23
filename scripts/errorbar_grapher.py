import numpy as np
import matplotlib.pyplot as plt

activity_mode_mapping = {0: 'patrol - neophobic', 1: 'explore - neophilic'}

def map_activity_mode(mode):
    return activity_mode_mapping.get(mode, mode)

trial_1_tth_6_10 = 'logs/6,10/t_1/20231020230808_TTH_np_6_metricslog.csv'
trial_2_tth_6_10 = 'logs/6,10/t_2/20231020231849_TTH_np_6_metricslog.csv'
trial_3_tth_6_10 = 'logs/6,10/t_3/20231020232915_TTH_6_10_metricslog.csv'
trial_4_tth_6_10 = 'logs/6,10/t_4/20231020235149_TTH_6_10_metricslog.csv'
trial_5_tth_6_10 = 'logs/6,10/t_5/20231020235747_TTH_6_10_metricslog.csv'

trial_1_tth_100_200 = 'logs/100,200/t_1/20231021001925_TTH_100_200_metricslog.csv'
trial_2_tth_100_200 = 'logs/100,200/t_2/20231021005245_TTH_100_200_metricslog.csv'
trial_3_tth_100_200 = 'logs/100,200/t_3/20231021010305_TTH_100_200_metricslog.csv'
trial_4_tth_100_200 = 'logs/100,200/t_4/20231021011359_TTH_100_200_metricslog.csv'

# trial_1_np = 'logs/np/t_1/20231020223249_TTH_np_5_metricslog.csv'
# trial_2_np = 'logs/np/t_2/20231020223947_TTH_np_5_metricslog.csv'
# trial_3_np = 'logs/np/t_3/20231020224653_TTH_np_5_metricslog.csv'
# trial_4_np = 'logs/np/t_4/20231020225356_TTH_np_5_metricslog.csv'

trial_1_tth_2_6 = 'logs/idk/6,7/trial_1/20231020193608_TTH_2_6_metricslog.csv'
trial_2_tth_2_6 = 'logs/idk/6,7/trial_2/20231020204230_TTH_2_6_metricslog.csv'
trial_3_tth_2_6 = 'logs/idk/6,7/trial_3/20231020210611_TTH_2_6_metricslog.csv'
trial_4_tth_2_6 = 'logs/idk/20231020215734_TTH_2_6_metricslog.csv'


trials_by_condition = {
    "TTH 6/10": [trial_1_tth_6_10, trial_2_tth_6_10, trial_3_tth_6_10, trial_4_tth_6_10, trial_5_tth_6_10],
    "TTH 100/200": [trial_1_tth_100_200, trial_2_tth_100_200, trial_3_tth_100_200, trial_4_tth_100_200],
    "TTH 2/6": [trial_1_tth_2_6, trial_2_tth_2_6, trial_3_tth_2_6, trial_4_tth_2_6],
}

total_conditions = len(trials_by_condition)
total_parameters = 3  # Activity Mode, Probability Value, Activity Level

fig = plt.figure(figsize=(15, 15))

plot_count = 1

for idx, (condition, file_paths) in enumerate(trials_by_condition.items()):
    min_length = min([len(np.genfromtxt(file_path, delimiter=',', skip_header=1)) for file_path in file_paths])
    common_time_grid = np.genfromtxt(file_paths[0], delimiter=',', skip_header=1)[:min_length, 0]
    activity_modes, probability_values, activity_levels = [], [], []

    for file_path in file_paths:
        data = np.genfromtxt(file_path, delimiter=',', skip_header=1)[:min_length]
        time = data[:, 0]
        activity_mode = data[:, 1].astype(int)
        probability_value = data[:, 2]
        activity_level = data[:, 3]

        activity_modes.append(activity_mode)
        probability_values.append(probability_value)
        activity_levels.append(activity_level)

    stacked_activity_modes = np.vstack(activity_modes)
    stacked_probability_values = np.vstack(probability_values)
    stacked_activity_levels = np.vstack(activity_levels)

    mean_activity_modes = np.mean(stacked_activity_modes, axis=0)
    std_activity_modes = np.std(stacked_activity_modes, axis=0)

    mean_probability_values = np.mean(stacked_probability_values, axis=0)
    std_probability_values = np.std(stacked_probability_values, axis=0)

    mean_activity_levels = np.mean(stacked_activity_levels, axis=0)
    std_activity_levels = np.std(stacked_activity_levels, axis=0)

    ax1 = fig.add_subplot(total_parameters, total_conditions, plot_count)
    ax2 = fig.add_subplot(total_parameters, total_conditions, plot_count + total_conditions)
    ax3 = fig.add_subplot(total_parameters, total_conditions, plot_count + 2 * total_conditions)

    ax1.errorbar(common_time_grid, mean_activity_modes, yerr=std_activity_modes, fmt='-o', color='tab:green')
    ax1.set_yticks([0, 1])
    ax1.set_yticklabels(['Patrol - (neophobic)', 'Explore - (neophilic)'])
    ax1.set_ylabel('Activity Mode', color='black')

    ax2.errorbar(common_time_grid, mean_probability_values, yerr=std_probability_values, fmt='-o', color='tab:red')
    ax2.set_ylabel('Probability Value', color='black')

    ax3.errorbar(common_time_grid, mean_activity_levels, yerr=std_activity_levels, fmt='-o', color='tab:blue')
    ax3.set_ylabel('Activity Level', color='black')
    ax3.set_xlabel('Time (seconds)')

    if plot_count <= total_conditions:
        ax1.set_title(condition)
    plot_count += 1

plt.tight_layout()
plt.show()
