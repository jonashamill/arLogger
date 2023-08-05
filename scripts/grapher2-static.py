import pandas as pd
import matplotlib.pyplot as plt


def plot_csv_data(file_path, label_prefix, max_y_value):
    # Read data from the CSV file
    data = pd.read_csv(file_path)
    time = data['Time']
    min_velocity = data['Min Velocity']
    max_velocity = data['Max Velocity']
    ids = data['ID']

    # Create the line graph for max and min velocities
    plt.plot(time, min_velocity, label=f'{label_prefix} - Min Velocity', color='blue', marker='o', linestyle='-')
    plt.plot(time, max_velocity, label=f'{label_prefix} - Max Velocity', color='red', marker='x', linestyle='-')

    # Add labeled dots/crosses for each ID
    for i, id_val in enumerate(ids):
        plt.scatter(time[i], min_velocity[i], color='blue', marker='o')
        plt.scatter(time[i], max_velocity[i], color='red', marker='x')
        plt.text(time[i], min_velocity[i], f'{label_prefix} - ID: {id_val}', ha='right', va='bottom', fontsize=8)
        plt.text(time[i], max_velocity[i], f'{label_prefix} - ID: {id_val}', ha='right', va='bottom', fontsize=8)

    # Set axis labels and title
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title(f'With {label_prefix} Behaviour')

    # Set the y-axis limit to the common max_y_value
    plt.ylim(0, max_y_value)

    # Add legend
    plt.legend()


def main(file_path1, label_prefix1, file_path2, label_prefix2):
    # Read data from the first CSV file
    data1 = pd.read_csv(file_path1)
    max_y_value1 = max(data1['Max Velocity'].max(), data1['Min Velocity'].max())

    # Read data from the second CSV file
    data2 = pd.read_csv(file_path2)
    max_y_value2 = max(data2['Max Velocity'].max(), data2['Min Velocity'].max())

    # Get the maximum value from both datasets
    max_y_value = max(max_y_value1, max_y_value2)

    # Plot the first CSV data
    plot_csv_data(file_path1, label_prefix1, max_y_value)

    # Plot the second CSV data
    plot_csv_data(file_path2, label_prefix2, max_y_value)

    # Show both plots together
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main('../logs/arlog_20230801201926.csv', 'Static', '../logs/arlog_20230801202825.csv', 'Plastic')
