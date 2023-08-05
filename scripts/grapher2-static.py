import pandas as pd
import matplotlib.pyplot as plt


def main(file_path, label_prefix):

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

    # Add legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # Call the main function for each CSV file
    main('../logs/arlog_20230801201926.csv', 'Static')
    main('../logs/arlog_20230801202825.csv', 'Plastic')
