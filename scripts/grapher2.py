import pandas as pd
import matplotlib.pyplot as plt


def main():
    # Read the data from the CSV file
    file = 'arlog_20230801201926'
    data = pd.read_csv('../logs/'+ file +'.csv')

    # Extracting data columns from the DataFrame
    time = data['Time']
    min_velocity = data['Min Velocity']
    max_velocity = data['Max Velocity']
    ids = data['ID']

    # Create the line graph for max and min velocities
    plt.plot(time, min_velocity, label='Min Velocity', color='blue', marker='o')
    plt.plot(time, max_velocity, label='Max Velocity', color='red', marker='x')

    # Add labeled dots/crosses for each ID
    for i, id_val in enumerate(ids):
        plt.scatter(time[i], min_velocity[i], color='blue', marker='o')
        plt.scatter(time[i], max_velocity[i], color='red', marker='x')
        plt.text(time[i], min_velocity[i], 'ID: {}'.format(id_val), ha='right', va='bottom', fontsize=8)
        plt.text(time[i], max_velocity[i], 'ID: {}'.format(id_val), ha='right', va='bottom', fontsize=8)

    # Set axis labels and title
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('With Static Behaviour')

    # Add legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.tight_layout()

    # Save the plot as a PNG file
    plt.savefig('../graphs/'+ file +'.png', dpi=300)  # dpi parameter controls the resolution of the image

    # Show the plot
    plt.show()


if __name__ == '__main__':
    main()
