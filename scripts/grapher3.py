import pandas as pd
import matplotlib.pyplot as plt


def main(file, type):
    # Read the data from the CSV file
    # file = 'arlog_20230801202825'
    data = pd.read_csv('../logs/'+ file +'.csv')

    # Extracting data columns from the DataFrame
    time = data['Time']
    min_velocity = data['Min Velocity']
    max_velocity = data['Max Velocity']
    ids = data['ID']

    # Convert velocities to percentage of max value
    max_value = max(max_velocity)
    min_velocity_percent = [(v / max_value) * 100 for v in min_velocity]
    max_velocity_percent = [(v / max_value) * 100 for v in max_velocity]

    # Create the figure and axes objects
    fig, ax1 = plt.subplots()

    # Create the line graph for max and min velocities on the left y-axis
    ax1.plot(time, min_velocity_percent, label='Min Velocity', color='blue', marker='o')
    ax1.plot(time, max_velocity_percent, label='Max Velocity', color='red', marker='x')

    # Add labeled dots/crosses for each ID on the left y-axis
    for i, id_val in enumerate(ids):
        if i == 0:  # Display ID label only once
            ax1.text(time[i], min_velocity_percent[i], 'ID: {}'.format(id_val), ha='right', va='bottom', fontsize=8)
            ax1.text(time[i], max_velocity_percent[i], 'ID: {}'.format(id_val), ha='right', va='bottom', fontsize=8)
        else:
            ax1.scatter(time[i], min_velocity_percent[i], color='blue', marker='o')
            ax1.scatter(time[i], max_velocity_percent[i], color='red', marker='x')

    # Set axis labels and title for left y-axis
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (% of Max)')
    ax1.set_title('With '+ type +' Behaviour (w/ anomalies)')

    # Add legend for left y-axis at the right side (about midway up)
    ax1.legend(loc='center left', bbox_to_anchor=(1.2, 0.5))

    # Create a second y-axis on the right-hand side for displaying the IDs
    ax2 = ax1.twinx()

    # Set ticks and labels for the right y-axis (IDs)
    ax2.set_yticks(ids)
    ax2.set_yticklabels(['ID: {}'.format(int(id_val)) for id_val in ids], fontsize=8)

    # Plot the ID points as crosses on the right y-axis
    for i, id_val in enumerate(ids):
        ax2.scatter(time[i], id_val, color='black', marker='x')

    # Set labels for the left side of the y-axis
    ax1.yaxis.set_label_position("left")
    ax1.set_ylabel("Velocity (% of Max)", fontsize=10)

    # Set x-axis limit to include the maximum value of time
    ax1.set_xlim(left=min(time), right=max(time))

    # Show the plot
    plt.grid(True)
    plt.tight_layout()

    # Save the plot as a PNG file
    plt.savefig('../graphs/'+ file +'_anomG3.png', dpi=300)  # dpi parameter controls the resolution of the image

    # Show the plot
    plt.show()


if __name__ == '__main__':
    main('arlog_20230801202825', 'Plastic')
    main('arlog_20230801201926', 'Static')
