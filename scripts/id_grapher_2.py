import numpy as np
import matplotlib.pyplot as plt

# Load the CSV file
csv_path = "logs/idk/6,7/trial_1/20231020193608_TTH_2_6_arlog.csv"


# Initialize empty lists to store time and ID data
time_data = []
id_data = []

# Read the CSV file and extract data
with open(csv_path, 'r') as file:
    next(file)  # Skip the header line
    for line in file:
        values = line.strip().split(',')
        time_data.append(float(values[1]))  # Assuming time is in the second column
        id_data.append(int(values[0]))  # Assuming ID is in the first column

# Convert lists to NumPy arrays
time_data = np.array(time_data)
id_data = np.array(id_data)

# Create an array of unique time points
unique_times = np.unique(time_data)

# Initialize an array to store the cumulative number of AR tags
cumulative_tags = np.zeros(len(unique_times))

# Calculate the cumulative number of AR tags
for i, time in enumerate(unique_times):
    # Count the unique IDs up to the current time point
    cumulative_tags[i] = np.sum(np.unique(id_data[time_data <= time]))

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(unique_times - unique_times[0], cumulative_tags)  # Start time at zero
plt.xlabel("Time (seconds)")
plt.ylabel("AR tags Detecteed")
plt.title("TTH 2_6 Single Trial Number of AR tags Detecteed Over Time")
plt.grid(True)

# Show the plot
plt.show()
