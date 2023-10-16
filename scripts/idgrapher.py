import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

from datetime import datetime
from scipy.fft import fft

def convert_time(rostime):
    cleaned_time = [t.replace(' ', '').strip() for t in rostime]
    return [datetime.strptime(t, '%H:%M:%S') for t in cleaned_time]

# Load data
# Replace these with paths to your own CSV files
data1 = np.genfromtxt('logs/20231013212935_arlog.csv', delimiter=',', skip_header=1, dtype=str)

rostime1, ID1 = data1[:, 3], data1[:, 0].astype(int)

rostime1 = convert_time(rostime1)

# FFT Parameters
window_size = 256  # Size of window for FFT
step_size = 128  # Step size for sliding window
n = len(ID1)

# Create figure and axis
fig, ax = plt.subplots()

plt.title('ID Spectrogram', pad=30)

# Initialize empty arrays for storing FFT results and corresponding times
fft_results = []
times = []

# Calculate FFT for each window
for start in range(0, n - window_size, step_size):
    end = start + window_size
    y = fft(ID1[start:end])
    
    fft_results.append(np.abs(y)[:window_size // 2])  # Store only positive frequencies
    times.append(rostime1[start + window_size // 2])  # Store the midpoint time for this window

# Convert lists to NumPy arrays for plotting
fft_results = np.array(fft_results).T
times = np.array(times)

# Plot the spectrogram
ax.imshow(fft_results, aspect='auto', cmap='inferno', origin='lower',
          extent=[mdates.date2num(times[0]), mdates.date2num(times[-1]), 0, window_size // 2])
ax.xaxis_date()

# Labels and title
ax.set_xlabel('ROS Time')
ax.set_ylabel('Frequency')
plt.colorbar(ax=ax, orientation='vertical', label='Amplitude')

plt.show()
