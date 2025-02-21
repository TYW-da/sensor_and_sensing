import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

# Serial port configuration
SERIAL_PORT = 'COM9'
BAUD_RATE = 115200

# Initialize the serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Failed to connect to {SERIAL_PORT}: {e}")
    exit()

# Data buffer for real-time plotting
data_buffer_size = 1000  # Number of data points to display
raw_data_buffer = deque(maxlen=data_buffer_size)
kalman_filtered_data_buffer = deque(maxlen=data_buffer_size)

# Create the figure and axis for plotting
fig, ax = plt.subplots()
line_raw, = ax.plot([], [], lw=2, label="Raw Data", color='#1f77b4')  # Blue
line_kalman, = ax.plot([], [], lw=2, label="Kalman Filtered", color='#d62728')  # Red

ax.set_ylim(-10, 800)  # Adjust based on your expected data range
ax.set_xlim(0, data_buffer_size)
ax.set_title("Real-Time Serial Data Plot with Kalman Filtering")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.legend()

# Kalman filter parameters
class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=1.0):
        self.process_noise = process_noise  # Process noise covariance
        self.measurement_noise = measurement_noise  # Measurement noise covariance
        self.estimated_value = 0  # Initial estimate
        self.error_covariance = 1.0  # Initial error covariance

    def update(self, measurement):
        # Prediction step
        predicted_value = self.estimated_value
        predicted_error_covariance = self.error_covariance + self.process_noise

        # Update step
        kalman_gain = predicted_error_covariance / (predicted_error_covariance + self.measurement_noise)
        self.estimated_value = predicted_value + kalman_gain * (measurement - predicted_value)
        self.error_covariance = (1 - kalman_gain) * predicted_error_covariance

        return self.estimated_value

# Initialize the Kalman filter
kalman_filter = KalmanFilter(process_noise=0.01, measurement_noise=5.0)

# Function to initialize the plot
def init():
    line_raw.set_data([], [])
    line_kalman.set_data([], [])
    return line_raw, line_kalman

# Function to update the plot
def update(frame):
    try:
        while ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='replace').strip()
            
            # Check if data is valid (non-empty and numeric)
            if data:
                try:
                    value = int(data)
                    raw_data_buffer.append(value)
                    
                    # Apply the Kalman filter
                    filtered_value = kalman_filter.update(value)
                    kalman_filtered_data_buffer.append(filtered_value)
                except ValueError:
                    print(f"Invalid data received: {data}")
    except serial.SerialException as e:
        print(f"Serial read error: {e}")
    
    # Update the raw data plot
    line_raw.set_data(range(len(raw_data_buffer)), raw_data_buffer)
    
    # Update the Kalman filtered data plot
    line_kalman.set_data(range(len(kalman_filtered_data_buffer)), kalman_filtered_data_buffer)
    
    return line_raw, line_kalman

# Set up the animation
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=2)

# Display the plot
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")