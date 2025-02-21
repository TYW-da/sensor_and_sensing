import can
import struct 
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
from scipy.signal import iirnotch, lfilter, butter, find_peaks
import time 
bus = can.Bus( 
    interface='slcan',
    channel='COM6',
    bitrate=1e6,
)

ser = serial.Serial('COM9', 115200, timeout=1)
def set_angle(angle):
    control_bytes = struct.pack('<i', angle*100)
    msg = can.Message(
        arbitration_id=0x141,
        data=[0xA3, 0, 0, 0, *control_bytes],
        is_extended_id=False
        )
    bus.send(msg)


# Data buffer for real-time plotting
data_buffer_size = 1000  # Number of data points to display
raw_data_buffer = deque(maxlen=data_buffer_size)
notch_filtered_data_buffer = deque(maxlen=data_buffer_size)
lowpass_data_buffer = deque(maxlen=data_buffer_size)
smoothed_data_buffer = deque(maxlen=data_buffer_size)

# Moving average window size
window_size = 10  # Adjust this value as needed
moving_window = deque(maxlen=window_size)

# Create the figure and axis for plotting
fig, ax = plt.subplots()
line_raw, = ax.plot([], [], lw=2, label="Raw Data")
line_notch_filtered, = ax.plot([], [], lw=2, label="Notch Filtered", color='green')
line_lowpass, = ax.plot([], [], lw=2, label="Low-Pass Filtered", color='yellow')
line_smoothed, = ax.plot([], [], lw=2, label="Smoothed Data", color='red')

ax.set_ylim(-10, 250)  # Adjust based on your expected data range
ax.set_xlim(0, data_buffer_size)
ax.set_title("Real-Time Serial Data Plot with Combined Notch, Low-Pass, and Smoothing")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.legend()

fs = 1000  # Sampling frequency (Hz)
last_peak_time = 0

# Design notch filters
frequencies = [50, 62, 124]
Q = 30
notch_filters = []
for f0 in frequencies:
    b_notch, a_notch = iirnotch(f0, Q, fs)
    notch_filters.append((b_notch, a_notch))

# Design low-pass filter
cutoff_freq = 400  # Cutoff frequency in Hz
nyquist = 0.5 * fs
normal_cutoff = cutoff_freq / nyquist
b_lowpass, a_lowpass = butter(4, normal_cutoff, btype='low', analog=False)

# Function to initialize the plot
def init():
    line_raw.set_data([], [])
    line_notch_filtered.set_data([], [])
    line_lowpass.set_data([], [])
    line_smoothed.set_data([], [])
    return line_raw, line_notch_filtered, line_lowpass, line_smoothed

# Function to update the plot
def update(frame):
    global last_peak_time
    try:
        while ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='replace').strip()
            
            # Check if data is valid (non-empty and numeric)
            if data:
                try:
                    value = int(data)
                    raw_data_buffer.append(value)
                    
                    # Apply the notch filters sequentially
                    filtered_value = value
                    for b_notch, a_notch in notch_filters:
                        filtered_value = lfilter(b_notch, a_notch, [filtered_value])[-1]
                    
                    # Store the combined notch-filtered value
                    notch_filtered_data_buffer.append(filtered_value)
                    
                    # Apply the low-pass filter
                    lowpass_value = lfilter(b_lowpass, a_lowpass, [filtered_value])[-1]
                    lowpass_data_buffer.append(lowpass_value)
                    
                    # Add the low-pass filtered value to the moving window
                    moving_window.append(lowpass_value)
                    
                    # Compute the moving average
                    smoothed_value = np.mean(moving_window)
                    smoothed_data_buffer.append(smoothed_value)
                except ValueError:
                    print(f"Invalid data received: {data}")
    except serial.SerialException as e:
        print(f"Serial read error: {e}")
    
    # Update the raw data plot
    line_raw.set_data(range(len(raw_data_buffer)), raw_data_buffer)
    
    # Update the combined notch-filtered data plot
    line_notch_filtered.set_data(range(len(notch_filtered_data_buffer)), notch_filtered_data_buffer)
    
    # Update the low-pass filtered data plot
    line_lowpass.set_data(range(len(lowpass_data_buffer)), lowpass_data_buffer)
    
    # Update the smoothed data plot
    line_smoothed.set_data(range(len(smoothed_data_buffer)), smoothed_data_buffer)
    
    # Perform peak detection on the smoothed data
    if len(smoothed_data_buffer) >= 10 and time.time() - last_peak_time > 0.5:  # Ensure enough data for peak detection
        recent_data = np.array(smoothed_data_buffer)[-250:]  # Extract the 250 most recent values
        peaks, _ = find_peaks(recent_data, height=40)  # Detect peaks in the recent data
        
        if len(peaks) > 0:
            last_peak_time = time.time()
            peak_heights = recent_data[peaks]
            amplitude = np.max(peak_heights)
            if amplitude < 80:
                set_angle(90)
                print('---- ACTION 1 DETECTED. Turn to 90 ----')
            else:
                set_angle(-90)
                print('---- ACTION 2 DETECTED. Turn to -90 ----')
                
    return line_raw, line_notch_filtered, line_lowpass, line_smoothed

# Set up the animation
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=2)

# Display the plot
plt.show()

# Close the serial connection when done
ser.close()
print("Serial connection closed.")

ser.close()
bus.shutdown()
print("Shut down")