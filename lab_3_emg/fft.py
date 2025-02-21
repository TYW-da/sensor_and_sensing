import serial
import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

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

# Collect raw data
data_length = 5000 # Number of data points to collect
raw_data = []
print("Collecting data...")
while len(raw_data) < data_length:
    try:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8', errors='replace').strip()
            if data:
                try:
                    value = float(data)
                    raw_data.append(value)
                except ValueError:
                    print(f"Invalid data received: {data}")
    except serial.SerialException as e:
        print(f"Serial read error: {e}")
ser.close()
print("Data collection complete.")

# Sampling frequency (adjust based on your actual sampling rate)
fs = 1000 # Sampling frequency in Hz

# Perform FFT
N = len(raw_data)  # Number of data points
yf = fft(raw_data)  # Compute FFT
xf = fftfreq(N, 1/fs)[:N//2]  # Frequency axis

# Take the absolute value of the FFT and normalize it
amplitude = 2.0 / N * np.abs(yf[:N//2])

# Plot the frequency spectrum
plt.figure(figsize=(10, 6))
plt.plot(xf, amplitude)

# Add custom ticks and gridlines
plt.xticks(np.arange(0, 501, 50))  # Major ticks every 50 Hz
plt.minorticks_on()  # Enable minor ticks
plt.gca().xaxis.set_ticks(np.arange(0, 501, 5), minor=True)  # Minor ticks every 5 Hz

# Style the gridlines
plt.grid(which='major', alpha=0.7, linestyle='-')  # Solid lines for major ticks
plt.grid(which='minor', alpha=0.3, linestyle=':')  # Dotted lines for minor ticks

plt.title("Frequency Spectrum of Raw Data")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")

plt.show()