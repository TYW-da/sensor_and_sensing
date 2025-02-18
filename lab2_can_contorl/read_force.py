import serial
float_data = 0
ser = serial.Serial('/dev/ttyACM1', 9600)
data = 0
while True:
    if (ser.in_waiting > 7): data = ser.readline().decode('utf-8').strip()
    if data: float_data = float(data)
    print(float_data)