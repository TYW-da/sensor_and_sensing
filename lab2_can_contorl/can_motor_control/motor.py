import serial
from time import sleep
from includes import Gyems, CanBus

def pd_control(q, dq, q_des, dq_des, kp, kd):
    
    error_q = q_des - q
    error_dq = dq_des - dq
    u = kp * error_q + kd * error_dq
    
    return u

ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
bus = CanBus()
motor = Gyems(bus)
float_data = 0
data = 0
k = 1
mi = 1/100

try:
    motor.enable()
    motor.set_zero()
    #motor.set_speed()
    while True:
        if (ser.in_waiting > 7): data = ser.readline().decode('utf-8').strip()
        if data: float_data = float(data)
        info = motor.info()
        acc = pd_control(info['angle'], info['speed'], float_data, 0, 7, 5)
        motor.set_speed(acc)
        print(info['angle'])

except KeyboardInterrupt:
    pass

finally:
    motor.disable(True)
    bus.close()