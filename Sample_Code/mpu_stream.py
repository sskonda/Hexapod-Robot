import os
import time
from Libs.mpu6050.mpu6050.mpu6050 import mpu6050

sensor = mpu6050(0x68)

while True:
    accel = sensor.get_accel_data()
    gyro = sensor.get_gyro_data()
    temp = sensor.get_temp()

    os.system("clear")
    print("MPU6050 live stream\n")
    print(f"Accel (m/s^2): x={accel['x']:.3f}, y={accel['y']:.3f}, z={accel['z']:.3f}")
    print(f"Gyro  (deg/s): x={gyro['x']:.3f}, y={gyro['y']:.3f}, z={gyro['z']:.3f}")
    print(f"Temp      (C): {temp:.2f}")
    time.sleep(0.1)
