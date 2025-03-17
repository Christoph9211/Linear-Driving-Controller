import time
from machine import Pin, I2C
from mpu6050 import init_mpu6050, get_mpu6050_data 

# Setup I2C and initialize MPU6050
i2c = I2C(0, scl=Pin(25), sda=Pin(24), freq=400000)
init_mpu6050(i2c)

def calibrate_gyro_bias(i2c, num_samples=100):
    bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    for _ in range(num_samples):
        data = get_mpu6050_data(i2c)
        bias['x'] += data['gyro']['x']
        bias['y'] += data['gyro']['y']
        bias['z'] += data['gyro']['z']
        time.sleep_ms(10)
    
    bias['x'] /= num_samples
    bias['y'] /= num_samples
    bias['z'] /= num_samples
    
    return bias

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def update(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

# Initialize Kalman filter for yaw angle
kalman_yaw = KalmanFilter(process_variance=1e-3, measurement_variance=1e-2, estimated_measurement_variance=1e-2)

# Calibrate the gyro
gyro_bias = calibrate_gyro_bias(i2c)

yaw = 0.0
prev_time = time.ticks_ms()

while True:
    data = get_mpu6050_data(i2c)
    curr_time = time.ticks_ms()
    dt = (curr_time - prev_time) / 1000.0  # convert ms to seconds
    prev_time = curr_time
    
     # Subtract the bias from the gyroscope data
    corrected_gyro_z = data['gyro']['z'] - gyro_bias['z']

    # Integrate the corrected gyro's z-axis
    raw_yaw = yaw + corrected_gyro_z * dt

    # Filter the raw yaw using the Kalman filter
    yaw = kalman_yaw.update(raw_yaw)

    print("Filtered Yaw: {:.2f} degrees".format(yaw))
    time.sleep_ms(10)
