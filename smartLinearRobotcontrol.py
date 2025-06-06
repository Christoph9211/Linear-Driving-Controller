import time
from machine import Pin, I2C, PWM
from mpu6050 import init_mpu6050, get_mpu6050_data

R_B = PWM(Pin(11))
R_A = PWM(Pin(10))
L_B = PWM(Pin(13))
L_A = PWM(Pin(12))

### ── MPU-6050 SETUP ─────────────────────────────────────────────────────────
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=400_000)
init_mpu6050(i2c)

def calibrate_gyro_bias(num_samples=100):
    """Return average gyro bias (deg/s) from N samples."""
    bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    for _ in range(num_samples):
        d = get_mpu6050_data(i2c)['gyro']
        bias['x'] += d['x']; bias['y'] += d['y']; bias['z'] += d['z']
        time.sleep_ms(10)
    for k in bias:  # average
        bias[k] /= num_samples
    return bias

print("Calibrating gyro bias… keep robot still")
gyro_bias = calibrate_gyro_bias()
print("Gyro bias =", gyro_bias)

### ── KALMAN FILTER ──────────────────────────────────────────────────────────
class Kalman1D:
    def __init__(self, q=1e-3, r=1e-2):
        self.q = q                      # process noise covariance
        self.r = r                      # measurement noise covariance
        self.x̂ = 0.0                   # state estimate
        self.p = 1.0                    # estimate error covariance

    def update(self, z):
        # prediction
        p_ = self.p + self.q
        x̂_ = self.x̂
        # update
        k = p_ / (p_ + self.r)          # Kalman gain
        self.x̂ = x̂_ + k * (z - x̂_)
        self.p = (1 - k) * p_
        return self.x̂

kf_yaw = Kalman1D()

### ── GLOBAL VARIABLES ───────────────────────────────────────────────────────
yaw = 0.0
_prev_time = time.ticks_ms()

### ── MOTOR CONTROL ───────────────────────────────────────────────────────────
class _motor_control:
    def __init__(self):
        R_B.freq(1000)
        R_A.freq(1000)
        L_B.freq(1000)
        L_A.freq(1000)
    def Car_Forward(self, speed1, speed2):
        speed1 = speed1*257
        speed2 = speed2*257
        R_B.duty_u16(0)
        R_A.duty_u16(speed2)
        L_B.duty_u16(speed1)
        L_A.duty_u16(0)
    def Car_Stop(self):
        R_B.duty_u16(0)
        R_A.duty_u16(0)
        L_B.duty_u16(0)
        L_A.duty_u16(0)
    def Car_Back(self, speed1, speed2):
        speed1 = speed1*257
        speed2 = speed2*257
        R_B.duty_u16(speed2)
        R_A.duty_u16(0)
        L_B.duty_u16(0)
        L_A.duty_u16(speed1)
    def Car_Left(self, speed1, speed2):
        speed1 = speed1*257
        speed2 = speed2*257
        R_B.duty_u16(0)
        R_A.duty_u16(speed2)
        L_B.duty_u16(0)
        L_A.duty_u16(speed1) 
    def Car_Right(self, speed1, speed2):
        speed1 = speed1*257
        speed2 = speed2*257
        R_B.duty_u16(speed2)
        R_A.duty_u16(0)
        L_B.duty_u16(speed1)
        L_A.duty_u16(0)



def mpu6050_get_yaw():
    global yaw
    global _prev_time
    _prev_time = time.ticks_ms()
    while True:
        data = get_mpu6050_data(i2c)
        curr_time = time.ticks_ms()
        dt = (curr_time - _prev_time) / 1000.0  # convert ms to seconds
        _prev_time = curr_time

        # Integrate the gyro's z-axis (assuming data['gyro']['z'] is in degrees per second)
        raw_yaw = yaw + data['gyro']['z'] * dt

        # Filter the raw yaw using the Kalman filter
        yaw = kf_yaw.update(raw_yaw)

        print("Filtered Yaw: {:.2f} degrees".format(yaw))
        time.sleep_ms(10)

def drive_straight(target_speed, target_yaw=0, kp=10.0, max_correction=50):
    """
    Drive the robot straight while compensating for yaw drift.

    Args:
        target_speed (int): Base motor speed (0-255)
        target_yaw (float, optional): Target yaw angle in degrees. If None, uses current yaw as target.
        kp (float): Proportional gain for yaw correction. Higher values make the correction more aggressive.
        max_correction (int): Maximum allowed correction to apply to motor speeds
    """
    global yaw
    filt_yaw = 0
    yaw_error = 0
    
    try:
        # Get current yaw from MPU6050
        current_yaw = mpu6050_get_yaw()
        
        # Apply Kalman filter
        filt_yaw = kf_yaw.update(current_yaw)
        
        # Calculate yaw error (difference from target)
        yaw_error = target_yaw - filt_yaw
        
        # Calculate correction (proportional control)
        correction = kp * yaw_error
        
        # Limit correction to max_correction
        correction = max(-max_correction, min(max_correction, correction))
        
        # Apply correction to motor speeds (opposite for each side)
        left_speed = target_speed - correction
        right_speed = target_speed + correction

        # Ensure speeds are within valid range (0-255)
        left_speed = max(0, min(155, left_speed))
        right_speed = max(0, min(155, right_speed))

        # Drive motors using the motor instance
        motor.Car_Forward(int(left_speed), int(right_speed))

    except Exception as e:
        print(f"Error in drive_straight: {e}")
        motor.Car_Stop()

    # Return current yaw and error for monitoring
    return filt_yaw, yaw_error

# Example usage:
while True:
    # Create an instance of _motor_control
    motor = _motor_control()
#     motor.Car_Forward(155,155)
    drive_straight(100)
        
