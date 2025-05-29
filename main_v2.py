import time
from machine import Pin, I2C  #, PWM
from mpu6050 import init_mpu6050, get_mpu6050_data
# import rp2                                            # needed for ws2812 PIO
from pico_car import Pico_Car                         # put your Pico_Car class in pico_car.py

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

# print("Calibrating gyro bias… keep robot still")
# gyro_bias = calibrate_gyro_bias()
# print("Gyro bias =", gyro_bias)

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

car = Pico_Car()

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def drive(left_spd, right_spd):
    left  = int(clamp(left_spd,  0, 255) * 257)
    right = int(clamp(right_spd, 0, 255) * 257)
    for m in car._motors[::2]:
        m.duty_u16(left)
    for m in car._motors[1::2]:
        m.duty_u16(right)

def stop():
    drive(0, 0)

### ── CALIBRATION ───────────────────────────────────────────────────────────
print("Calibrating gyro… keep robot still")
stop()                                    # <-- ensure motors off
gyro_bias = calibrate_gyro_bias()
print("Bias:", gyro_bias, "  done.")

### ── CONTROL CONSTANTS ─────────────────────────────────────────────────────
BASE_SPEED = 100
KP         = 5.0        # tune to taste

### ── MAIN LOOP ─────────────────────────────────────────────────────────────
yaw = 0.0
prev_ms = time.ticks_ms()
target_yaw = 0.0  # Initialize to 0 instead of None
drive_enabled = True  # Enable driving right after calibration

while True:
    try:
        raw = get_mpu6050_data(i2c)
        now = time.ticks_ms()
        dt = time.ticks_diff(now, prev_ms) / 1000.0
        prev_ms = now

        # integrate gyro Z (bias-corrected)
        yaw += (raw['gyro']['z'] - gyro_bias['z']) * dt
        filt_yaw = kf_yaw.update(yaw)

        if drive_enabled:
            # Calculate error (difference from target yaw)
            error = filt_yaw - target_yaw
            correction = KP * error
            
            # Only apply correction when error exceeds a small threshold
            if abs(error) > 1.0:  # 1 degree deadband
                # Apply correction to motor speeds
                left_cmd = BASE_SPEED - correction  # Note: sign might need to be flipped
                right_cmd = BASE_SPEED + correction
            else:
                # When on target, both motors at full speed
                left_cmd = BASE_SPEED
                right_cmd = BASE_SPEED
            
            # Apply commands
            drive(left_cmd, right_cmd)
            
            # Debug prints
            print(f"Yaw: {filt_yaw:.2f}°, Error: {error:.2f}°, L: {left_cmd:.0f}, R: {right_cmd:.0f}")
        time.sleep_ms(5)
        
    except Exception as e:
        print(f"Error: {e}")
        stop()
        break