# pico_car_heading_hold.py
"""
Drive forward immediately after startup and continuously trim heading using gyro feedback.
Tested on Raspberry Pi Pico W + MPU‑6050 + Pico_Car motor driver.

 * Sets initial yaw as the target heading (straight ahead)
 * Uses a proportional controller to bias left/right motor speeds
 * Keeps wheels rolling forward at BASE_SPEED while applying corrections
 * Calibrate KP, BASE_SPEED, MAX_SPEED, and MIN_CORR/MAX_CORR for your chassis
"""

from machine import Pin, I2C
import time
from mpu6050 import init_mpu6050, get_mpu6050_data
from pico_car import Pico_Car  # make sure pico_car.py exposes set_motor_speed(l, r)

# ── CONFIGURATION ────────────────────────────────────────────────────────────
I2C_ID        = 0
I2C_SDA_PIN   = 8
I2C_SCL_PIN   = 9
I2C_FREQ_HZ   = 400_000

GYRO_SAMPLES  = 200               # samples for bias calibration
BASE_SPEED    = 100               # straight‑line PWM duty (0‑255)
MAX_SPEED     = 155               # upper PWM limit (safe for your motors)
KP            = 4.0               # proportional gain – start small
MIN_CORR      = -60               # min correction PWM offset
MAX_CORR      =  60               # max correction PWM offset
YAW_DEADBAND  = 1.5               # deg – ignore tiny errors to prevent jitter

# ── HELPER FUNCTIONS ────────────────────────────────────────────────────────

def clamp(val, lo, hi):
    return max(lo, min(val, hi))


def calibrate_gyro_bias(i2c, n=GYRO_SAMPLES):
    """Average Z‑gyro for n samples to find static bias (deg/s)."""
    bias_z = 0.0
    for _ in range(n):
        bias_z += get_mpu6050_data(i2c)["gyro"]["z"]
        time.sleep_ms(5)
    return bias_z / n


# Simple integrator for yaw (degrees)

def integrate_yaw(prev_yaw, gyro_z, bias_z, dt):
    return prev_yaw + (gyro_z - bias_z) * dt


# ── INITIALISE HARDWARE ─────────────────────────────────────────────────────

i2c = I2C(I2C_ID, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ_HZ)
init_mpu6050(i2c)
print("Calibrating gyro …")
BIAS_Z = calibrate_gyro_bias(i2c)
print("Bias Z =", BIAS_Z)

car = Pico_Car()

# ── MAIN LOOP ───────────────────────────────────────────────────────────────

target_yaw = 0.0                 # current forward direction ≙ 0°
current_yaw = 0.0
last_t = time.ticks_ms()

# Begin moving *immediately* after calibration
car._set_motor_speeds(BASE_SPEED, BASE_SPEED)

while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_t) / 1000.0  # Δt in seconds
    last_t = now

    # Read sensors
    gyro_z = get_mpu6050_data(i2c)["gyro"]["z"]

    # Integrate yaw
    current_yaw = integrate_yaw(current_yaw, gyro_z, BIAS_Z, dt)

    # Compute error (positive = veer right / clockwise)
    error = current_yaw - target_yaw
    if abs(error) < YAW_DEADBAND:
        error = 0.0                   # ignore very small drift

    # Proportional steering correction (PWM offset)
    corr = clamp(KP * error, MIN_CORR, MAX_CORR)

    # Differential motor speeds – opposite sign to error keeps heading
    left_pwm  = clamp(BASE_SPEED - corr, 0, MAX_SPEED)
    right_pwm = clamp(BASE_SPEED + corr, 0, MAX_SPEED)

    car._set_motor_speeds(int(left_pwm), int(right_pwm))

    # (Optional) add small delay to reduce I2C traffic
    time.sleep_ms(10)
