# pico_car_heading_hold.py (v3 – always‑forward differential)
"""
Keeps both wheels driving forward **at all times** while trimming heading.

Fixes previous issue where a wheel could reverse (negative PWM) when yaw error was
large. Now we limit corrections so each wheel stays ≥ `MIN_FWD`, guaranteeing
continuous forward motion.

• **MIN_FWD** – the slowest forward PWM that still turns the motor.  
• **BASE_SPEED** – cruise midpoint.  
• **MAX_PWM** – safety ceiling.

If you wired a motor backwards, correct it in your motor driver (or swap wires),
not by sending negative PWM.
"""

from machine import Pin, I2C
import time
from mpu6050 import init_mpu6050, get_mpu6050_data
from pico_car import Pico_Car  # exposes set_motor_speed(left_pwm, right_pwm)

# ─── CONFIG ──────────────────────────────────────────────────────────────────
I2C_ID, SDA, SCL, I2C_FREQ = 0, 8, 9, 400_000
GYRO_SAMPLES = 200

# Motor PWM bands ------------------------------------------------------------
MIN_FWD      = 100      # lowest PWM that reliably turns the wheel forward
BASE_SPEED   = 120     # "cruise" midpoint
MAX_PWM      = 155     # ceiling

# Kick‑start burst -----------------------------------------------------------
KICK_PWM     = 140     # overcome static friction
KICK_MS      = 250

# Control gains --------------------------------------------------------------
KP           = 4.0     # proportional gain (deg → PWM Δ)
YAW_BAND     = 2.0     # ignore error within ±1°

# Derived clamp for correction so no wheel < MIN_FWD or > MAX_PWM
CORR_MAX_POS = MAX_PWM - BASE_SPEED          # upward headroom
CORR_MAX_NEG = BASE_SPEED - MIN_FWD          # downward headroom (positive number)

# ─── HELPERS ────────────────────────────────────────────────────────────────

def clamp(val, lo, hi):
    return max(lo, min(val, hi))


def calibrate_bias(i2c, n):
    acc = 0.0
    for _ in range(n):
        acc += get_mpu6050_data(i2c)["gyro"]["z"]
        time.sleep_ms(5)
    return acc / n


def integrate(yaw, gz, bias, dt):
    return yaw + (gz - bias) * dt

# ─── INITIALISE ─────────────────────────────────────────────────────────────

i2c = I2C(I2C_ID, scl=Pin(SCL), sda=Pin(SDA), freq=I2C_FREQ)
init_mpu6050(i2c)
BIAS_Z = calibrate_bias(i2c, GYRO_SAMPLES)

car = Pico_Car()

# Kick‑start
car._set_motor_speeds(KICK_PWM, KICK_PWM)
time.sleep_ms(KICK_MS)

# Switch to cruise
car._set_motor_speeds(BASE_SPEED, BASE_SPEED)

# ─── CONTROL LOOP ───────────────────────────────────────────────────────────

yaw        = 0.0
last_tick  = time.ticks_ms()

target_yaw = 0.0   # initial heading

while True:
    now = time.ticks_ms()
    dt  = time.ticks_diff(now, last_tick) / 1000.0
    last_tick = now

    gz  = get_mpu6050_data(i2c)["gyro"]["z"]
    yaw = integrate(yaw, gz, BIAS_Z, dt)

    err = yaw - target_yaw
    if abs(err) < YAW_BAND:
        err = 0.0

    corr = clamp(KP * err, -CORR_MAX_NEG, CORR_MAX_POS)

    left_pwm  = clamp(BASE_SPEED + corr, MIN_FWD, MAX_PWM)
    right_pwm = clamp(BASE_SPEED - corr, MIN_FWD, MAX_PWM)

    car._set_motor_speeds(int(left_pwm), int(right_pwm))

    # Optional debug
    print(f"yaw={yaw:+6.1f}° err={err:+5.1f}°  L={left_pwm:.0f}  R={right_pwm:.0f}")

    time.sleep_ms(10)
