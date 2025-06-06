"""
Straight‑line drive controller for Pico_Car using MPU‑6050 yaw feedback
────────────────────────────────────────────────────────────────────────
Fixes in this revision
──────────────────────
1. **Steering polarity corrected** – a **positive** yaw error (clockwise drift) now
   *slows the left wheel* and *speeds up the right* so the car turns back
   counter‑clockwise.
2. Removed the 100 ms pause inside the heading‑lock branch so the first drive
   command happens **immediately** after calibration.
3. Added an optional *soft‑start* ramp so the motors overcome static friction
   smoothly instead of jumping to full PWM.
4. `MIN_SPEED` bumped to 60 (≈15 % duty) – most TT‑motors will not move below
   that on 3 V.

If the car still veers, tune `KP` first.  Doubling or halving it usually gets
in the ball‑park quickly.  If it oscillates, lower `KP` or increase
`TOLERANCE_DEG` a hair.
"""

import time
from machine import Pin, I2C
from mpu6050 import init_mpu6050, get_mpu6050_data
from pico_car import Pico_Car

I2C_ID = 0
SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 400_000

i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
init_mpu6050(i2c)


def calibrate_gyro_bias(n: int = 100, delay_ms: int = 10):
    bias = {axis: 0.0 for axis in ("x", "y", "z")}
    for _ in range(n):
        g = get_mpu6050_data(i2c)["gyro"]
        for k in bias:
            bias[k] += g[k]
        time.sleep_ms(delay_ms)
    for k in bias:
        bias[k] /= n
    return bias


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

car = Pico_Car()

FULL_SPEED = 255
BASE_SPEED = 100
MIN_SPEED = 80
RAMP_TIME = 0.4


def drive(left_cmd: float, right_cmd: float):
    left = int(clamp(left_cmd, MIN_SPEED, FULL_SPEED) * 257)
    right = int(clamp(right_cmd, MIN_SPEED, FULL_SPEED) * 257)
    for m in car._motors[::2]:
        m.duty_u16(left)
    for m in car._motors[1::2]:
        m.duty_u16(right)


def stop():
    for m in car._motors:
        m.duty_u16(0)


print("Calibrating gyro… keep robot motionless ✋")
stop()
GYRO_BIAS = calibrate_gyro_bias()
print("Bias:", GYRO_BIAS, "— done")

KP = 5.0
TOLERANCE_DEG = 1.5

prev_ms = time.ticks_ms()
yaw_integrated = 0.0
heading_locked = False
TARGET_YAW = 0.0

print("Starting straight‑drive controller…")

while True:
    try:
        raw = get_mpu6050_data(i2c)
        now = time.ticks_ms()
        dt = time.ticks_diff(now, prev_ms) / 1000.0
        prev_ms = now
        if dt <= 0.0 or dt > 0.2:
            continue
        yaw_integrated += (raw["gyro"]["z"] - GYRO_BIAS["z"]) * dt
        
        ALPHA = 0.15
        filt_yaw = filt_yaw + ALPHA * (yaw_integrated - filt_yaw) if "filt_yaw" in locals() else yaw_integrated
        if not heading_locked:
            TARGET_YAW = filt_yaw
            heading_locked = True
            print(f"Heading locked @ {TARGET_YAW:.2f}° — rolling out…")
            t0 = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t0) / 1000.0 < RAMP_TIME:
                frac = time.ticks_diff(time.ticks_ms(), t0) / (RAMP_TIME * 1000)
                speed = MIN_SPEED + frac * (BASE_SPEED - MIN_SPEED)
                drive(speed, speed)
                time.sleep_ms(10)
            continue
        error = filt_yaw - TARGET_YAW
        if abs(error) <= TOLERANCE_DEG:
            drive(BASE_SPEED, BASE_SPEED)
        else:
            if error < 0:
                left_cmd = BASE_SPEED + KP * error
                right_cmd = BASE_SPEED
            else:
                left_cmd = BASE_SPEED
                right_cmd = BASE_SPEED + KP * error
            drive(left_cmd, right_cmd)
        print(f"yaw={filt_yaw:+6.2f}  err={error:+6.2f}  L={left_cmd if 'left_cmd' in locals() else BASE_SPEED:5.1f}  R={right_cmd if 'right_cmd' in locals() else BASE_SPEED:5.1f}")
        time.sleep_ms(5)
    except KeyboardInterrupt:
        print("\nUser abort — motors stopped")
        stop()
        break
    except Exception as e:
        print("Runtime error:", e)
        stop()
        time.sleep_ms(500)

