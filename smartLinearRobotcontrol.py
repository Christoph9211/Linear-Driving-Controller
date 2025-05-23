import time
from kalman_Yaw import KalmanFilter

# Constants for motor directions
DIRECTION_VOID = 0    # No movement
DIRECTION_FORWARD = 1 # Forward motion (direction_just)
DIRECTION_BACK = 2    # Backward motion (direction_back)

# Initialize Kalman filter for yaw angle
kalman_yaw = KalmanFilter(process_variance=1e-3, measurement_variance=1e-2, estimated_measurement_variance=1e-2)

yaw = 0.0
prev_time = time.ticks_ms()


   
# Stub functions – replace these with actual implementations
def motor_control(direction_A, speed_A, direction_B, speed_B, control_enabled):
    # Replace with actual motor control code.
    print("Motor A -> Direction: {}, Speed: {}; Motor B -> Direction: {}, Speed: {}"
          .format(direction_A, speed_A, direction_B, speed_B))

# def mpu6050_get_yaw():
#     while True:
#         data = get_mpu6050_data(i2c)
#         curr_time = time.ticks_ms()
#         dt = (curr_time - prev_time) / 1000.0  # convert ms to seconds
#         prev_time = curr_time

#         # Integrate the gyro's z-axis (assuming data['gyro']['z'] is in degrees per second)
#         raw_yaw = yaw + data['gyro']['z'] * dt

#         # Filter the raw yaw using the Kalman filter
#         yaw = kalman_yaw.update(raw_yaw)

#         print("Filtered Yaw: {:.2f} degrees".format(yaw))
# #         time.sleep_ms(5)

def car_leave_the_ground():
    # Replace with actual check to see if the car is properly on the ground.
    return True

def smart_robot_car_linear_motion_control(direction, direction_record, speed, kp, upper_limit, yaw, yaw_storage):
    """
    Moves the robot in a straight line (forward or backward) while correcting
    for drift using the yaw from an MPU6050 sensor.
    
    Parameters:
      direction: "Forward" or "Backward"
      direction_record: an integer record to detect changes in motion commands
      speed: base speed (0-255)
      kp: proportional gain for correcting yaw error
      upper_limit: maximum allowed speed command for correction
    """
    # Static variables are stored as attributes of the function
    
#     smart_robot_car_linear_motion_control.yaw = 0.0
#     smart_robot_car_linear_motion_control.yaw_storage = 0.0
#     smart_robot_car_linear_motion_control.en = 110  # initial value
#     smart_robot_car_linear_motion_control.is_time = time.ticks_ms()
#     
    current_time = time.ticks_ms()
    
    # If the driving direction has changed or more than 10ms has passed, update sensor reading
    if (smart_robot_car_linear_motion_control.en != direction_record or 
        time.ticks_diff(current_time, smart_robot_car_linear_motion_control.is_time) > 10):
        # Stop motors before reading new sensor data
        motor_control(DIRECTION_VOID, 0, DIRECTION_VOID, 0, True)
        smart_robot_car_linear_motion_control.yaw = kalman_yaw.update(yaw)
        smart_robot_car_linear_motion_control.is_time = current_time

    # If the direction record has changed or the car is not on the ground,
    # update the stored reference yaw value.
    if (smart_robot_car_linear_motion_control.en != direction_record or 
        not car_leave_the_ground()):
        smart_robot_car_linear_motion_control.en = direction_record
        smart_robot_car_linear_motion_control.yaw_storage = smart_robot_car_linear_motion_control.yaw

    # Calculate right and left motor speeds using proportional control
    R = (smart_robot_car_linear_motion_control.yaw - smart_robot_car_linear_motion_control.yaw_storage) * kp + speed
    if R > upper_limit:
        R = upper_limit
    elif R < 10:
        R = 10

    L = (smart_robot_car_linear_motion_control.yaw_storage - smart_robot_car_linear_motion_control.yaw) * kp + speed
    if L > upper_limit:
        L = upper_limit
    elif L < 10:
        L = 10

    # Command the motors based on the desired direction
    if direction == "Forward":
        motor_control(DIRECTION_FORWARD, int(R), DIRECTION_FORWARD, int(L), True)
    elif direction == "Backward":
        motor_control(DIRECTION_BACK, int(L), DIRECTION_BACK, int(R), True)

# Example usage:
# To move forward with a base speed of 100, a Kp of 2, and an upper limit of 180:
smart_robot_car_linear_motion_control("Forward", direction_record=1, speed=100, kp=2, upper_limit=180, yaw=0, yaw_storage=0)
