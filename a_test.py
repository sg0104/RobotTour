from pololu_3pi_2040_robot import robot
import time

# Initialize hardware components
motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()  # Correct IMU usage
display = robot.Display()
buttonA = robot.ButtonA()

# Initialize IMU
imu.reset()
imu.enable_default()

# PID Controller Class for Straight Movement Correction
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

# PID tuning values (adjust based on testing)
KP = 0.8
KI = 0.01
KD = 0.05

pid = PIDController(KP, KI, KD)

# Movement Constants
# Maximum speed
TURN_SPEED = 1000  # Turning speed
MOVE_TIME = 1.5  # Adjust for accurate 50 cm movement
TURN_ANGLE = 90  # Degrees for precise turning

def stop_motors():
    motors.set_speeds(0, 0)

def move_forward():
    display.fill(1)
    display.text("Moving Forward", 20, 20, 0)
    display.show()

    # Reset encoders
    left_start, right_start = encoders.get_counts()

    motors.set_speeds(-1000, -1000)
    start_time = time.time()

    while time.time() - start_time < MOVE_TIME:
        left_count, right_count = encoders.get_counts()
        left_diff = left_count - left_start
        right_diff = right_count - right_start

        # Compute error (difference between left and right distances)
        error = left_diff - right_diff  

        # Ignore very small errors (±3 counts) to prevent overcorrection
        if abs(error) > 3:
            correction = pid.compute(0, error)  # Target is 0 (balanced movement)
            correction = max(-300, min(300, correction))  # Limit correction to ±300 speed
        else:
            correction = 0  # No correction needed

        # Apply correction to motors
        motors.set_speeds(-1000 - correction, -1000 + correction)

        time.sleep(0.05)  # Small delay for stability

    stop_motors()
    time.sleep(0.2)

def move_backward():
    display.fill(1)
    display.text("Moving Backward", 8, 20, 0)
    display.show()

    # Reset encoders
    left_start, right_start = encoders.get_counts()

    motors.set_speeds(1000, 1000)
    start_time = time.time()

    while time.time() - start_time < MOVE_TIME:
        left_count, right_count = encoders.get_counts()
        left_count -= left_start
        right_count -= right_start

        correction = pid.compute(left_count, right_count)
        motors.set_speeds(1000 - correction, 1000 + correction)

    stop_motors()
    time.sleep(0.2)

def turn_right():
    display.fill(1)
    display.text("Turning Right", 8, 20, 0)
    display.show()

    motors.set_speeds(-TURN_SPEED, TURN_SPEED)  # Start turning right

    current_angle = 0
    start_time = time.time()  # Track how long it's turning

    while abs(current_angle) < TURN_ANGLE:
        if time.time() - start_time > 2:  # Timeout after 2 seconds
            break

        imu.read()  # Read IMU data
        gyro_data = imu.gyro.last_reading_dps[2]  # Get Z-axis rotation

        if abs(gyro_data) > 0.1:  # Ignore very small or zero values
            current_angle += gyro_data * 0.02  # Increase integration rate

        time.sleep(0.02)  # Increase step time

    stop_motors()
    time.sleep(0.2)

def turn_left():
    display.fill(1)
    display.text("Turning Left", 8, 20, 0)
    display.show()

    motors.set_speeds(TURN_SPEED, -TURN_SPEED)  # Start turning left

    current_angle = 0
    start_time = time.time()  # Track how long it's turning

    while abs(current_angle) < TURN_ANGLE:
        if time.time() - start_time > 2:  # Timeout after 2 seconds
            break

        imu.read()  # Read IMU data
        gyro_data = imu.gyro.last_reading_dps[2]  # Get Z-axis rotation

        if abs(gyro_data) > 0.1:  # Ignore very small or zero values
            current_angle += gyro_data * 0.02  # Increase integration rate

        time.sleep(0.02)  # Increase step time

    stop_motors()
    time.sleep(0.2)

def navigate_path():
    stop_motors()

    display.fill(1)
    display.text("Press A to Start", 2, 20, 0)
    display.show()

    while not buttonA.check():
        time.sleep(0.1)

    display.fill(1)
    display.text("Starting!", 30, 20, 0)
    display.show()
    time.sleep(1)

    move_forward()
    move_forward()
    move_forward()
    move_backward()

    stop_motors()

# Run the navigation function
navigate_path()