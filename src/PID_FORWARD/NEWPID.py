import math



# PID Constants for Distance Control
kP_distance = 2.8
kI_distance = 0.0
kD_distance = 1
timeout = 10
# PID Constant for Heading Correction
kP_heading = 0.05



def pid_drive(distance_inches, max_velocity_percent, timeout=3.0):

    LeftMotors.set_stopping(BRAKE)
    RightMotors.set_stopping(BRAKE)
    Left_Front.set_stopping(BRAKE)
    Right_front.set_stopping(BRAKE)
    """
    Drives the robot forward for a given distance (in inches) with PID control 
    and IMU-based heading correction.

    Args:
        distance_inches (float): Target distance to move (in inches).
        max_velocity_percent (float): Maximum motor speed (0-100%).
        timeout (float): Maximum time allowed for movement (seconds).
    """

    # Reset motor encoders
    LeftMotors.set_position(0, DEGREES)
    RightMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    Inertial21.set_rotation(0, DEGREES)

    # Capture the starting heading from the IMU
    target_heading = Inertial21.rotation()


    integral_distance = 0.0
    last_error_distance = 0.0
    start_time = brain.timer.time(SECONDS)  # Use Brain's timer

    while True:

        # Calculate error in distance (in encoder ticks)
        error_distance = distance_inches - (((RightMotors.position(DEGREES)/360)*math.pi*2.75)+((Right_front.position(DEGREES)/360)*math.pi*2.75)+((LeftMotors.position(DEGREES)/360)*math.pi*2.75)+((Left_Front.position(DEGREES)/360)*math.pi*2.75)/4) 

        # Break if we're within tolerance or timeout has been exceeded
        if abs(error_distance) < 3 or (brain.timer.time(SECONDS) - start_time) > timeout:
            break

        # Distance PID calculations
        integral_distance += error_distance
        derivative_distance = error_distance - last_error_distance
        last_error_distance = error_distance

        pid_output = (kP_distance * error_distance +
                      kI_distance * integral_distance +
                      kD_distance * derivative_distance)

        # Clamp the output to the maximum allowed velocity
        pid_output = max(-max_velocity_percent, min(max_velocity_percent, pid_output))
        
        # Heading correction using IMU
        current_heading = Inertial21.rotation()
        error_heading = target_heading - current_heading
        heading_correction = kP_heading * error_heading

        # Combine outputs: Add heading correction to the left side and subtract from the right
        left_output = pid_output + heading_correction
        right_output = pid_output - heading_correction

        # Clamp motor outputs to the maximum allowed velocity
        left_output = max(-max_velocity_percent, min(max_velocity_percent, left_output))
        right_output = max(-max_velocity_percent, min(max_velocity_percent, right_output))

        # Set motor velocities
        LeftMotors.set_velocity(left_output, PERCENT)
        RightMotors.set_velocity(right_output, PERCENT)
        Left_Front.set_velocity(left_output, PERCENT)
        Right_front.set_velocity(right_output, PERCENT)

        # Spin motors
        LeftMotors.spin(REVERSE)
        Left_Front.spin(REVERSE)
        RightMotors.spin(FORWARD)
        Right_front.spin(FORWARD)
        wait(0.05,SECONDS)

    # Stop motors when the movement is complete
    RightMotors.stop()
    Right_front.stop()
    LeftMotors.stop()
    Left_Front.stop()


import time
import math

# Constants for the robot's wheels and movement calculations
WHEEL_DIAMETER = 2.75  # Inches
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # Calculate the circumference of the wheel (inches)
DEGREES_PER_INCH = 360 / WHEEL_CIRCUMFERENCE  # Degrees of wheel rotation per inch of movement

# Function to get the current drive position by averaging the positions of all four motors
def get_drive_position():
    # Get the position of each motor (in degrees), convert to inches, and average them
    return (((RightMotors.position(DEGREES) / 360) * math.pi * 2.75) +
            ((Right_front.position(DEGREES) / 360) * math.pi * 2.75) +
            ((LeftMotors.position(DEGREES) / 360) * math.pi * 2.75) +
            ((Left_Front.position(DEGREES) / 360) * math.pi * 2.75)) / 4

# PID Drive function that controls both distance and heading to reach the target position
def pid_drive_curve(target_distance_inches, target_heading, max_velocity):
    global Inertial21, RightMotors, Right_front, LeftMotors, Left_Front, start_heading
    RightMotors.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    LeftMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    # Convert target distance from inches to wheel rotations in degrees
    target_distance = target_distance_inches
    
    # PID Constants for distance and turning
    Kp_dist, Ki_dist, Kd_dist = 0.6, 0.02, 0.4  # Proportional, Integral, Derivative for distance
    Kp_turn, Ki_turn, Kd_turn = 0.5, 0.01, 0.3  # Proportional, Integral, Derivative for turning
    
    # Get the current position and heading
    start_distance = get_drive_position()
    start_heading = Inertial21.rotation(DEGREES)
    target_pos = start_distance + target_distance  # Calculate target position
    
    # Initialize PID variables for distance and turning
    prev_error_dist, integral_dist = 0, 0
    prev_error_turn, integral_turn = 0, 0
    
    # Set a timeout for the movement in case it doesn't reach the target
    start_time = time.time()
    timeout = 10.0  # Timeout in seconds
    
    # Main control loop
    while True:
        # Calculate errors for distance and turning
        error_dist = target_pos - get_drive_position()
        error_turn = target_heading - Inertial21.rotation(DEGREES)
        
        # Break the loop if the robot is within a small error threshold for both distance and heading
        if abs(error_dist) < (3 * DEGREES_PER_INCH) and abs(error_turn) < 2:
            break
        
        # Stop if the timeout is reached
        if time.time() - start_time > timeout:
            break
        
        # Update the integral and derivative for distance
        integral_dist += error_dist
        derivative_dist = error_dist - prev_error_dist
        prev_error_dist = error_dist
        
        # Update the integral and derivative for turning
        integral_turn += error_turn
        derivative_turn = error_turn - prev_error_turn
        prev_error_turn = error_turn
        
        # Calculate the speed based on the PID formula for both distance and turning
        speed_dist = (Kp_dist * error_dist) + (Ki_dist * integral_dist) + (Kd_dist * derivative_dist)
        speed_turn = (Kp_turn * error_turn) + (Ki_turn * integral_turn) + (Kd_turn * derivative_turn)
        
        # Limit the speeds to the max velocity, ensuring they stay within the range
        speed_dist = max(min(speed_dist, max_velocity), 10)
        speed_turn = max(min(speed_turn, max_velocity / 2), -max_velocity / 2)
        
        # Calculate left and right motor speeds based on distance and turning adjustments
        left_speed = max(min(speed_dist + speed_turn, max_velocity), 10)
        right_speed = max(min(speed_dist - speed_turn, max_velocity), 10)
        
        # Set the motor velocities and spin the motors
        LeftMotors.set_velocity(left_speed, PERCENT)
        Left_Front.set_velocity(left_speed, PERCENT)
        RightMotors.set_velocity(right_speed, PERCENT)
        Right_front.set_velocity(right_speed, PERCENT)
        
        if target_distance_inches > 0:
            LeftMotors.spin(REVERSE)
            Left_Front.spin(REVERSE)
            RightMotors.spin(REVERSE)
            Right_front.spin(FORWARD)
        else:
            LeftMotors.spin(FORWARD)
            Left_Front.spin(FORWARD)
            RightMotors.spin(FORWARD)
            Right_front.spin(REVERSE)
        # Small delay to prevent the loop from running too fast
        wait(0.01, SECONDS)
    
    # Stop all motors after the movement is complete
    RightMotors.stop()
    LeftMotors.stop()
    Right_front.stop()
    Left_Front.stop()

    





