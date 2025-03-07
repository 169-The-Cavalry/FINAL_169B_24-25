import time

# Autonomous Mode Options
auto_modes = [
    "[NO AUTO]",
    "[RED LEFT RING]",
    "[BLUE RIGHT RING]",
    "[RED RIGHT STAKE]",
    "[BLUE LEFT STAKE]"
]

AutoSelect = 0  # Default to NO AUTO

def update_auto_display():
    """Updates the controller screen with the selected autonomous mode."""
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print(auto_modes[AutoSelect])
    controller_1.screen.set_cursor(2, 1)
    controller_1.screen.print("<< L1 | L2 >>")  # Navigation instructions

def next_auto_mode():
    """Scroll to the NEXT autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect + 1) % len(auto_modes)
    controller_1.rumble(".")
    update_auto_display()

def previous_auto_mode():
    """Scroll to the PREVIOUS autonomous mode."""
    global AutoSelect
    AutoSelect = (AutoSelect - 1) % len(auto_modes)
    controller_1.rumble(".")
    update_auto_display()

def when_started6():
    """Initialize the auto selector."""
    controller_1.screen.clear_screen()
    controller_1.screen.set_cursor(1, 1)
    controller_1.screen.print("AUTO SELECT MODE")
    time.sleep(0.5)  # Brief delay before displaying options
    update_auto_display()

# Bind button events

def onevent_controller_1buttonL1_pressed_0():
    next_auto_mode()

def onevent_controller_1buttonL2_pressed_0():
    previous_auto_mode()









#region VEXcode Generated Robot Configuration
from vex import *
'''from DRIVER_FUNCTIONS.drive import ondriver_drivercontrol_0, ondriver_drivercontrol_1, ondriver_drivercontrol_2, ondriver_drivercontrol_3, onauton_autonomous_0, onevent_controller_1axis2Changed_0, onevent_controller_1axis3Changed_0
from AUTO.autoselect import onevent_controller_1buttonL1_pressed_0
from INIT.init import when_started4, onevent_stop_initialize_0
from DRIVER_FUNCTIONS.LB import ondriver_drivercontrol_4
from AUTO.autonomous import onauton_autonomous_0
from OTHER.testcode import when_started2, when_started3
from AUTO.autoselect import when_started5'''
import urandom # type: ignore

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
RightMotors_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
RightMotors_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
RightMotors = MotorGroup(RightMotors_motor_a, RightMotors_motor_b)
LeftMotors_motor_a = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
LeftMotors_motor_b = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
LeftMotors = MotorGroup(LeftMotors_motor_a, LeftMotors_motor_b)
Right_front = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
Left_Front = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
digital_out_b = DigitalOut(brain.three_wire_port.b)
rotation_15 = Rotation(Ports.PORT15, False)
Lady_Brown_motor_a = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
Lady_Brown_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)
Lady_Brown = MotorGroup(Lady_Brown_motor_a, Lady_Brown_motor_b)
Inertial21 = Inertial(Ports.PORT5)
digital_out_g = DigitalOut(brain.three_wire_port.g)
optical_4 = Optical(Ports.PORT4)
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
digital_out_e = DigitalOut(brain.three_wire_port.e)
gps_3 = Gps(Ports.PORT3, 0.00, 5.00, MM, -90)
distance_optical = Optical(Ports.PORT11)



# wait for rotation sensor to fully initialize
wait(30, MSEC)


"""# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()"""


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

remote_control_code_enabled = True
vexcode_brain_precision = 0
vexcode_console_precision = 0
vexcode_controller_1_precision = 0
message1 = Event()
forward_move = Event()
Back_move = Event()
Stop = Event()
turn_right = Event()
'''turn = Event()'''
calibrate = Event()
stop_initialize = Event()
Auto_Stop = Event()
turn_left = Event()
start_auto = Event()
intake_forward = Event()
intake_backward = Event()
DOon = False
IntakeF = False
INTAKER = False
LB = False
DOon2 = False
DOon3 = False
Blue = False
RED = True
BLUE = False
Intake_Control = False
Intake_running = False
myVariable = 0
volocity = 0
Right_Axis = 0
Left_Axis = 0
IntakeStake = 0
Degree = 0
pi = 0
movement = 0
distance1 = 0
time1 = 0
rot = 0
turn1 = 0
LadyBrown_Up = 0
LadyBrown_score = 0
LadyBrown = 0
Right_turn = 0
Left_turn = 0
DriveState = 0
start = 0
Next = 0
dos = 0
tog = 0
error = 0
output = 0
Kp = 0
Ki = 0
Kd = 0
Dellay = 0
Distance_travled = 0
imput = 0
Proportional = 0
integral = 0
derivitive = 0
direction = 0
Previus_error = 0
AutoSelect = 0
X_Start = 0
Y_Start = 0
Y_End = 0
X_End = 0
Angle = 0
Distnce2 = 0
Distance2 = 0
Turn_Angle = 0



def ondriver_drivercontrol_6():
    global rotation_15, Lady_Brown, controller_1

    # LADY BROWN INITIALIZATION
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)
    
    while True:
        # Move down (controller DOWN button)
        if controller_1.buttonDown.pressing() and rotation_15.position(DEGREES) > 198:
            Lady_Brown.set_velocity(40, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 276:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move up (controller UP button)
        elif controller_1.buttonUp.pressing():
            Lady_Brown.set_velocity(100, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(FORWARD)

            while rotation_15.position(DEGREES) > 152:
                wait(5, MSEC)

            Lady_Brown.set_stopping(HOLD)
            Lady_Brown.stop()

        # Move back (controller LEFT button) with SAFEGUARD
        elif controller_1.buttonLeft.pressing():
            Lady_Brown.set_velocity(80, PERCENT)
            Lady_Brown.set_stopping(COAST)
            Lady_Brown.spin(REVERSE)

            start_time = time.time()  # Record start time
            while rotation_15.position(DEGREES) < 300:
                if time.time() - start_time > 2:  # If movement takes longer than 2 sec
                    print("⚠️ SAFEGUARD TRIGGERED: Movement timeout!")
                    break
                wait(5, MSEC)

            Lady_Brown.set_stopping(COAST)
            Lady_Brown.stop()
            rotation_15.set_position(300, DEGREES)

        wait(5, MSEC)  # Reduce CPU usage'''



'''def ondriver_drivercontrol_4():
    global rotation_15, Lady_Brown, controller_1

    # PID Constants (Tune these values)
    Kp = 0.6   # Proportional Gain
    Ki = 0.002 # Integral Gain
    Kd = 0.3   # Derivative Gain

    integral = 0
    last_error = 0

    def pid_control(target_position):
        """PID control to move Lady_Brown to the target position."""
        global integral, last_error

        tolerance = 2  # Acceptable error margin in degrees
        max_speed = 100  # Max motor speed
        min_speed = 15   # Minimum speed to prevent stalling

        while True:
            current_position = rotation_15.position(DEGREES)
            error = target_position - current_position

            # PID Calculations
            integral += error
            derivative = error - last_error
            last_error = error

            # Compute output speed
            output = (Kp * error) + (Ki * integral) + (Kd * derivative)

            # Constrain speed
            speed = max(min(abs(output), max_speed), min_speed)

            # Set motor direction and speed
            if error > 0:
                Lady_Brown.spin(FORWARD, speed, PERCENT)
            else:
                Lady_Brown.spin(REVERSE, speed, PERCENT)

            # Stop if within tolerance
            if abs(error) <= tolerance:
                break

            wait(10, MSEC)

        Lady_Brown.stop(HOLD)  # Hold position when finished

    # Initialize Lady_Brown
    Lady_Brown.spin_to_position(0, DEGREES)
    rotation_15.set_position(300, DEGREES)
    Lady_Brown.set_timeout(1, SECONDS)

    while True:
        # Move Down (Button DOWN)
        if controller_1.buttonDown.pressing() and rotation_15.position(DEGREES) > 198:
            pid_control(276)

        # Move Up (Button UP)
        elif controller_1.buttonUp.pressing():
            pid_control(152)

        # Move Back (Button LEFT) with SAFEGUARD
        elif controller_1.buttonLeft.pressing():
            start_time = time.time()
            pid_control(300)  # Move to initial position

            # Safeguard: If movement takes too long, stop the motor
            if time.time() - start_time > 2:
                print("⚠️ SAFEGUARD TRIGGERED: Movement timeout!")
                Lady_Brown.stop(COAST)

        wait(5, MSEC)  # Reduce CPU usage'''



def when_started4():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CALIBRATE AND INIT
    optical_4.gesture_disable()
    optical_4.set_light(LedStateType.ON)
    start = 1
    Degree = 0
    pi = 3.14159265359
    Lady_Brown.set_velocity(100, PERCENT)
    Lady_Brown.set_position(0, DEGREES)
    intake.set_velocity(80, PERCENT)
    movement = 0
    Intake_Control = True
    Inertial21.calibrate()
    while Inertial21.is_calibrating():
        sleep(50)


def onevent_stop_initialize_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # INIT
    RightMotors.set_stopping(BRAKE)
    LeftMotors.set_stopping(BRAKE)
    Right_front.set_stopping(BRAKE)
    Left_Front.set_stopping(BRAKE)
    Lady_Brown.set_stopping(BRAKE)
    intake.set_stopping(BRAKE)
    RightMotors.set_velocity(0, PERCENT)
    LeftMotors.set_velocity(0, PERCENT)
    Right_front.set_velocity(0, PERCENT)
    Left_Front.set_velocity(0, PERCENT)
    RightMotors.set_max_torque(100, PERCENT)
    LeftMotors.set_max_torque(100, PERCENT)
    Right_front.set_max_torque(100, PERCENT)
    Left_Front.set_max_torque(100, PERCENT)

def Move_In_direction_Degree_Speed(Move_In_direction_Degree_Speed__Degree, Move_In_direction_Degree_Speed__Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # LINKED WITH CURRENT PID FOR MOVEMENT VELOCITY
    if Move_In_direction_Degree_Speed__Degree > 0:
        RightMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Right_front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        LeftMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Left_Front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
    else:
        RightMotors.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        Right_front.set_velocity(Move_In_direction_Degree_Speed__Speed, PERCENT)
        LeftMotors.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)
        Left_Front.set_velocity((Move_In_direction_Degree_Speed__Speed), PERCENT)


def Forward_PID_Distance_Max_Speed(Forward_PID_Distance_Max_Speed__Distance, Forward_PID_Distance_Max_Speed__Max_Speed):
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CURRENT PID - DISTANCE IN INCHES
    # PID VALUES: 0.5 - 0.01 - 0.1 - 0.05
    error = 0
    Kp = 0.45
    Ki = 0
    Kd = 0.01
    Dellay = 0.05

    Distance_travled = 0
    Inertial21.set_rotation(0, DEGREES)
    RightMotors.set_velocity(5, PERCENT)
    Right_front.set_velocity(5, PERCENT)
    LeftMotors.set_velocity(5, PERCENT)
    Left_Front.set_velocity(5, PERCENT)
    RightMotors.set_position(0, DEGREES)
    Right_front.set_position(0, DEGREES)
    LeftMotors.set_position(0, DEGREES)
    Left_Front.set_position(0, DEGREES)
    while True:
        imput = Inertial21.rotation(DEGREES)
        error = 0 - imput
        Proportional = error
        Distance_travled = ((math.fabs(Right_front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(Left_Front.position(DEGREES)) / 360) * (2.75 * pi) + ((math.fabs(RightMotors.position(DEGREES)) / 360) * (2.75 * pi) + (math.fabs(LeftMotors.position(DEGREES)) / 360) * (2.75 * pi)))) / 4
        integral = (integral + error) * Dellay
        derivitive = (error - Previus_error) * Dellay
        direction = (Kp * Proportional + (Ki * integral + Kd * derivitive))
        Previus_error = error
        Move_In_direction_Degree_Speed((direction * Forward_PID_Distance_Max_Speed__Max_Speed) / 33, Forward_PID_Distance_Max_Speed__Max_Speed)
        RightMotors.spin(REVERSE)
        Right_front.spin(REVERSE)
        LeftMotors.spin(FORWARD)
        Left_Front.spin(FORWARD)
        if Forward_PID_Distance_Max_Speed__Distance < Distance_travled:
            RightMotors.stop()
            Right_front.stop()
            LeftMotors.stop()
            Left_Front.stop()
            break
        wait(5, MSEC)





        


# create a function for handling the starting and stopping of all autonomous tasks


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

    





import time

def onauton_autonomous_0():
    global turn_heading_velocity_momentum, IntakeF, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False
    

    stop_initialize.broadcast()
    # AUTO SELECT
    intake.set_velocity(90, PERCENT)
    BLUE_RIGHT_RING_ELIMS()
    '''BLUE_RIGHT_RING() BLUE_LEFT_SAFE() BLUE_LEFT_SAFE_ELIMS() RED_LEFT_RING_ELIMS()
    BLUE_RIGHT_RING_ELIMS()
    BLUE_RIGHT_RING()
    RED_LEFT_RING()
    RED_RIGHT_SAFE()
    BLUE_LEFT_SAFE()'''




import time

def onauton_autonomous_1():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF, intake
    while True:
        controller_1.screen.clear_screen()
        wait(0.1, SECONDS)
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print(intake.efficiency(PERCENT))
        while intake.efficiency(PERCENT) == 0 and IntakeF == True:
            controller_1.screen.clear_screen()
            controller_1.screen.set_cursor(1, 2)
            controller_1.screen.print("intake:(")
            intake.spin(REVERSE)
            wait(0.2, SECONDS)
            intake.spin(FORWARD)
    
        '''if intake.efficiency(PERCENT) < 10 and IntakeF == True:
            startT = brain.timer.time(SECONDS)
        if intake.efficiency(PERCENT) < 10 and IntakeF == True and (brain.timer.time(SECONDS) - startT) <= 2:
            wait(2, SECONDS)
        if intake.efficiency(PERCENT) < 10 and IntakeF == True and (brain.timer.time(SECONDS) - startT) <= 2:
            intake.spin(REVERSE)
            wait(0.3, SECONDS)
            intake.spin(FORWARD)'''
    

'''def onauton_autonomous_1():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF, intake
    while True:
        if intake.efficiency(PERCENT) < 10 and IntakeF == True:
            startT = brain.timer.time(SECONDS)
        while IntakeF == True and intake.efficiency(PERCENT) < 10 and (brain.timer.time(SECONDS) - startT) >= 2:
            controller_1.screen.set_cursor(1, 1)
            controller_1.screen.clear_screen()
            wait(0.1, SECONDS)
            controller_1.screen.print(intake.efficiency(PERCENT))
            intake.spin(REVERSE)
            wait(0.3, SECONDS)
            intake.spin(FORWARD)
            wait(1, SECONDS)'''





def run_autonomous():
    global AutoSelect
    """Executes the selected autonomous routine."""
    if AutoSelect == 0:
        no_auto()
    elif AutoSelect == 0.1:
        RED_LEFT_RING()
    elif AutoSelect == 2:
        BLUE_RIGHT_RING()
    elif AutoSelect == 3:
        RED_RIGHT_SAFE()
    elif AutoSelect == 4:
        BLUE_LEFT_SAFE()

# === AUTONOMOUS ROUTINES ===

def no_auto():
    global brain
    """No autonomous action."""
    brain.screen.print("No autonomous selected.")

    
def BLUE_LEFT_GOAL_RUSH_ELIMS():
    global IntakeF, RED
    RED = True
    pid_drive(57, 100)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    pid_drive(-40, 100)


def RED_LEFT_RING():
    global IntakeF, BLUE
    pid_drive(15, 70)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-50, 60)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    pid_turn(150, 100)
    wait(0.1, SECONDS)
    intake.spin(FORWARD)
    pid_drive(39, 100)
    wait(1, SECONDS)
    pid_drive(-20, 100)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(21, 100)
    wait(0.5, SECONDS)
    pid_turn(70, 100)
    wait(0.1, SECONDS)
    pid_drive(24, 100)
    wait(0.5, SECONDS)
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(120, 100)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(350, DEGREES)
    pid_drive(47, 100)

def RED_LEFT_RING_ELIMS():
    global IntakeF, BLUE
    BLUE = True
    digital_out_g.set(True)
    digital_out_e.set(True)
    IntakeF = True
    intake.spin(FORWARD)
    pid_drive(62, 100)
    wait(0.5, SECONDS)
    IntakeF = False
    intake.stop()
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(65, 40)
    wait(0.1, SECONDS)
    digital_out_g.set(False)
    digital_out_e.set(False)
    pid_drive(-27, 40)
    wait(0.5, SECONDS)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_drive(33, 100)
    wait(0.5, SECONDS)
    pid_turn(-53, 100)
    wait(0.1, SECONDS)
    pid_drive(19, 100)
    wait(0.5, SECONDS)
    pid_drive(-24, 100)
    pid_turn(120, 100)
    pid_drive(24, 100)



def BLUE_RIGHT_RING_ELIMS():
    global IntakeF, RED
    RED = True
    digital_out_g.set(True)
    digital_out_e.set(True)
    IntakeF = True
    intake.spin(FORWARD)
    pid_drive(62, 100)
    wait(0.5, SECONDS)
    IntakeF = False
    intake.stop()
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(65, 40)
    wait(0.1, SECONDS)
    digital_out_g.set(False)
    digital_out_e.set(False)
    pid_drive(-27, 40)
    wait(0.5, SECONDS)
    digital_out_b.set(True)
    IntakeF = True
    intake.spin(FORWARD)
    pid_turn(5, 100)
    pid_drive(33, 100)
    wait(0.5, SECONDS)
    pid_turn(-55, 100)
    wait(0.1, SECONDS)
    pid_drive(22, 100)
    wait(0.5, SECONDS)
    pid_drive(-24, 100)
    wait(0.1, SECONDS)
    pid_turn(140, 100)
    wait(0.1, SECONDS)
    pid_drive(40, 100)
    wait(0.1, SECONDS)
    pid_turn(-30, 100)
    wait(0.1, SECONDS)
    pid_drive(55, 100)
    pid_drive(15, 100)
    wait(0.5, SECONDS)
    pid_drive(-10, 100)


    

def BLUE_RIGHT_RING():
    global IntakeF, RED
    RED = True
    pid_drive(15, 70)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-50, 60)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    pid_turn(-150, 100)
    wait(0.1, SECONDS)
    intake.spin(FORWARD)
    pid_drive(39, 100)
    wait(1, SECONDS)
    pid_drive(-20, 100)
    pid_turn(40, 100)
    wait(0.1, SECONDS)
    pid_drive(21, 100)
    wait(0.5, SECONDS)
    pid_turn(-70, 100)
    wait(0.1, SECONDS)
    pid_drive(24, 100)
    wait(0.5, SECONDS)
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(-120, 100)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(350, DEGREES)
    pid_drive(47, 100)

    
    
def RED_RIGHT_SAFE():
    global IntakeF, BLUE
    BLUE = True
    pid_drive(-35, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(45, 100)
    intake.spin(FORWARD)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    wait(0.3, SECONDS)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    pid_turn(-20, 100)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    intake.stop()
    pid_drive(39, 60)
    wait(0.1, SECONDS)
    pid_turn(-160, 100)
    digital_out_g.set(False)
    pid_turn(160, 40)
    intake.spin(FORWARD)
    wait(0.1, SECONDS)
    pid_drive(15, 100)
    wait(1.3, SECONDS)
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(180, 70)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    Lady_Brown.spin_to_position(250, DEGREES)
    pid_drive(40, 100)
    'LADDER VAR'
    '''pid_turn(150, 70)
    wait(0.1, SECONDS)
    pid_drive(15, 100)
    digital_out_b.set(False)
    pid_turn(-20, 70)
    pid_drive(15, 100)
    wait(0.1, SECONDS)
    pid_drive(20, 100)
    digital_out_g.set(True)'''

    
def BLUE_LEFT_SAFE_ELIMS():
    global IntakeF, RED
    RED = True
    pid_drive(-35, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(55, 100)
    intake.spin(FORWARD)
    IntakeF = True
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    wait(0.3, SECONDS)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    pid_turn(-20, 100)
    wait(0.1, SECONDS)
    pid_drive(47, 50)
    wait(0.5, SECONDS)
    pid_drive(-9, 30)
    wait(0.5, SECONDS)
    pid_drive(11, 100)
    wait(0.8, SECONDS)
    pid_drive(-18, 100)
    pid_turn(60, 100)
    digital_out_b.set(False)
    pid_turn(-40, 100)
    pid_drive(-25, 100)



def BLUE_LEFT_SAFE():
    global IntakeF, RED
    RED = True
    pid_drive(-35, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(55, 100)
    intake.spin(FORWARD)
    IntakeF = True
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    wait(0.3, SECONDS)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    pid_turn(-20, 100)
    wait(0.1, SECONDS)
    pid_drive(47, 50)
    wait(0.5, SECONDS)
    pid_drive(-18, 100)
    pid_turn(200, 100)
    Lady_Brown.spin_to_position(250, DEGREES)
    pid_drive(70, 100)






def ondriver_drivercontrol_4():
    
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # INTAKE CONTROLLER CONTROL
    while True:
        while Intake_Control:
            if controller_1.buttonR1.pressing():
                intake.set_velocity(100, PERCENT)
                intake.spin(FORWARD)
            elif controller_1.buttonR2.pressing():
                intake.set_velocity(100, PERCENT)
                intake.spin(REVERSE)
            else:
                intake.stop()
            wait(5, MSEC)
        wait(5, MSEC)
  



def onevent_controller_1axis2Changed_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER jOYSTICK
    Right_Axis = controller_1.axis2.position()




def onevent_controller_1axis3Changed_0():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER jOYSTICK
    Left_Axis = controller_1.axis3.position()

def ondriver_drivercontrol_1():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER MOTOR VELOCITY CONTROL
    remote_control_code_enabled = True
    DriveState = 1
    volocity = 200
    RightMotors.set_stopping(COAST)
    LeftMotors.set_stopping(COAST)
    Right_front.set_stopping(COAST)
    Left_Front.set_stopping(COAST)
    while True:
        if Right_Axis > 0:
            Right_Axis =  30 * Right_Axis ** 1/3
        else:
            Right_Axis = 30 * -(math.fabs(Right_Axis) ** 1/3)

        if Left_Axis > 0:
            Left_Axis = 30 * (Left_Axis) ** 1/3
        else:
            Left_Axis = 30 * -(math.fabs(Left_Axis) ** 1/3)

        while True:
            RightMotors.set_velocity(Right_Axis, PERCENT)
            LeftMotors.set_velocity(Left_Axis, PERCENT)
            Right_front.set_velocity(Right_Axis, PERCENT)
            Left_Front.set_velocity(Left_Axis, PERCENT)
            RightMotors.spin(FORWARD)
            LeftMotors.spin(REVERSE)
            Right_front.spin(FORWARD)
            Left_Front.spin(REVERSE)
            wait(5, MSEC)
        wait(5, MSEC)

def ondriver_drivercontrol_7():
    pass
    '''global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF, intake
    while intake.is_spinning and controller_1.buttonR1.pressing and Intake_Control == True and :
        if intake.velocity(PERCENT) == 0 :
            brain.screen.print("STUCK")
            Intake_Control = False
            intake.spin(REVERSE)
            wait(0.3, SECONDS)
            intake.spin(FORWARD)
            Intake_Control = True
            wait(2, SECONDS)'''
def ondriver_drivercontrol_2():
    global DOon  # Assuming DOon keeps track of clamp state
    previous_button_state = False  # Track previous button press

    while True:
        current_button_state = controller_1.buttonB.pressing()

        # Detect the moment the button is first pressed
        if current_button_state and not previous_button_state:
            DOon = not DOon  # Toggle state
            digital_out_b.set(DOon)  # Actuate clamp
            wait(0.1, SECONDS)  # Debounce delay

        previous_button_state = current_button_state  # Update state tracking
        wait(5, MSEC)  # Prevent CPU overuse

'''def ondriver_drivercontrol_2():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # CONTROLLER CLAMP CONTROL
    while True:
        if controller_1.buttonB.pressing():
            if DOon:
                digital_out_b.set(True)
                DOon = False
                wait(0.1, SECONDS)
            else:
                digital_out_b.set(False)
                DOon = True
                wait(0.1, SECONDS)
            wait(0.2, SECONDS)
        wait(5, MSEC)'''

def ondriver_drivercontrol_3():
    global DOon2  # Assuming DOon keeps track of clamp state
    previous_button_state2 = False  # Track previous button press

    while True:
        current_button_state2 = controller_1.buttonY.pressing()

        # Detect the moment the button is first pressed
        if current_button_state2 and not previous_button_state2:
            DOon2 = not DOon2  # Toggle state
            digital_out_e.set(DOon2)  # Actuate clamp
            wait(0.1, SECONDS)  # Debounce delay

        previous_button_state2 = current_button_state2  # Update state tracking
        wait(5, MSEC)  # Prevent CPU overuse



def ondriver_drivercontrol_0():
    global DOon3  # Assuming DOon keeps track of clamp state
    previous_button_state3 = False  # Track previous button press

    while True:
        current_button_state3 = controller_1.buttonA.pressing()

        # Detect the moment the button is first pressed
        if current_button_state3 and not previous_button_state3:
            DOon3 = not DOon3  # Toggle state
            digital_out_g.set(DOon3)  # Actuate clamp
            wait(0.1, SECONDS)  # Debounce delay

        previous_button_state3 = current_button_state3  # Update state tracking
        wait(5, MSEC)  # Prevent CPU overuse
        
optical_value = optical_4.hue()
def ondriver_drivercontrol_5():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    global intake, optical_4, Intake_Control, BLUE, RED
    """Stops intake when a blue ring is detected."""
    while True:
            # Check if optical sensor detects a blue ring (DEFAULTS TO BLUE)
        
        '''while BLUE:
            if  240 > optical_4.hue() > 190:
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                wait(0.3, SECONDS)
                intake.spin(REVERSE)
                wait(0.3, SECONDS)
                intake.stop()
                wait(0.5, SECONDS)
                Intake_Control = True'''
        while RED:
            # Check if optical sensor detects a red ring
            if    30 > optical_4.hue() > 0:
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                wait(0.3, SECONDS)
                intake.spin(REVERSE)
                wait(0.3, SECONDS)
                intake.stop()
                wait(0.5, SECONDS)
                Intake_Control = True


def onauton_autonomous_2():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    global intake, optical_4, Intake_Control, BLUE, RED
    """Stops intake when a blue ring is detected."""
    while True:
            # Check if optical sensor detects a blue ring (DEFAULTS TO BLUE)
        
        while BLUE:
            if  240 > optical_4.hue() > 190:
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                wait(0.3, SECONDS)
                intake.spin(REVERSE)
                wait(0.3, SECONDS)
                intake.stop()
                wait(0.5, SECONDS)
                Intake_Control = True
        while RED:
            # Check if optical sensor detects a red ring
            if   30 > optical_4.hue() > 0:
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                wait(0.3, SECONDS)
                intake.spin(REVERSE)
                wait(0.3, SECONDS)
                intake.stop()
                wait(0.5, SECONDS)
                Intake_Control = True
'''def BLUE_EJECT():
    global intake, optical_4, Intake_Control
    """Stops intake when a blue ring is detected."""
    while True:
        
        # Check if optical sensor detects a blue ring
        if optical_4.is_near_object() and 200 < optical_4.hue() < 260:
            intake.stop()  # Stop intake immediately
            wait(0.2, SECONDS)

        
        wait(10, MSEC)  # Small delay to reduce CPU usage

def RED_EJECT():
    """Stops intake when a red ring is detected."""
    while True:
        
        # Check if optical sensor detects a red ring
        if optical_4.is_near_object() and 0 < optical_4.hue() < 60:
            intake.stop()  # Stop intake immediately
        wait(10, MSEC)  # Small delay to reduce CPU usage'''







def pid_turn(target_heading, max_velocity):
    global Inertial21, RightMotors, Right_front, LeftMotors, Left_Front
    
    # PID Constants (Adjusted for faster turns) 0.65 --- 0.45
    Kp = 0.65  # Increased proportional gain by 20% for faster response
    Kd = 0.45  # Derivative gain (kept the same for stability)

    # Get current heading (DO NOT RESET IMU)
    start_heading = Inertial21.rotation(DEGREES)
    target = start_heading + (target_heading )  # Adjust for relative turning

    # Initialize PID variables
    prev_error = 0
    import time
    start_time = time.time()  # Timeout start time

    while True:
        # Calculate error (how far from target)
        error = target - Inertial21.rotation(DEGREES)

        # If the error is small enough, stop
        if abs(error) < 7:  # Close enough threshold for accuracy
            break

        # Timeout safety to prevent infinite loops
        if time.time() - start_time > 5:  # Timeout after 5 seconds
            break

        # PID calculations
        derivative = error - prev_error  # Change in error
        prev_error = error  # Store current error

        # Compute PID output
        speed = (Kp * error) + (Kd * derivative)

        # Adaptive speed adjustment to avoid overshooting (lower speed as close to target)
        speed *= max(0.1, 1 - (abs(error) / target_heading) ** 2)

        # Limit speed to max_velocity and reasonable minimum
        speed = max(min(speed, max_velocity), 40)  # Minimum speed = 15%

        # Apply speed to motors for turning
        if error > 0:  # Turn RIGHT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(REVERSE)  # Reverse for right turn
            Right_front.spin(REVERSE)
            LeftMotors.spin(REVERSE)  # Forward for right turn
            Left_Front.spin(REVERSE)
        else:  # Turn LEFT
            RightMotors.set_velocity(speed, PERCENT)
            Right_front.set_velocity(speed, PERCENT)
            LeftMotors.set_velocity(speed, PERCENT)
            Left_Front.set_velocity(speed, PERCENT)

            RightMotors.spin(FORWARD)  # Forward for left turn
            Right_front.spin(FORWARD)
            LeftMotors.spin(FORWARD)  # Reverse for left turn
            Left_Front.spin(FORWARD)

        wait(10, MSEC)  # Small delay for smoother updates

    # Stop motors after reaching target
    RightMotors.stop()
    LeftMotors.stop()
    Right_front.stop()
    Left_Front.stop()























def vexcode_auton_function():
    # Start the autonomous control tasks
    
    auton_task_0 = Thread( onauton_autonomous_0 )
    auton_task_1 = Thread( onauton_autonomous_1 )
    auton_task_2 = Thread( onauton_autonomous_2 )
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the autonomous control tasks
    auton_task_0.stop()
    auton_task_1.stop()
    auton_task_2.stop()
def vexcode_driver_function():
    # Start the driver control tasks
    driver_control_task_0 = Thread( ondriver_drivercontrol_0 )
    driver_control_task_1 = Thread( ondriver_drivercontrol_1 )
    driver_control_task_2 = Thread( ondriver_drivercontrol_2 )
    driver_control_task_3 = Thread( ondriver_drivercontrol_3 )
    driver_control_task_4 = Thread( ondriver_drivercontrol_4 )
    driver_control_task_5 = Thread( ondriver_drivercontrol_5 )
    driver_control_task_6 = Thread( ondriver_drivercontrol_6 )
    driver_control_task_7 = Thread( ondriver_drivercontrol_7 )


    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
    driver_control_task_0.stop()
    driver_control_task_1.stop()
    driver_control_task_2.stop()
    driver_control_task_3.stop()
    driver_control_task_4.stop()
    driver_control_task_5.stop()
    driver_control_task_6.stop()
    driver_control_task_7.stop()

# register the competition functions
competition = Competition( vexcode_driver_function, vexcode_auton_function )

# system event handlers
controller_1.axis2.changed(onevent_controller_1axis2Changed_0)
controller_1.axis3.changed(onevent_controller_1axis3Changed_0)
stop_initialize(onevent_stop_initialize_0)
controller_1.buttonL1.pressed(onevent_controller_1buttonL1_pressed_0)
controller_1.buttonL2.pressed(onevent_controller_1buttonL2_pressed_0)
# add 15ms delay to make sure events are registered correctly.
wait(15, MSEC)

'''ws2 = Thread( when_started2 )
ws3 = Thread( when_started3 )'''
ws4 = Thread( when_started4 )
'''ws5 = Thread( when_started5 )'''
