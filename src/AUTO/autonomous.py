import time

def onauton_autonomous_0():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False

    stop_initialize.broadcast()
    # AUTO SELECT
    intake.set_velocity(100, PERCENT)
    pid_drive(9, 60)
    wait(1, SECONDS)
    Lady_Brown.spin_to_position(350, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    wait(1, SECONDS)
    pid_drive(-46, 60)
    wait(1, SECONDS)
    digital_out_b.set(True)
    wait(1, SECONDS)
    pid_turn(-70, 100)
    wait(1, SECONDS)
    pid_turn(-60, 100)
    wait(1, SECONDS)
    intake.spin(FORWARD)
    wait(1, SECONDS)
    pid_drive(25, 60)
    wait(1, SECONDS)
    pid_drive(-10, 60)
    wait(1, SECONDS)
    pid_turn(-20, 100)
    wait(1, SECONDS)
    pid_drive(10, 60)
    wait(1, SECONDS)
    pid_drive(-10, 60)
    wait(1, SECONDS)
    pid_turn(-50, 100)
    wait(1, SECONDS)
    pid_drive(15, 60)


    '''wait(1, SECONDS)
    pid_turn(70, 100)
    wait(1, SECONDS)
    pid_turn(70, 100)
    wait(1, SECONDS)
    pid_turn(70, 100)
    wait(1, SECONDS)
    pid_turn(70, 100)
    wait(1, SECONDS)
    pid_turn(70, 100)
    wait(1, SECONDS)
    pid_turn(140, 100)'''

def onauton_autonomous_1():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF
    while not IntakeF:
        if intake.velocity(PERCENT) < 10:
            intake.spin(REVERSE)
            wait(0.5, SECONDS)
            intake.spin(FORWARD)

def INTAKEF():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF
    IntakeF = True
    while IntakeF:
        intake.spin(FORWARD)

    '''Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)'''
    


    '''RED_LEFT_RING()'''
    '''BLUE_LEFT_SAFE()'''
    '''RED_RIGHT_SAFE()'''
    '''BLUE_RIGHT_RING()'''
    '''run_autonomous()'''



'''def run_autonomous():
    global AutoSelect
    """Executes the selected autonomous routine."""
    if AutoSelect == 0:
        no_auto()
    elif AutoSelect == 1:
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

    

def SKILLS_PROGRAM():
    Lady_Brown.spin_to_position(350, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    wait(0.5, SECONDS)
    pid_turn(-20, 100)
    pid_drive(-19, 100)
    wait(0.5, SECONDS)
    digital_out_b.set(True)
    pid_turn(-120, 100)
    intake.spin(FORWARD)
    pid_drive(30, 100)
    wait(1, SECONDS)
    pid_turn(-77, 100)
    pid_drive(25, 100)

    wait(1, SECONDS)
    pid_turn(-85, 100)
    pid_drive(20, 100)
    wait(1, SECONDS)
    pid_drive(25, 100)
    wait(1, SECONDS)
    pid_turn(-90, 100)
    pid_drive(-18, 100)
    digital_out_b.set(False)
    wait(0.5,SECONDS)
    intake.spin(REVERSE)
    wait(0.5,SECONDS)
    pid_drive(70, 100)
    intake.spin(FORWARD)
    pid_turn(190,100)
    pid_drive(-25, 100)
    digital_out_b.set(True)
    pid_turn(100,100)
    pid_drive(25, 100)


    wait(1,SECONDS)
    pid_turn(80,100)
    pid_drive(25, 100)
    wait(1,SECONDS)
    pid_turn(90,100)
    pid_drive(25, 100)
    wait(2,SECONDS)
    pid_drive(25, 100)
    pid_turn(100,100)
    pid_drive(-25, 100)
    digital_out_b.set(False)


def RED_LEFT_RING():
    

def BLUE_RIGHT_RING():
    pid_drive(11, 100)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-43, 100)
    digital_out_b.set(True)
    pid_turn(-103, 100)
    intake.spin(FORWARD)
    pid_drive(30, 100)
    
    
def RED_RIGHT_SAFE():
    pid_drive(-38, 100)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_turn(-50, 100)
    pid_drive(25, 100)
    wait(1, SECONDS)

    
def BLUE_LEFT_SAFE():
    pid_drive(-38, 100)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_turn(50, 100)
    pid_drive(25, 100)
    wait(1, SECONDS)'''


