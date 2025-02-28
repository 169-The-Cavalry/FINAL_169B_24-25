import time

def onauton_autonomous_0():
    global turn_heading_velocity_momentum, IntakeF, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False
    

    stop_initialize.broadcast()
    # AUTO SELECT
    intake.set_velocity(100, PERCENT)

    '''BLUE STAKE SIDE'''
    pid_drive(-31, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(55, 100)
    intake.spin(FORWARD)
    pid_drive(26, 100)
    wait(0.3, SECONDS)
    pid_turn(-40, 100)
    pid_drive(30, 100)
    pid_turn(-30, 100)
    digital_out_g.set(True)
    pid_drive(30, 100)
    pid_turn(-100, 60)
    digital_out_g.set(False)
    pid_turn(100, 70)
    pid_drive(7, 100)
    wait(0.3, SECONDS)
   




    '''RED STAKE SIDE'''
    '''pid_drive(-28, 100)
    digital_out_b.set(True)
    pid_turn(-60, 100)
    intake.spin(FORWARD)
    pid_drive(26, 100)
    wait(0.3, SECONDS)
    pid_turn(-60, 100)
    digital_out_e.set(True)
    pid_drive(30, 100)'''

    '''pid_drive(12.5, 100)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-49, 100)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    pid_turn(170, 100)
    pid_drive(20, 100)
    pid_drive_curve(-24, 70, 100)
    intake.spin(FORWARD)
    pid_drive(10, 100)
    wait(0.3, SECONDS)
    pid_drive(10, 100)'''
    

    '''REDSIDE'''
    '''pid_drive(12.5, 100)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-49, 60)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    pid_turn(150, 100)
    intake.spin(FORWARD)
    pid_drive(20, 100)
    pid_drive(15, 100)
    wait(0.5, SECONDS)
    pid_turn(-80, 100)
    pid_drive(15, 60)
    wait(0.5, SECONDS)
    pid_turn(-120, 100)
    Lady_Brown.spin_to_position(300, DEGREES)
    pid_drive(37, 100)'''



    '''BLUE RING SIDE'''
    '''pid_drive(12.5, 100)
    Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    pid_drive(-49, 60)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    pid_turn(-150, 100)
    intake.spin(FORWARD)
    pid_drive(20, 100)
    pid_drive(18, 100)
    wait(0.5, SECONDS)
    pid_turn(70, 100)
    pid_drive(13, 60)
    wait(0.5, SECONDS)
    pid_turn(140, 100)
    Lady_Brown.spin_to_position(300, DEGREES)
    pid_drive(37, 100)'''




def onauton_autonomous_1():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF, intake
    while intake.is_spinning and IntakeF==True:
        if intake.velocity(PERCENT) == 0 :
            brain.screen.print("STUCK")
            intake.spin(REVERSE)
            wait(0.3, SECONDS)
            intake.spin(FORWARD)
            wait(2, SECONDS)



    '''Lady_Brown.spin_to_position(360, DEGREES)
    wait(0.0.1, SECONDS)
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

    

def SKILLS_PROGRAM():
    Lady_Brown.spin_to_position(350, DEGREES)
    wait(0.0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    wait(0.5, SECONDS)
    pid_turn(-20, 100)
    pid_drive(-19, 100)
    wait(0.5, SECONDS)
    digital_out_b.set(True)
    pid_turn(-120, 100)
    intake.spin(FORWARD)
    pid_drive(30, 100)
    wait(0.1, SECONDS)
    pid_turn(-77, 100)
    pid_drive(25, 100)

    wait(0.1, SECONDS)
    pid_turn(-85, 100)
    pid_drive(20, 100)
    wait(0.1, SECONDS)
    pid_drive(25, 100)
    wait(0.1, SECONDS)
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


    wait(0.1,SECONDS)
    pid_turn(80,100)
    pid_drive(25, 100)
    wait(0.1,SECONDS)
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
    wait(0.0.1, SECONDS)
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
    wait(0.1, SECONDS)

    
def BLUE_LEFT_SAFE():
    pid_drive(-38, 100)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_turn(50, 100)
    pid_drive(25, 100)
    wait(0.1, SECONDS)'''


