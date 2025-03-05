import time

def onauton_autonomous_0():
    global turn_heading_velocity_momentum, IntakeF, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False
    

    stop_initialize.broadcast()
    # AUTO SELECT
    intake.set_velocity(90, PERCENT)
    BLUE_RIGHT_RING()
    '''BLUE_LEFT_SAFE()
    BLUE_RIGHT_RING_ELIMS()
    BLUE_RIGHT_RING()
    RED_LEFT_RING()
    RED_RIGHT_SAFE()
    BLUE_LEFT_SAFE()'''








def onauton_autonomous_1():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision, IntakeF, intake
    while intake.is_spinning:
        if intake.velocity(PERCENT) < 10 and intake.efficiency(PERCENT) < 30 and IntakeF == True:
            controller_1.screen.clear_screen()
            controller_1.screen.print(intake.efficiency(PERCENT))
            intake.spin(REVERSE)
            wait(0.3, SECONDS)
            intake.spin(FORWARD)
            wait(1, SECONDS)





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
    pid_drive(57, 100)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    pid_drive(-40, 100)


def RED_LEFT_RING():
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
    pid_drive(38, 100)
    wait(0.5, SECONDS)
    pid_turn(-80, 100)
    wait(0.1, SECONDS)
    pid_drive(19, 100)
    wait(0.5, SECONDS)
    pid_turn(-120, 100)
    Lady_Brown.spin_to_position(300, DEGREES)
    pid_drive(47, 100)

    
def BLUE_RIGHT_RING_ELIMS():
    digital_out_g.set(True)
    digital_out_e.set(True)
    intake.spin(FORWARD)
    pid_drive(62, 100)
    wait(0.5, SECONDS)
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
    pid_turn(160, 100)
    pid_drive(30, 100)
    wait(0.5, SECONDS)
    pid_turn(25, 100)
    wait(0.1, SECONDS)
    pid_drive(7, 100)
    digital_out_e.set(True)
    wait(0.1, SECONDS)
    pid_turn(50, 100)
    wait(0.1, SECONDS)
    digital_out_e.set(False)
    pid_turn(-10, 100)
    wait(0.1, SECONDS)
    pid_drive(13, 100)
    

def BLUE_RIGHT_RING():
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
    pid_drive(38, 100)
    wait(0.5, SECONDS)
    pid_drive(-20, 100)
    pid_turn(40, 100)
    wait(0.1, SECONDS)
    pid_drive(19, 100)
    wait(0.5, SECONDS)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(18, 100)
    wait(0.5, SECONDS)
    pid_drive(-20, 100)
    wait(0.1, SECONDS)
    pid_turn(-120, 100)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(350, DEGREES)
    pid_drive(47, 100)

    
    
def RED_RIGHT_SAFE():
    pid_drive(-35, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(55, 100)
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

    
def BLUE_LEFT_SAFE():
    pid_drive(-35, 100)
    intake.set_velocity(100, PERCENT)
    digital_out_b.set(True)
    pid_turn(55, 100)
    intake.spin(FORWARD)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    wait(0.3, SECONDS)
    pid_turn(-40, 100)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    pid_turn(-20, 100)
    wait(0.1, SECONDS)
    pid_drive(47, 60)
    wait(0.5, SECONDS)
    pid_drive(-20, 60)
    digital_out_g.set(True)
    wait(1, SECONDS)
    intake.stop()
    pid_drive(20, 60)
    wait(0.1, SECONDS)
    pid_turn(-160, 100)
    digital_out_g.set(False)
    pid_turn(160, 40)
    wait(0.1, SECONDS)
    pid_drive(-30, 100)
    wait(0.1, SECONDS)
    pid_turn(90, 70)
    digital_out_b.set(False)
    wait(0.1, SECONDS)
    pid_drive(35, 100)
    pid_turn(180, 100)
    'LADDER VAR'
    '''pid_turn(160, 40)
    wait(0.1, SECONDS)
    pid_drive(-30, 100)
    wait(0.1, SECONDS)
    pid_turn(180, 70)
    wait(0.1, SECONDS)
    pid_drive(30, 100)
    Lady_Brown.spin_to_position(250, DEGREES)
    pid_drive(40, 100)'''



