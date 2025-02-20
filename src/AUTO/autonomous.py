import time

def onauton_autonomous_0():
    global turn_heading_velocity_momentum, Forward_PID_Distance_Max_Speed, message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    # GLOBAL FINAL AUTONOMOUS SELECTION
    remote_control_code_enabled = False

    stop_initialize.broadcast()
    # AUTO SELECT
    intake.set_velocity(80, PERCENT)
    SKILLS_PROGRAM()


    

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
    pid_drive(30, 100)

    wait(1, SECONDS)
    pid_turn(-85, 100)
    pid_drive(20, 100)
    wait(1, SECONDS)
    pid_drive(25, 100)
    wait(1, SECONDS)
    pid_turn(-90, 100)
    pid_drive(-20, 100)
    digital_out_b.set(False)
    wait(0.5,SECONDS)
    intake.spin(REVERSE)
    wait(0.5,SECONDS)
    pid_drive(70, 100)
    intake.spin(FORWARD)
    pid_turn(200,100)
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
    wait(1,SECONDS)
    pid_drive(25, 100)
    pid_turn(-90,100)





    '''wait(0.5, SECONDS)
    pid_turn(104, 100)
    pid_drive(71, 100)
    wait(0.5, SECONDS)
    pid_drive(-15, 100)
    wait(0.5, SECONDS)
    pid_turn(185, 100)
    pid_drive(90, 100) 
    pid_turn(40, 100)
    pid_drive(40, 40)
    pid_drive(-40, 100)
    pid_turn(-90, 100)
    pid_drive(60, 100)
    pid_turn(-20, 100)
    pid_drive(80, 100)
    pid_turn(-100, 100)'''



    



def RED_LEFT_RING():
    color_sensing_BLUE_thread = Thread(BLUE_EJECT)
    pid_drive(7, 80)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(350, DEGREES)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(0, DEGREES)
    wait(0.1, SECONDS)
    pid_turn(1, 100)
    wait(0.1, SECONDS)
    pid_drive(-30, 80)
    wait(0.1, SECONDS)
    digital_out_b.set(True)
    wait(0.1, SECONDS)
    pid_turn(110, 100)
    intake.spin(FORWARD)
    pid_drive(30, 80)
    wait(1, SECONDS)
    pid_drive(-10, 80)
    pid_turn(10, 100)
    pid_drive(20, 80)
    wait(1, SECONDS)
    pid_drive(-10, 80)
    wait(0.1, SECONDS)
    pid_turn(100, 100)
    wait(0.1, SECONDS)
    pid_drive(15, 80)
    wait(1, SECONDS)
    intake.stop()
    wait(0.1, SECONDS)
    pid_turn(160, 100)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(200, DEGREES)
    wait(0.1, SECONDS)
    pid_drive(30, 80)


def RED_RIGHT_GOAL_RUSH():
    color_sensing_BLUE_thread = Thread(BLUE_EJECT)
    pid_drive(70, 80)
    wait(0.1, SECONDS)
    pid_turn(20, 100)
    wait(0.1, SECONDS)
    pid_drive(10, 80)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    pid_turn(-20, 100)
    wait(0.1, SECONDS)
    pid_drive(-30, 80)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    pid_turn(180, 100)
    wait(0.1, SECONDS)
    pid_drive(-15, 80)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    intake.spin(FORWARD)
    wait(1, SECONDS)
    digital_out_g.set(False)
    wait(0.1, SECONDS)
    pid_drive(20, 80)
    wait(0.1, SECONDS)
    pid_turn(60, 100)
    wait(0.1, SECONDS)
    pid_drive(-40, 80)
    wait(0.1, SECONDS)
    digital_out_g.set(True)
    wait(0.1, SECONDS)
    pid_turn(40, 100)
    wait(0.1, SECONDS)
    pid_drive(40, 80)
    wait(0.1, SECONDS)
    pid_turn(180, 100)
    wait(0.1, SECONDS)
    Lady_Brown.spin_to_position(200, DEGREES)
    wait(0.1, SECONDS)
    pid_drive(20, 80)
    


    '''pid_drive(-50, 80)
    digital_out_b.set(True)
    intake.spin(FORWARD)
    pid_turn(180, 100)
    wait(0.5, SECONDS)
    Lady_Brown.spin_to_position(200, DEGREES)'''
    
def BLUE_LEFT_RING():
    color_sensing_RED_thread = Thread(RED_EJECT)

def BLUE_RIGHT_GOAL_RUSH():
    color_sensing_RED_thread = Thread(RED_EJECT)


