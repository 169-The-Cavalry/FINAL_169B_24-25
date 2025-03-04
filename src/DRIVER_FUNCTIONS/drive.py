
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
        
def ondriver_drivercontrol_5():
    global message1, forward_move, Back_move, Stop, turn_right, turn, calibrate, stop_initialize, Auto_Stop, turn_left, start_auto, intake_forward, intake_backward, DOon, LB, DOon2, Blue, Red, Intake_Control, Intake_running, myVariable, volocity, Right_Axis, Left_Axis, IntakeStake, Degree, pi, movement, distance1, time1, rot, turn1, LadyBrown_Up, LadyBrown_score, LadyBrown, Right_turn, Left_turn, DriveState, start, Next, dos, tog, error, output, Kp, Ki, Kd, Dellay, Distance_travled, imput, Proportional, integral, derivitive, direction, Previus_error, AutoSelect, X_Start, Y_Start, Y_End, X_End, Angle, Distnce2, Distance2, Turn_Angle, remote_control_code_enabled, vexcode_brain_precision, vexcode_console_precision, vexcode_controller_1_precision
    global intake, optical_4, Intake_Control, BLUE, RED
    """Stops intake when a blue ring is detected."""
    while True:
            # Check if optical sensor detects a blue ring (DEFAULTS TO BLUE)
        while BLUE:
            if  230 > optical_4.hue() > 200:
                wait(0.13, SECONDS)
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                brain.screen.clear_screen()
                brain.screen.print("Blue")
                wait(0.5, SECONDS)
                Intake_Control = True
        while RED:
            # Check if optical sensor detects a red ring
            if  230 > optical_4.hue() > 200:
                wait(0.13, SECONDS)
                Intake_Control = False
                intake.stop()  # Stop intake immediately
                brain.screen.clear_screen()
                brain.screen.print("Red")
                wait(0.5, SECONDS)
                Intake_Control = True