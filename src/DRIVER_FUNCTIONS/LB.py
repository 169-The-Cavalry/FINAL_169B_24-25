

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


