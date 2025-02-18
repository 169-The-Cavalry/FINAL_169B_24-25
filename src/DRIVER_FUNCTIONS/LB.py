

def ondriver_drivercontrol_4():
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
            Lady_Brown.set_velocity(40, PERCENT)
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

        wait(5, MSEC)  # Reduce CPU usage

