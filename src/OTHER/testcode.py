def anti_jam():
        """ Detects intake stall and reverses to clear jams. """
        jam_timer = 0
         
        while Intake_running:
            if intake.velocity == 0: # Detects jam if motor is not moving
                jam_timer += 1
                if jam_timer > 10:  # If stalled for 10 iterations (~50ms each)
                    intake.spin(REVERSE)  # Reverse intake
                    wait(500, MSEC)  # Allow jam clearing
                    intake.spin(FORWARD)  # Resume normal operation
                    jam_timer = 0  # Reset jam detection timer
            else:
                jam_timer = 0  # Reset if intake moves normally
            wait(5, MSEC)

    while True:
        anti_jam()
        wait(10, MSEC)  # Reduce unnecessary CPU usage

def BLUE_EJECT():
    """Stops intake when a blue ring is detected."""
    while True:
        
        # Check if optical sensor detects a blue ring
        if optical_4.is_near_object() and 200 < optical_4.hue() < 260:
            intake.stop()  # Stop intake immediately
        
        wait(10, MSEC)  # Small delay to reduce CPU usage

def RED_EJECT():
    """Stops intake when a red ring is detected."""
    while True:
        
        # Check if optical sensor detects a red ring
        if optical_4.is_near_object() and 0 < optical_4.hue() < 60:
            intake.stop()  # Stop intake immediately
        
        wait(10, MSEC)  # Small delay to reduce CPU usage





