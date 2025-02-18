

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





