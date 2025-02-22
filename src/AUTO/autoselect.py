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




