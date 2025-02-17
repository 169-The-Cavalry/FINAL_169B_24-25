# GitHub Copilot

## Project Overview
This project is designed for a VEX robotics competition, utilizing Python for controlling the robot's various functions. The code is structured to handle autonomous and driver control modes, with PID control for precise movements.

## Table of Contents
- [Autonomous Mode Options](#autonomous-mode-options)
- [Display Functions](#display-functions)
- [Event Handlers](#event-handlers)
- [Initialization Functions](#initialization-functions)
- [PID Control Functions](#pid-control-functions)
- [Driver Control Functions](#driver-control-functions)
- [Speedometer Display](#speedometer-display)
- [Competition Functions](#competition-functions)

## Autonomous Mode Options
This section defines the different autonomous modes available for the robot. The `auto_modes` list contains various pre-defined autonomous routines that the robot can execute. The `AutoSelect` variable is used to keep track of the currently selected autonomous mode.

## Display Functions
These functions handle the display updates on the VEX controller screen, including animations and mode selections.

- `update_auto_display()`: Updates the controller screen with the currently selected autonomous mode and provides navigation instructions.
- `fancy_scroll_effect()`: Creates a flashing effect when switching modes.
- `smooth_scroll_effect()`: Provides a smooth scrolling text effect when switching modes.

## Event Handlers
These functions handle button press events to switch between different autonomous modes.

- `onevent_controller_1buttonL1_pressed_0()`: Scrolls to the next autonomous mode when the L1 button is pressed.
- `onevent_controller_1buttonL2_pressed_0()`: Scrolls to the previous autonomous mode when the L2 button is pressed.

## Initialization Functions
These functions initialize various components and settings of the robot.

- `when_started5()`: Initializes the auto selector with a visually clean display.
- `when_started4()`: Calibrates and initializes sensors and motors.
- `onevent_stop_initialize_0()`: Initializes motor settings.

## PID Control Functions
These functions implement PID control for precise movements and turns.

- `Move_In_direction_Degree_Speed(degree, speed)`: Moves in a specific direction with a given speed.
- `Forward_PID_Distance_Max_Speed(distance, max_speed)`: Moves forward a specific distance with PID control.
- `pid_drive(distance_inches, max_velocity_percent, timeout=20.0)`: Drives forward with PID control and heading correction.
- `pid_turn(target_heading, max_velocity)`: Turns to a specific heading with PID control.

## Driver Control Functions
These functions handle the driver control period, including joystick input and motor control.

- `ondriver_drivercontrol_1()`: Handles joystick input for driving.
- `ondriver_drivercontrol_2()`: Handles button input for clamp control.
- `ondriver_drivercontrol_3()`: Handles button input for sweeper control.
- `ondriver_drivercontrol_0()`: Handles button input for intake control.
- `ondriver_drivercontrol_4()`: Handles button input for Lady Brown control.

## Speedometer Display
This function draws a speedometer on the VEX brain screen to display motor speed.

- `draw_speedometer()`: Draws a speedometer on the brain screen.

## Competition Functions
These functions manage the autonomous and driver control periods during the competition.

- `vexcode_auton_function()`: Starts the autonomous control tasks.
- `vexcode_driver_function()`: Starts the driver control tasks.

## System Event Handlers
These handlers manage system events and register competition functions.

- `competition = Competition(vexcode_driver_function, vexcode_auton_function)`: Registers the competition functions.
- `stop_initialize(onevent_stop_initialize_0)`: Registers the stop initialize event handler.
- `ws4 = Thread(when_started4)`: Starts the initialization thread.

---

This EXPLAINATION provides an overview of each section of the code, explaining its purpose and functionality.
