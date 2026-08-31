This directory contains the main control firmware for the robot.

The code is kept as multiple versions to show the progression of the control system as new capabilities were developed.

Versions
`mecanum_control_v1.ino`

The first working control implementation.

It established the basic control pipeline:

Bluetooth gamepad input
ESP32 motor control
PWM speed control
74HC595 shift-register direction control
Forward and reverse movement
Clockwise and counter-clockwise rotation

`mecanum_control_v2.ino`

An expanded version of the controller that adds:

Left and right strafing
Diagonal movement
Additional controller inputs
Improved movement control (removed `map` function)


This version represents the transition from basic drive control toward taking advantage of the robot's mecanum drivetrain.