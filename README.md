# ESP32 Mecanum-Wheel Robot
A Bluetooth-controlled omnidirectional mobile robot built around an ESP32. The robot uses four mecanum wheels to achieve forward/backward motion, lateral strafing, diagonal translation, and rotation in place.
The system combines embedded motor control, mecanum-wheel kinematics, Bluetooth controller input, PWM speed control, and a split-power electrical architecture.

 **Controller:** WEMOS Lolin D32 / ESP32
 
 **Input:** PlayStation 4 DualShock Controller via Bluetooth
 
 **Drive:** 4-wheel mecanum
 
 **Motor drivers:** 2 × L298N
 
 **GPIO expansion:** 74HC595 shift register
 
 **Firmware:** C++
 
## What the robot can do
- Forward and backward translation

- Left and right strafing

- Diagonal movement

- Rotation in place

- Variable-speed control from the PS4 analog sticks/triggers
# Microcontroller and Power Architecture
<img width="294" height="718" alt="mecanum_block diagram drawio" src="https://github.com/user-attachments/assets/32cb9420-f4c1-4ed8-9472-22eea626bb83" />

The robot is powered by a WEMOS Lolin D32 ESP32 Development Board, chosen for its integrated battery management system and wireless capabilities (Wi-Fi and Bluetooth). A split-power architecture was implemented, with one lithium-ion battery for the ESP32 and two for the motors. This separation prevents voltage drops during high-power demands, ensuring stable performance.
# Motor Control Architecture
Four DC motors require multiple direction-control signals. Rather than dedicating eight ESP32 GPIO pins to motor direction, the project uses a 74HC595 serial-in, parallel-out shift register.

The ESP32 interfaces with the shift register using three control lines:

Motor speed remains under direct ESP32 PWM control, while the shift register handles the digital direction states.

This reduced the GPIO requirements while keeping independent directional control of all four motors.
# Controller Interface
Control is achieved via a Bluetooth-connected PlayStation 4 DualShock Controller using the Bluepad32 library. The controller inputs map to intuitive omnidirectional commands, enabling smooth operation through the analog sticks and triggers.
