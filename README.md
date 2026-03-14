# Omnidirectional Robot Platform Development
This documentation details the development of a Bluetooth-controlled omnidirectional robot using the ESP32 microcontroller. The project evolved from a simple differential-drive model to a versatile system capable of complex movements, including strafing sideways and diagonal movement.
Initially, the robot used a basic differential drive with four regular wheels powered by a single L298N Motor Driver Module, enabling only forward/reverse motion and basic turning. To improve agility, Mecanum wheels and an additional motor driver were added, allowing for true omnidirectional movement and enhanced control.
# Microcontroller and Power Architecture
The robot is powered by a WEMOS Lolin D32 ESP32 Development Board, chosen for its integrated battery management system and wireless capabilities (Wi-Fi and Bluetooth). A split-power architecture was implemented, with one lithium-ion battery for the ESP32 and two for the motors. This separation prevents voltage drops during high-power demands, ensuring stable performance.
# Motor Control Architecture
To optimize GPIO usage, a 74HC595 Shift Register was employed, reducing the need for eight GPIO pins for motor control. Four pins generate PWM signals for motor speed, while three pins control the shift register to manage directional outputs, allowing independent control of each motor.
# Controller Interface
Control is achieved via a Bluetooth-connected PlayStation 4 DualShock Controller using the Bluepad32 library. The controller inputs map to intuitive omnidirectional commands, enabling smooth operation through the analog sticks and triggers.
