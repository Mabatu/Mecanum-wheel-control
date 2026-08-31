# 74HC595 Shift Register Test

This test verifies the operation of the 74HC595 shift register used for motor direction control.

A single HIGH bit is shifted through all eight outputs, allowing each output to be checked individually. The test also verifies the ESP32-to-shift-register connections for:

DATA
CLOCK
LATCH
