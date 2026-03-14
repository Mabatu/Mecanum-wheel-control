#include <Arduino.h>
// Library for Bluetooth game controller support
#include <Bluepad32.h>

// These pins control the speed of each wheel
#define RIGHT_FRONT_WHEEL 32
#define RIGHT_BACK_WHEEL 33  
#define LEFT_FRONT_WHEEL 25  
#define LEFT_BACK_WHEEL 26   

// SHIFT REGISTER PINS used to send direction control bits to the motor driver
#define DATA_PIN 18
#define CLOCK_PIN 19
#define LATCH_PIN 23

// These hex values represent motor direction combinations
#define FORWARD 0XAA
#define REVERSE 0X55
#define CLOCKWISE 0X5A
#define COUNTER_CLOCKWISE 0XA5
#define STRAFE_RIGHT 0X69
#define STRAFE_LEFT 0X96
#define DIAGONAL_RIGHT_FORWARD 0X28
#define DIAGONAL_RIGHT_REVERSE 0X14

#define DIAGONAL_LEFT_FORWARD 0X82
#define DIAGONAL_LEFT_REVERSE 0X41

#define NEUTRAL_POINT 4
#define DEAD_ZONE 4
#define MAX_SPEED 1023
#define MIN_SPEED 200
#define DIAGONAL_SPEED 800

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //All motors off -> direction control.
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
  digitalWrite(LATCH_PIN, HIGH);

  int LY = ctl->axisY();
  int RX = ctl->axisRX();
  int L2 = ctl->brake();
  int R2 = ctl->throttle();

  int buttons = ctl->buttons();
  int dPad = ctl->dpad();  // D-pad

  //Controls for left wheels.
  if (LY < NEUTRAL_POINT - DEAD_ZONE) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, FORWARD);
    digitalWrite(LATCH_PIN, HIGH);

    int dutyCycle = map(LY, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FRONT_WHEEL, dutyCycle);
    analogWrite(LEFT_FRONT_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  else if (LY > NEUTRAL_POINT + DEAD_ZONE) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, REVERSE);
    digitalWrite(LATCH_PIN, HIGH);

    int dutyCycle = map(LY, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FRONT_WHEEL, dutyCycle);
    analogWrite(LEFT_FRONT_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  if (RX > NEUTRAL_POINT + DEAD_ZONE) {
    //turn clockwise
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, CLOCKWISE);
    digitalWrite(LATCH_PIN, HIGH);

    //Set speed.
    int dutyCycle = map(RX, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FRONT_WHEEL, dutyCycle);
    analogWrite(LEFT_FRONT_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  else if (RX < NEUTRAL_POINT - DEAD_ZONE) {
    //turn counter-clockwise
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, COUNTER_CLOCKWISE);
    digitalWrite(LATCH_PIN, HIGH);

    //Set speed.
    int dutyCycle = map(RX, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FRONT_WHEEL, dutyCycle);
    analogWrite(LEFT_FRONT_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }
  if (L2 > 4) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, STRAFE_LEFT);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, L2);
    analogWrite(LEFT_FRONT_WHEEL, L2);
    analogWrite(RIGHT_BACK_WHEEL, L2);
    analogWrite(LEFT_BACK_WHEEL, L2);
  }
  if (R2 > 4) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, STRAFE_RIGHT);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, R2);
    analogWrite(LEFT_FRONT_WHEEL, R2);
    analogWrite(RIGHT_BACK_WHEEL, R2);
    analogWrite(LEFT_BACK_WHEEL, R2);
  }
  if (buttons == 0x0020) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, DIAGONAL_RIGHT_FORWARD);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, 0);
    analogWrite(LEFT_FRONT_WHEEL, DIAGONAL_SPEED);
    analogWrite(RIGHT_BACK_WHEEL, DIAGONAL_SPEED);
    analogWrite(LEFT_BACK_WHEEL, 0);
  }
  if (buttons == 0x0008) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, DIAGONAL_RIGHT_REVERSE);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, 0);
    analogWrite(LEFT_FRONT_WHEEL, DIAGONAL_SPEED);
    analogWrite(RIGHT_BACK_WHEEL, DIAGONAL_SPEED);
    analogWrite(LEFT_BACK_WHEEL, 0);
  }
  if (buttons == 0x0010) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, DIAGONAL_LEFT_FORWARD);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, DIAGONAL_SPEED);
    analogWrite(LEFT_FRONT_WHEEL, 0);
    analogWrite(RIGHT_BACK_WHEEL, 0);
    analogWrite(LEFT_BACK_WHEEL, DIAGONAL_SPEED);
  }
  if (dPad == 0x01) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, DIAGONAL_LEFT_REVERSE);
    digitalWrite(LATCH_PIN, HIGH);

    analogWrite(RIGHT_FRONT_WHEEL, DIAGONAL_SPEED);
    analogWrite(LEFT_FRONT_WHEEL, 0);
    analogWrite(RIGHT_BACK_WHEEL, 0);
    analogWrite(LEFT_BACK_WHEEL, DIAGONAL_SPEED);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);

  pinMode(RIGHT_FRONT_WHEEL, OUTPUT);
  pinMode(RIGHT_BACK_WHEEL, OUTPUT);
  pinMode(LEFT_FRONT_WHEEL, OUTPUT);
  pinMode(LEFT_BACK_WHEEL, OUTPUT);

  analogWriteFrequency(25000);
  analogWriteResolution(10);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
}
