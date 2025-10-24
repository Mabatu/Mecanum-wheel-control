#include <Bluepad32.h>

#define RIGHT_FORWARD_WHEEL 32
#define RIGHT_BACK_WHEEL 19
#define LEFT_FORWARD_WHEEL 26
#define LEFT_BACK_WHEEL 25

#define DATA_PIN 18
#define CLOCK_PIN 33
#define LATCH_PIN 23
#define OUTPUT_ENABLE 27

#define FORWARD 0XA9
#define REVERSE 0X56
#define CLOCKWISE 0XD1
#define COUNTER_CLOCKWISE 0X2E
#define STRAIF_RIGHT 0X1B
#define STRIF_LEFT 0XE4

#define NEUTRAL_POINT 4
#define DEAD_ZONE 4
#define MAX_SPEED 1023
#define MIN_SPEED 200

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

uint8_t set_direction(int right_forward_wheel, int right_back_wheel, int left_forward_wheel, int left_back_wheel) {
  uint8_t direction_buffer = 0;
  direction_buffer |= right_forward_wheel << 6;
  direction_buffer |= right_back_wheel << 4;
  direction_buffer |= left_forward_wheel << 2;
  direction_buffer |= left_back_wheel;
  return direction_buffer;
}

void processGamepad(ControllerPtr ctl) {
  //All motors off -> direction control.
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
  digitalWrite(LATCH_PIN, HIGH);

  /*analogWrite(leftWheels, 0);
  analogWrite(rightWheels, 0);*/

  int LY = ctl->axisY();
  int RX = ctl->axisRX();
  int L2 = ctl->brake();
  int R2 = ctl->throttle();
  int buttons = ctl->buttons();

  //Controls for left wheels.
  if (LY < NEUTRAL_POINT - DEAD_ZONE) {
    uint8_t direction = set_direction(FORWARD, FORWARD, FORWARD, FORWARD);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, direction);
    digitalWrite(LATCH_PIN, HIGH);
    Serial.println(direction, BIN);

    int dutyCycle = map(LY, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FORWARD_WHEEL, dutyCycle);
    analogWrite(LEFT_FORWARD_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  else if (LY > NEUTRAL_POINT + DEAD_ZONE) {
    uint8_t direction = set_direction(REVERSE, REVERSE, REVERSE, REVERSE);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, direction);
    digitalWrite(LATCH_PIN, HIGH);
    Serial.println(direction, BIN);

    //Set speed.
    int dutyCycle = map(LY, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FORWARD_WHEEL, dutyCycle);
    analogWrite(LEFT_FORWARD_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  if (RX > NEUTRAL_POINT + DEAD_ZONE) {
    //turn clockwise
    uint8_t direction = set_direction(REVERSE, REVERSE, FORWARD, FORWARD);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, direction);
    digitalWrite(LATCH_PIN, HIGH);
    Serial.println(direction, BIN);

    //Set speed.
    int dutyCycle = map(RX, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FORWARD_WHEEL, dutyCycle);
    analogWrite(LEFT_FORWARD_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }

  else if (RX < NEUTRAL_POINT - DEAD_ZONE) {
    //turn counter-clockwise
    uint8_t direction = set_direction(FORWARD, FORWARD, REVERSE, REVERSE);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, direction);
    digitalWrite(LATCH_PIN, HIGH);
    Serial.println(direction, BIN);

    //Set speed.
    int dutyCycle = map(RX, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(RIGHT_FORWARD_WHEEL, dutyCycle);
    analogWrite(LEFT_FORWARD_WHEEL, dutyCycle);
    analogWrite(RIGHT_BACK_WHEEL, dutyCycle);
    analogWrite(LEFT_BACK_WHEEL, dutyCycle);
  }
  /*if (L2 > 4) {
    digitalWrite(RightWheelsForward, LOW);
    digitalWrite(RightWheelsReverse, HIGH);
    digitalWrite(LeftWheelsForward, LOW);
    digitalWrite(LeftWheelsReverse, HIGH);

    int dutyCycle = map(L2, 0, 1023, MIN_SPEED, MAX_SPEED);
    analogWrite(rightWheels, dutyCycle);
    analogWrite(leftWheels, dutyCycle);
  }
  if (R2 > 4) {
    digitalWrite(RightWheelsForward, HIGH);
    digitalWrite(RightWheelsReverse, LOW);
    digitalWrite(LeftWheelsForward, HIGH);
    digitalWrite(LeftWheelsReverse, LOW);

    int dutyCycle = map(R2, 0, 1023, MIN_SPEED, MAX_SPEED);
    analogWrite(rightWheels, dutyCycle);
    analogWrite(leftWheels, dutyCycle);
  }*/
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

  //PWM pins to control speed of wheels
  /*pinMode(rightWheels, OUTPUT);

  pinMode(leftWheels, OUTPUT);*/


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
