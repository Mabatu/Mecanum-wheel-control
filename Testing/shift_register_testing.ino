//shift register pins
#define DATA_PIN 19
#define CLOCK_PIN 18
#define LATCH_PIN 23

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 1 << i);
    digitalWrite(LATCH_PIN, HIGH);
    delay(500);
  }
}
