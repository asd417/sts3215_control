// Arduino smoke test: opens the port, pings the servo once, and turns the
// on-board LED on if it answered. Needs this repo installed as a library.
#include <sts3215.h>

const uint8_t SERVO_ID = 1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  PORT_HANDLE port = nullptr;
  int servoError = 0;
  if (!openPort(1000000, &port) && !ping(port, SERVO_ID, &servoError) && !servoError)
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {}
