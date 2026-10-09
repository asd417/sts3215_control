#include "src/driver.h"
#include "src/platforms/platform.h"
#include <stdint.h>

const uint8_t BUTTON_PIN = 3; // button to GND, internal pull-up: pressed = LOW
const uint8_t LED_PIN = LED_BUILTIN; // on-board "L" LED (pin 13): HIGH = on

HardwareSerial *m;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  //openPort(1000000, m);
  //int servoerror;
  //ping(m,1,&servoerror);
}

void loop() {
  digitalWrite(LED_PIN, digitalRead(BUTTON_PIN) == LOW ? HIGH : LOW);
  //digitalWrite(LED_PIN, HIGH); delay(500);
  //digitalWrite(LED_PIN, LOW);  delay(500);
}
