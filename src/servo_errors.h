#pragma once
#include <stdint.h>
//   ┌─────┬───────┬──────────────┬───────────────────────────────────────────────┬────────────────────────────────────┐
//   │ Bit │ Value │     Name     │                  Fires when                   │            Usual cause             │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 0   │ 1     │ Voltage      │ supply is outside Min/Max voltage (15 and 14) │ weak or sagging supply, or         │
//   │     │       │              │                                               │ back-EMF from a fast stop          │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 1   │ 2     │ Angle /      │ the position sensor reports a fault, or the   │ magnetic encoder problem, or       │
//   │     │       │ sensor       │ goal is outside the angle limits (9 and 11)   │ commanding past the limits         │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 2   │ 4     │ Overheat     │ Present temperature (63) exceeds Max          │ sustained load, or holding a       │
//   │     │       │              │ temperature (13)                              │ stalled position                   │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 3   │ 8     │ Overcurrent  │ current exceeds Protection current (28) for   │ jam, short, or a sudden large step │
//   │     │       │              │ longer than Overcurrent time (38)             │                                    │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 4   │ 16    │ —            │ unused                                        │                                    │
//   ├─────┼───────┼──────────────┼───────────────────────────────────────────────┼────────────────────────────────────┤
//   │ 5   │ 32    │ Overload     │ torque stays above Overload torque (36) for   │ fighting an obstruction, or an     │
//   │     │       │              │ longer than Protection time (35)              │ under-sized servo for the load     │
//   └─────┴───────┴──────────────┴───────────────────────────────────────────────┴────────────────────────────────────┘

constexpr uint8_t SERVO_ERROR_ANY      = 0b101111;
constexpr uint8_t SERVO_ERROR_VOLTAGE  = 0b000001;
constexpr uint8_t SERVO_ERROR_ANGLE    = 0b000010;
constexpr uint8_t SERVO_ERROR_TEMP     = 0b000100;
constexpr uint8_t SERVO_ERROR_CURRENT  = 0b001000;
constexpr uint8_t SERVO_ERROR_OVERLOAD = 0b100000;

#ifndef __AVR__ // not enough ram. just use error codes directly
const char* servoErrorToString(int error);
#endif