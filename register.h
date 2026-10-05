#pragma once
#include <stdint.h>
uint32_t baudFromIndex(uint8_t baudIndex);
//   ┌────────────────┬───────────┐
//   │ Register value │ Baud rate │
//   ├────────────────┼───────────┤
//   │ 0              │ 1,000,000 │
//   ├────────────────┼───────────┤
//   │ 1              │ 500,000   │
//   ├────────────────┼───────────┤
//   │ 2              │ 250,000   │
//   ├────────────────┼───────────┤
//   │ 3              │ 128,000   │
//   ├────────────────┼───────────┤
//   │ 4              │ 115,200   │
//   ├────────────────┼───────────┤
//   │ 5              │ 76,800    │
//   ├────────────────┼───────────┤
//   │ 6              │ 57,600    │
//   ├────────────────┼───────────┤
//   │ 7              │ 38,400    │
//   └────────────────┴───────────┘

#define AS_ADDR(name, addr, size) ADDR_##name = addr,
#define AS_SIZE(name, addr, size) SIZE_##name = size,

#define REGISTERS(X)                                                           \
  /* EEPROM (unlock with ADDR_LOCK = 0 before writing) */                      \
  X(FIRMWARE_MAJOR, 0, 1)                                                      \
  X(FIRMWARE_MINOR, 1, 1)                                                      \
  X(MODEL_NUM, 3, 2)                                                           \
  X(ID, 5, 1)                                                                  \
  X(BAUD_RATE, 6, 1)                                                           \
  X(RETURN_DELAY, 7, 1)                                                        \
  X(RESPONSE_LEVEL, 8, 1)                                                      \
  X(MIN_ANGLE_LIMIT, 9, 2)                                                     \
  X(MAX_ANGLE_LIMIT, 11, 2)                                                    \
  X(MAX_TEMPERATURE, 13, 1)                                                    \
  X(MAX_VOLTAGE, 14, 1)                                                        \
  X(MIN_VOLTAGE, 15, 1)                                                        \
  X(MAX_TORQUE, 16, 2)                                                         \
  X(PHASE, 18, 1)                                                              \
  X(UNLOADING_CONDITION, 19, 1)                                                \
  X(LED_ALARM_CONDITION, 20, 1)                                                \
  X(P_GAIN, 21, 1)                                                             \
  X(D_GAIN, 22, 1)                                                             \
  X(I_GAIN, 23, 1)                                                             \
  X(MIN_STARTUP_FORCE, 24, 2)                                                  \
  X(CW_DEAD_ZONE, 26, 1)                                                       \
  X(CCW_DEAD_ZONE, 27, 1)                                                      \
  X(PROTECTION_CURRENT, 28, 2)                                                 \
  X(ANGULAR_RESOLUTION, 30, 1)                                                 \
  X(POSITION_OFFSET, 31, 2) /* sign bit 11 */                                  \
  X(OPERATING_MODE, 33, 1)                                                     \
  X(PROTECTIVE_TORQUE, 34, 1)                                                  \
  X(PROTECTION_TIME, 35, 1)                                                    \
  X(OVERLOAD_TORQUE, 36, 1)                                                    \
  X(SPEED_P_GAIN, 37, 1)                                                       \
  X(OVERCURRENT_TIME, 38, 1)                                                   \
  X(SPEED_I_GAIN, 39, 1)                                                       \
  /* RAM */                                                                    \
  X(TORQUE_ENABLE, 40, 1)                                                      \
  X(ACCELERATION, 41, 1)                                                       \
  X(GOAL_POSITION, 42, 2) /* sign bit 15 */                                    \
  X(GOAL_TIME, 44, 2)     /* sign bit 10, PWM mode */                          \
  X(GOAL_SPEED, 46, 2)    /* sign bit 15 */                                    \
  X(TORQUE_LIMIT, 48, 2)                                                       \
  X(LOCK, 55, 1)                                                               \
  X(PRESENT_POSITION, 56, 2) /* sign bit 15 */                                 \
  X(PRESENT_SPEED, 58, 2)    /* sign bit 15 */                                 \
  X(PRESENT_LOAD, 60, 2)     /* sign bit 10 */                                 \
  X(PRESENT_VOLTAGE, 62, 1)                                                    \
  X(PRESENT_TEMPERATURE, 63, 1)                                                \
  X(STATUS, 65, 1)                                                             \
  X(MOVING, 66, 1)                                                             \
  X(PRESENT_CURRENT, 69, 2)

enum ADDR : uint8_t { REGISTERS(AS_ADDR) };
enum ADDR_SIZE : uint8_t { REGISTERS(AS_SIZE) };

