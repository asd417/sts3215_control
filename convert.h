#pragma once
#include <stdint.h>

constexpr uint8_t SIGN_PRESENT = 15;
constexpr uint8_t SIGN_SPEED = 15;
constexpr uint8_t SIGN_LOAD = 10;
constexpr uint8_t SIGN_GOAL_SPEED_WHEEL = 15;
constexpr uint8_t SIGN_GOAL_POS = 15;
constexpr uint8_t SIGN_GOAL_TIME = 10;
constexpr uint8_t SIGN_POS_OFFSET = 11;

uint8_t makeLowByte(uint16_t in);
uint8_t makeHighByte(uint16_t in);

uint16_t to_scs(int16_t v, uint8_t sign);
int16_t from_scs(uint16_t v, uint8_t sign);

#define MAKE_TOLOHI_HEADER(name)                                               \
  void name##ToLoHi(int16_t in, uint8_t *lo, uint8_t *hi);
#define MAKE_FROMLOHI_HEADER(name)                                             \
  int16_t name##FromLoHi(uint8_t lo, uint8_t hi);

MAKE_TOLOHI_HEADER(positionOffset)   // positionOffsetToLoHi
MAKE_FROMLOHI_HEADER(positionOffset) // positionOffsetFromLoHi
MAKE_TOLOHI_HEADER(goalPosition)     // goalPositionToLoHi
MAKE_FROMLOHI_HEADER(goalPosition)   // goalPositionFromLoHi

// only used in pwm mode
MAKE_TOLOHI_HEADER(goalTime)   // goalTimeToLoHi
MAKE_FROMLOHI_HEADER(goalTime) // goalTimeFromLoHi

MAKE_TOLOHI_HEADER(goalSpeed)   // goalSpeedToLoHi
MAKE_FROMLOHI_HEADER(goalSpeed) // goalSpeedFromLoHi

MAKE_FROMLOHI_HEADER(presentPosition) // presentPositionFromLoHi
MAKE_FROMLOHI_HEADER(presentSpeed)    // presentSpeedFromLoHi
MAKE_FROMLOHI_HEADER(presentLoad)     // presentLoadFromLoHi

#undef MAKE_TOLOHI_HEADER
#undef MAKE_FROMLOHI_HEADER