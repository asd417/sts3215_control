#include "convert.h"

uint8_t makeLowByte(uint16_t in) { return in & 0xFF; }
uint8_t makeHighByte(uint16_t in) { return in >> 8; }

uint16_t to_scs(int16_t v, uint8_t sign) {
  return v < 0 ? (uint16_t)(-v) | (1u << sign) : (uint16_t)v;
}
int16_t from_scs(uint16_t v, uint8_t sign) {
  return (v & (1u << sign)) ? -(int16_t)(v & ~(1u << sign)) : (int16_t)v;
}

#define MAKE_TOLOHI(name, signB)                                               \
  void name##ToLoHi(int16_t in, uint8_t *lo, uint8_t *hi) {                    \
    uint16_t raw = to_scs(in, signB);                                          \
    *lo = makeLowByte(raw);                                                        \
    *hi = makeHighByte(raw);                                                       \
  }

#define MAKE_FROMLOHI(name, signB)                                             \
  int16_t name##FromLoHi(uint8_t lo, uint8_t hi) {                             \
    uint16_t raw = (uint16_t)hi << 8 | lo;                                     \
    return from_scs(raw, signB);                                               \
  }


MAKE_TOLOHI(positionOffset, SIGN_POS_OFFSET)   // positionOffsetToLoHi
MAKE_FROMLOHI(positionOffset, SIGN_POS_OFFSET) // positionOffsetFromLoHi

MAKE_TOLOHI(goalPosition, SIGN_GOAL_POS)     // goalPositionToLoHi
MAKE_FROMLOHI(goalPosition, SIGN_GOAL_POS)   // goalPositionFromLoHi

// only used in pwm mode
MAKE_TOLOHI(goalTime, SIGN_GOAL_TIME)   // goalTimeToLoHi
MAKE_FROMLOHI(goalTime, SIGN_GOAL_TIME) // goalTimeFromLoHi

MAKE_TOLOHI(goalSpeed, SIGN_GOAL_SPEED_WHEEL)   // goalSpeedToLoHi
MAKE_FROMLOHI(goalSpeed, SIGN_GOAL_SPEED_WHEEL) // goalSpeedFromLoHi

MAKE_FROMLOHI(presentPosition, SIGN_PRESENT) // presentPositionFromLoHi
MAKE_FROMLOHI(presentSpeed, SIGN_SPEED)    // presentSpeedFromLoHi
MAKE_FROMLOHI(presentLoad, SIGN_LOAD)     // presentLoadFromLoHi