#include "servo_errors.h"

#ifndef __AVR__
// error is the servo's error byte. several bits can be set at once: a single
// bit gets its own message, a combination gets "Multiple servo errors", so
// pass one bit at a time to name each.
const char *servoErrorToString(int error) {
  switch (error) {
  case 0:
    return "OK";
  case SERVO_ERROR_VOLTAGE:
    return "Supply voltage out of range";
  case SERVO_ERROR_ANGLE:
    return "Angle sensor fault or goal outside angle limits";
  case SERVO_ERROR_TEMP:
    return "Overheat";
  case SERVO_ERROR_CURRENT:
    return "Overcurrent";
  case SERVO_ERROR_OVERLOAD:
    return "Overload";
  default:
    return (error & ~SERVO_ERROR_ANY) ? "Unknown servo error bit"
                                      : "Multiple servo errors";
  }
}
#endif
