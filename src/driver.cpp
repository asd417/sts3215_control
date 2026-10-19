#include "driver.h"
#include "convert.h"
#include "driver_errors.h"
#include "driver_util.h"
#include "packet.h"
#include "platforms/platform.h"
#include "register.h"
#include "servo_errors.h"

#define MAKE_GETTER_U8(name, reg)                                              \
  uint8_t get##name(PORT_HANDLE h, const uint8_t ID, uint8_t *value,               \
                uint8_t *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    uint8_t error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    *value = p[0];                                                             \
    return error;                                                              \
  }

#define MAKE_GETTER_UNSIGNED(name, reg)                                        \
  uint8_t get##name(PORT_HANDLE h, const uint8_t ID, uint16_t *value,              \
                uint8_t *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    uint8_t error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    *value = (uint16_t)p[1] << 8 | p[0];                                       \
    return error;                                                              \
  }

#define MAKE_GETTER_SIGNED(name, reg, signB)                                   \
  uint8_t get##name(PORT_HANDLE h, const uint8_t ID, int16_t *value,               \
                uint8_t *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    uint8_t error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    uint16_t raw = (uint16_t)p[1] << 8 | p[0];                                 \
    *value = from_scs(raw, signB);                                             \
    return error;                                                              \
  }
// synced getters: value[i] belongs to IDs[i]; entries whose results[i] is
// PACKET_TIMEOUT are left untouched (see syncRead)
#define MAKE_SYNCED_GETTER_U8(name, reg)                                       \
  uint8_t getSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          uint8_t *value, uint8_t *results,                    \
                          uint8_t *servoErrors) {                              \
    return syncRead(h, ADDR_##reg, SIZE_##reg, count, IDs, value, results,     \
                    servoErrors);                                              \
  }

// value doubles as syncRead's raw buffer (count * 2 bytes) and is converted in
// place
#define MAKE_SYNCED_GETTER_UNSIGNED(name, reg)                                 \
  uint8_t getSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          uint16_t *value, uint8_t *results,                   \
                          uint8_t *servoErrors) {                              \
    uint8_t *raw = (uint8_t *)value;                                           \
    uint8_t error = syncRead(h, ADDR_##reg, SIZE_##reg, count, IDs, raw,       \
                             results, servoErrors);                            \
    for (uint8_t i = 0; i < count; i++)                                        \
      if (results[i] == 0 || results[i] == SERVO_ERROR)                        \
        value[i] = (uint16_t)raw[2 * i + 1] << 8 | raw[2 * i];                 \
    return error;                                                              \
  }

#define MAKE_SYNCED_GETTER_SIGNED(name, reg, signB)                            \
  uint8_t getSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          int16_t *value, uint8_t *results,                    \
                          uint8_t *servoErrors) {                              \
    uint8_t *raw = (uint8_t *)value;                                           \
    uint8_t error = syncRead(h, ADDR_##reg, SIZE_##reg, count, IDs, raw,       \
                             results, servoErrors);                            \
    for (uint8_t i = 0; i < count; i++)                                        \
      if (results[i] == 0 || results[i] == SERVO_ERROR)                        \
        value[i] = from_scs((uint16_t)raw[2 * i + 1] << 8 | raw[2 * i], signB); \
    return error;                                                              \
  }

// plain write: RAM registers take effect now, EEPROM ones are lost at
// power-off unless the servo is unlocked
#define MAKE_SETTER_U8(name, reg, validMin, validMax)                          \
  uint8_t set##name(PORT_HANDLE h, const uint8_t ID, uint8_t value,                \
                uint8_t *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t *p = &value;                                                       \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }

#define MAKE_SETTER_U16(name, reg, validMin, validMax)                         \
  uint8_t set##name(PORT_HANDLE h, const uint8_t ID, uint16_t value,               \
                uint8_t *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2] = {makeLowByte(value), makeHighByte(value)};                  \
    uint8_t *p = d;                                                            \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }

// conv is the convert.h prefix, e.g. goalPosition -> goalPositionToLoHi
#define MAKE_SETTER_S16(name, conv, reg, validMin, validMax)                   \
  uint8_t set##name(PORT_HANDLE h, const uint8_t ID, int16_t value,                \
                uint8_t *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2];                                                              \
    conv##ToLoHi(value, &d[0], &d[1]);                                         \
    uint8_t *p = d;                                                            \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }

// setSave* variants: unlock, write, relock so EEPROM values persist
#define MAKE_SAVE_SETTER_U8(name, reg, validMin, validMax)                     \
  uint8_t setSave##name(PORT_HANDLE h, const uint8_t ID, uint8_t value,            \
                    uint8_t *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t *p = &value;                                                       \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

#define MAKE_SAVE_SETTER_U16(name, reg, validMin, validMax)                    \
  uint8_t setSave##name(PORT_HANDLE h, const uint8_t ID, uint16_t value,           \
                    uint8_t *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2] = {makeLowByte(value), makeHighByte(value)};                  \
    uint8_t *p = d;                                                            \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

// conv is the convert.h prefix, e.g. goalPosition -> goalPositionToLoHi
#define MAKE_SAVE_SETTER_S16(name, conv, reg, validMin, validMax)              \
  uint8_t setSave##name(PORT_HANDLE h, const uint8_t ID, int16_t value,            \
                    uint8_t *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2];                                                              \
    conv##ToLoHi(value, &d[0], &d[1]);                                         \
    uint8_t *p = d;                                                            \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

// synced setters: values[i] goes to IDs[i], all in one SYNC_WRITE packet, no
// reply. one value out of range rejects the whole call before anything is sent
#define MAKE_SYNCED_SETTER_U8(name, reg, validMin, validMax)                   \
  uint8_t setSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          const uint8_t *values) {                             \
    for (uint8_t i = 0; i < count; i++)                                        \
      if (values[i] < validMin || values[i] > validMax)                        \
        return RANGE_ERROR;                                                    \
    uint8_t *b = syncWriteBuffer(count, SIZE_##reg);                           \
    if (!b)                                                                    \
      return INCORRECT_WRITE_PACKET_SIZE;                                      \
    for (uint8_t i = 0; i < count; i++) {                                      \
      b[2 * i] = IDs[i];                                                       \
      b[2 * i + 1] = values[i];                                                \
    }                                                                          \
    return syncWrite(h, ADDR_##reg, SIZE_##reg, count, b);                     \
  }

#define MAKE_SYNCED_SETTER_U16(name, reg, validMin, validMax)                  \
  uint8_t setSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          const uint16_t *values) {                            \
    for (uint8_t i = 0; i < count; i++)                                        \
      if (values[i] < validMin || values[i] > validMax)                        \
        return RANGE_ERROR;                                                    \
    uint8_t *b = syncWriteBuffer(count, SIZE_##reg);                           \
    if (!b)                                                                    \
      return INCORRECT_WRITE_PACKET_SIZE;                                      \
    for (uint8_t i = 0; i < count; i++) {                                      \
      b[3 * i] = IDs[i];                                                       \
      b[3 * i + 1] = makeLowByte(values[i]);                                   \
      b[3 * i + 2] = makeHighByte(values[i]);                                  \
    }                                                                          \
    return syncWrite(h, ADDR_##reg, SIZE_##reg, count, b);                     \
  }

// conv is the convert.h prefix, e.g. goalPosition -> goalPositionToLoHi
#define MAKE_SYNCED_SETTER_S16(name, conv, reg, validMin, validMax)            \
  uint8_t setSynced##name(PORT_HANDLE h, const uint8_t count, uint8_t *IDs,    \
                          const int16_t *values) {                             \
    for (uint8_t i = 0; i < count; i++)                                        \
      if (values[i] < validMin || values[i] > validMax)                        \
        return RANGE_ERROR;                                                    \
    uint8_t *b = syncWriteBuffer(count, SIZE_##reg);                           \
    if (!b)                                                                    \
      return INCORRECT_WRITE_PACKET_SIZE;                                      \
    for (uint8_t i = 0; i < count; i++) {                                      \
      b[3 * i] = IDs[i];                                                       \
      conv##ToLoHi(values[i], &b[3 * i + 1], &b[3 * i + 2]);                   \
    }                                                                          \
    return syncWrite(h, ADDR_##reg, SIZE_##reg, count, b);                     \
  }

/* 1 byte */
MAKE_GETTER_U8(FirmwareMajor, FIRMWARE_MAJOR)
MAKE_GETTER_U8(FirmwareMinor, FIRMWARE_MINOR)
MAKE_GETTER_U8(ID, ID)
MAKE_GETTER_U8(BaudRate, BAUD_RATE)
MAKE_GETTER_U8(ReturnDelay, RETURN_DELAY)
MAKE_GETTER_U8(ResponseLevel, RESPONSE_LEVEL)
MAKE_GETTER_U8(MaxTemperature, MAX_TEMPERATURE)
MAKE_GETTER_U8(MaxVoltage, MAX_VOLTAGE)
MAKE_GETTER_U8(MinVoltage, MIN_VOLTAGE)
MAKE_GETTER_U8(Phase, PHASE)
MAKE_GETTER_U8(UnloadingCondition, UNLOADING_CONDITION)
MAKE_GETTER_U8(LedAlarmCondition, LED_ALARM_CONDITION)
MAKE_GETTER_U8(PGain, P_GAIN)
MAKE_GETTER_U8(DGain, D_GAIN)
MAKE_GETTER_U8(IGain, I_GAIN)
MAKE_GETTER_U8(CwDeadZone, CW_DEAD_ZONE)
MAKE_GETTER_U8(CcwDeadZone, CCW_DEAD_ZONE)
MAKE_GETTER_U8(AngularResolution, ANGULAR_RESOLUTION)
MAKE_GETTER_U8(OperatingMode, OPERATING_MODE)
MAKE_GETTER_U8(ProtectiveTorque, PROTECTIVE_TORQUE)
MAKE_GETTER_U8(ProtectionTime, PROTECTION_TIME)
MAKE_GETTER_U8(OverloadTorque, OVERLOAD_TORQUE)
MAKE_GETTER_U8(SpeedPGain, SPEED_P_GAIN)
MAKE_GETTER_U8(OvercurrentTime, OVERCURRENT_TIME)
MAKE_GETTER_U8(SpeedIGain, SPEED_I_GAIN)
MAKE_GETTER_U8(TorqueEnable, TORQUE_ENABLE)
MAKE_GETTER_U8(Acceleration, ACCELERATION)
MAKE_GETTER_U8(Lock, LOCK)
MAKE_GETTER_U8(PresentVoltage, PRESENT_VOLTAGE)
MAKE_GETTER_U8(PresentTemperature, PRESENT_TEMPERATURE)
MAKE_GETTER_U8(Status, STATUS)
MAKE_GETTER_U8(Moving, MOVING)

/* 2 bytes, unsigned, low byte first */
MAKE_GETTER_UNSIGNED(ModelNumber, MODEL_NUM)
MAKE_GETTER_UNSIGNED(MinAngleLimit, MIN_ANGLE_LIMIT)
MAKE_GETTER_UNSIGNED(MaxAngleLimit, MAX_ANGLE_LIMIT)
MAKE_GETTER_UNSIGNED(MaxTorque, MAX_TORQUE)
MAKE_GETTER_UNSIGNED(MinStartupForce, MIN_STARTUP_FORCE)
MAKE_GETTER_UNSIGNED(ProtectionCurrent, PROTECTION_CURRENT)
MAKE_GETTER_UNSIGNED(TorqueLimit, TORQUE_LIMIT)
MAKE_GETTER_UNSIGNED(PresentCurrent, PRESENT_CURRENT)

/* 2 bytes, sign-magnitude */
MAKE_GETTER_SIGNED(PositionOffset, POSITION_OFFSET, SIGN_POS_OFFSET)
MAKE_GETTER_SIGNED(GoalPosition, GOAL_POSITION, SIGN_GOAL_POS)
MAKE_GETTER_SIGNED(GoalTime, GOAL_TIME, SIGN_GOAL_TIME)
MAKE_GETTER_SIGNED(GoalSpeed, GOAL_SPEED, SIGN_GOAL_SPEED_WHEEL)
MAKE_GETTER_SIGNED(PresentPosition, PRESENT_POSITION, SIGN_PRESENT)
MAKE_GETTER_SIGNED(PresentSpeed, PRESENT_SPEED, SIGN_SPEED)
MAKE_GETTER_SIGNED(PresentLoad, PRESENT_LOAD, SIGN_LOAD)

/* synced getters: RAM registers only */
MAKE_SYNCED_GETTER_U8(TorqueEnable, TORQUE_ENABLE)
MAKE_SYNCED_GETTER_U8(Acceleration, ACCELERATION)
MAKE_SYNCED_GETTER_U8(PresentVoltage, PRESENT_VOLTAGE)
MAKE_SYNCED_GETTER_U8(PresentTemperature, PRESENT_TEMPERATURE)
MAKE_SYNCED_GETTER_U8(Status, STATUS)
MAKE_SYNCED_GETTER_U8(Moving, MOVING)
MAKE_SYNCED_GETTER_UNSIGNED(TorqueLimit, TORQUE_LIMIT)
MAKE_SYNCED_GETTER_UNSIGNED(PresentCurrent, PRESENT_CURRENT)
MAKE_SYNCED_GETTER_SIGNED(GoalPosition, GOAL_POSITION, SIGN_GOAL_POS)
MAKE_SYNCED_GETTER_SIGNED(GoalTime, GOAL_TIME, SIGN_GOAL_TIME)
MAKE_SYNCED_GETTER_SIGNED(GoalSpeed, GOAL_SPEED, SIGN_GOAL_SPEED_WHEEL)
MAKE_SYNCED_GETTER_SIGNED(PresentPosition, PRESENT_POSITION, SIGN_PRESENT)
MAKE_SYNCED_GETTER_SIGNED(PresentSpeed, PRESENT_SPEED, SIGN_SPEED)
MAKE_SYNCED_GETTER_SIGNED(PresentLoad, PRESENT_LOAD, SIGN_LOAD)

#undef MAKE_GETTER_U8
#undef MAKE_GETTER_UNSIGNED
#undef MAKE_GETTER_SIGNED
#undef MAKE_SYNCED_GETTER_U8
#undef MAKE_SYNCED_GETTER_UNSIGNED
#undef MAKE_SYNCED_GETTER_SIGNED

/* setters */


MAKE_SETTER_U8(ReturnDelay, RETURN_DELAY, 0, 254)
MAKE_SAVE_SETTER_U8(ReturnDelay, RETURN_DELAY, 0, 254)
MAKE_SETTER_U8(ResponseLevel, RESPONSE_LEVEL, 0, 1)
MAKE_SAVE_SETTER_U8(ResponseLevel, RESPONSE_LEVEL, 0, 1)
MAKE_SETTER_U16(MinAngleLimit, MIN_ANGLE_LIMIT, 0, 32767)
MAKE_SAVE_SETTER_U16(MinAngleLimit, MIN_ANGLE_LIMIT, 0, 32767)
MAKE_SETTER_U16(MaxAngleLimit, MAX_ANGLE_LIMIT, 0, 32767)
MAKE_SAVE_SETTER_U16(MaxAngleLimit, MAX_ANGLE_LIMIT, 0, 32767)
MAKE_SETTER_U8(MaxTemperature, MAX_TEMPERATURE, 0, 100)
MAKE_SAVE_SETTER_U8(MaxTemperature, MAX_TEMPERATURE, 0, 100)
MAKE_SETTER_U8(MaxVoltage, MAX_VOLTAGE, 0, 254)
MAKE_SAVE_SETTER_U8(MaxVoltage, MAX_VOLTAGE, 0, 254)
MAKE_SETTER_U8(MinVoltage, MIN_VOLTAGE, 0, 254)
MAKE_SAVE_SETTER_U8(MinVoltage, MIN_VOLTAGE, 0, 254)
MAKE_SETTER_U16(MaxTorque, MAX_TORQUE, 0, 1000)
MAKE_SAVE_SETTER_U16(MaxTorque, MAX_TORQUE, 0, 1000)
MAKE_SETTER_U8(Phase, PHASE, 0, 254)
MAKE_SAVE_SETTER_U8(Phase, PHASE, 0, 254)
MAKE_SETTER_U8(UnloadingCondition, UNLOADING_CONDITION, 0, 254)
MAKE_SAVE_SETTER_U8(UnloadingCondition, UNLOADING_CONDITION, 0, 254)
MAKE_SETTER_U8(LedAlarmCondition, LED_ALARM_CONDITION, 0, 254)
MAKE_SAVE_SETTER_U8(LedAlarmCondition, LED_ALARM_CONDITION, 0, 254)
MAKE_SETTER_U8(PGain, P_GAIN, 0, 254)
MAKE_SAVE_SETTER_U8(PGain, P_GAIN, 0, 254)
MAKE_SETTER_U8(DGain, D_GAIN, 0, 254)
MAKE_SAVE_SETTER_U8(DGain, D_GAIN, 0, 254)
MAKE_SETTER_U8(IGain, I_GAIN, 0, 254)
MAKE_SAVE_SETTER_U8(IGain, I_GAIN, 0, 254)
MAKE_SETTER_U16(MinStartupForce, MIN_STARTUP_FORCE, 0, 1000)
MAKE_SAVE_SETTER_U16(MinStartupForce, MIN_STARTUP_FORCE, 0, 1000)
MAKE_SETTER_U8(CwDeadZone, CW_DEAD_ZONE, 0, 32)
MAKE_SAVE_SETTER_U8(CwDeadZone, CW_DEAD_ZONE, 0, 32)
MAKE_SETTER_U8(CcwDeadZone, CCW_DEAD_ZONE, 0, 32)
MAKE_SAVE_SETTER_U8(CcwDeadZone, CCW_DEAD_ZONE, 0, 32)
MAKE_SETTER_U16(ProtectionCurrent, PROTECTION_CURRENT, 0, 511)
MAKE_SAVE_SETTER_U16(ProtectionCurrent, PROTECTION_CURRENT, 0, 511)
MAKE_SETTER_U8(AngularResolution, ANGULAR_RESOLUTION, 1, 3)
MAKE_SAVE_SETTER_U8(AngularResolution, ANGULAR_RESOLUTION, 1, 3)
MAKE_SETTER_S16(PositionOffset, positionOffset, POSITION_OFFSET, -2047, 2047)
MAKE_SAVE_SETTER_S16(PositionOffset, positionOffset, POSITION_OFFSET, -2047,
                     2047)
MAKE_SETTER_U8(OperatingMode, OPERATING_MODE, 0, 3)
MAKE_SAVE_SETTER_U8(OperatingMode, OPERATING_MODE, 0, 3)
MAKE_SETTER_U8(ProtectiveTorque, PROTECTIVE_TORQUE, 0, 100)
MAKE_SAVE_SETTER_U8(ProtectiveTorque, PROTECTIVE_TORQUE, 0, 100)
MAKE_SETTER_U8(ProtectionTime, PROTECTION_TIME, 0, 254)
MAKE_SAVE_SETTER_U8(ProtectionTime, PROTECTION_TIME, 0, 254)
MAKE_SETTER_U8(OverloadTorque, OVERLOAD_TORQUE, 0, 100)
MAKE_SAVE_SETTER_U8(OverloadTorque, OVERLOAD_TORQUE, 0, 100)
MAKE_SETTER_U8(SpeedPGain, SPEED_P_GAIN, 0, 100)
MAKE_SAVE_SETTER_U8(SpeedPGain, SPEED_P_GAIN, 0, 100)
MAKE_SETTER_U8(OvercurrentTime, OVERCURRENT_TIME, 0, 254)
MAKE_SAVE_SETTER_U8(OvercurrentTime, OVERCURRENT_TIME, 0, 254)
MAKE_SETTER_U8(SpeedIGain, SPEED_I_GAIN, 0, 254)
MAKE_SAVE_SETTER_U8(SpeedIGain, SPEED_I_GAIN, 0, 254)

/* RAM */
MAKE_SETTER_U8(TorqueEnable, TORQUE_ENABLE, 0, 1)
MAKE_SETTER_U8(Acceleration, ACCELERATION, 0, 254)
MAKE_SETTER_S16(GoalPosition, goalPosition, GOAL_POSITION, -30719, 30719)
MAKE_SETTER_S16(GoalTime, goalTime, GOAL_TIME, -1000, 1000)
MAKE_SETTER_S16(GoalSpeed, goalSpeed, GOAL_SPEED, -3400, 3400)
MAKE_SETTER_U16(TorqueLimit, TORQUE_LIMIT, 0, 1000)
MAKE_SETTER_U8(Lock, LOCK, 0, 1)

/* synced setters: RAM registers only, EEPROM needs the per-servo unlock */
MAKE_SYNCED_SETTER_U8(TorqueEnable, TORQUE_ENABLE, 0, 1)
MAKE_SYNCED_SETTER_U8(Acceleration, ACCELERATION, 0, 254)
MAKE_SYNCED_SETTER_S16(GoalPosition, goalPosition, GOAL_POSITION, -30719, 30719)
MAKE_SYNCED_SETTER_S16(GoalTime, goalTime, GOAL_TIME, -1000, 1000)
MAKE_SYNCED_SETTER_S16(GoalSpeed, goalSpeed, GOAL_SPEED, -3400, 3400)
MAKE_SYNCED_SETTER_U16(TorqueLimit, TORQUE_LIMIT, 0, 1000)

#undef MAKE_SETTER_U8
#undef MAKE_SETTER_U16
#undef MAKE_SETTER_S16
#undef MAKE_SAVE_SETTER_U8
#undef MAKE_SAVE_SETTER_U16
#undef MAKE_SAVE_SETTER_S16
#undef MAKE_SYNCED_SETTER_U8
#undef MAKE_SYNCED_SETTER_U16
#undef MAKE_SYNCED_SETTER_S16

/* EEPROM: set* is temporary, setSave* persists. setID is written by hand below
 */

uint8_t setBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value,
                uint8_t *servoError) {
  return writeBaud(h, ID, value, false, servoError);
}

uint8_t setSaveBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value,
                    uint8_t *servoError) {
  return writeBaud(h, ID, value, true, servoError);
}

// Register 40 Write 128: Arbitrary current position correction to 2048.
uint8_t setCalibrateCurrentPos(PORT_HANDLE h, const uint8_t ID, uint8_t *servoError) {
  uint8_t v = 128;
  uint8_t *vp = &v;
  return writeRegister(h, ID, ADDR_TORQUE_ENABLE, SIZE_TORQUE_ENABLE, &vp,
                       servoError);
}

// relock goes to the new ID: the servo answers only to `value` after the write
uint8_t setID(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError) {
  // 0xFE is broadcast: as the target it would rename every servo, as the value
  // it makes a servo that can't be addressed on its own
  if (ID == 0xFE || value == 0xFE)
    return INVALID_ARGUMENT;
  // the new ID is free only if nobody answers. any other result (a servo, a
  // faulted servo, garbled bytes) means something is there.
  // only sees servos that are powered and at the host's baud rate
  if (value != ID && ping(h, value, servoError) != PACKET_TIMEOUT)
    return ID_IN_USE;
  uint8_t r = setLock(h, ID, 0, servoError);
  if (r)
    return r;
  uint8_t *p = &value;
  // result ignored: the reply may carry either ID, and a servo can apply the
  // write even when its reply is lost or garbled. the relock below decides
  writeRegister(h, ID, ADDR_ID, SIZE_ID, &p, servoError);
  r = setLock(h, value, 1, servoError);
  if (r && value != ID)
    setLock(h, ID, 1, servoError); // it never moved: don't leave it unlocked
  return r;
}

/* instructions */

uint8_t ping(PORT_HANDLE h, const uint8_t ID, uint8_t *servoError) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_PING)
    return INCORRECT_WRITE_PACKET_SIZE;
  createPingPacket(w, ID);
  if (sendPacket(w, PACKET_SIZE_PING, h))
    return PACKET_SEND_FAILED;
  return readStatus(h, ID, 0, nullptr, servoError);
}
