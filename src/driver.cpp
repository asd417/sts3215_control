#include "driver.h"
#include "convert.h"
#include "driver_errors.h"
#include "packet.h"
#include "platforms/platform.h"
#include "register.h"
#include "servo_errors.h"

//  timeout = reply_bytes * time_per_byte + return_delay + platform_latency +
//  margin
// ponytail: fixed worst case, compute from the formula above if the bus needs
// tighter timing
#define READ_TIMEOUT_MS 50

#define MAKE_GETTER_U8(name, reg)                                              \
  int get##name(PORT_HANDLE h, const uint8_t ID, uint8_t *value,               \
                int *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    int error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    *value = p[0];                                                             \
    return error;                                                              \
  }

#define MAKE_GETTER_UNSIGNED(name, reg)                                        \
  int get##name(PORT_HANDLE h, const uint8_t ID, uint16_t *value,              \
                int *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    int error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    *value = (uint16_t)p[1] << 8 | p[0];                                       \
    return error;                                                              \
  }

#define MAKE_GETTER_SIGNED(name, reg, signB)                                   \
  int get##name(PORT_HANDLE h, const uint8_t ID, int16_t *value,               \
                int *servoError) {                                             \
    uint8_t *p;                                                                \
    *servoError = 0;                                                           \
    int error = readRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);   \
    if (error && error != SERVO_ERROR)                                         \
      return error;                                                            \
    uint16_t raw = (uint16_t)p[1] << 8 | p[0];                                 \
    *value = from_scs(raw, signB);                                             \
    return error;                                                              \
  }

// plain write: RAM registers take effect now, EEPROM ones are lost at
// power-off unless the servo is unlocked
#define MAKE_SETTER_U8(name, reg, validMin, validMax)                          \
  int set##name(PORT_HANDLE h, const uint8_t ID, uint8_t value,                \
                int *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t *p = &value;                                                       \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }



#define MAKE_SETTER_U16(name, reg, validMin, validMax)                         \
  int set##name(PORT_HANDLE h, const uint8_t ID, uint16_t value,               \
                int *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2] = {makeLowByte(value), makeHighByte(value)};                  \
    uint8_t *p = d;                                                            \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }

// conv is the convert.h prefix, e.g. goalPosition -> goalPositionToLoHi
#define MAKE_SETTER_S16(name, conv, reg, validMin, validMax)                   \
  int set##name(PORT_HANDLE h, const uint8_t ID, int16_t value,                \
                int *servoError) {                                             \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2];                                                              \
    conv##ToLoHi(value, &d[0], &d[1]);                                         \
    uint8_t *p = d;                                                            \
    return writeRegister(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);       \
  }

// setSave* variants: unlock, write, relock so EEPROM values persist
#define MAKE_SAVE_SETTER_U8(name, reg, validMin, validMax)                     \
  int setSave##name(PORT_HANDLE h, const uint8_t ID, uint8_t value,            \
                    int *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t *p = &value;                                                       \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

#define MAKE_SAVE_SETTER_U16(name, reg, validMin, validMax)                    \
  int setSave##name(PORT_HANDLE h, const uint8_t ID, uint16_t value,           \
                    int *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2] = {makeLowByte(value), makeHighByte(value)};                  \
    uint8_t *p = d;                                                            \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

// conv is the convert.h prefix, e.g. goalPosition -> goalPositionToLoHi
#define MAKE_SAVE_SETTER_S16(name, conv, reg, validMin, validMax)              \
  int setSave##name(PORT_HANDLE h, const uint8_t ID, int16_t value,            \
                    int *servoError) {                                         \
    if (value < validMin || value > validMax)                                  \
      return RANGE_ERROR;                                                      \
    uint8_t d[2];                                                              \
    conv##ToLoHi(value, &d[0], &d[1]);                                         \
    uint8_t *p = d;                                                            \
    return writeEEPROM(h, ID, ADDR_##reg, SIZE_##reg, &p, servoError);         \
  }

// shared read flow: on success *parms points at `size` data bytes in the RX
// buffer (valid until the next read)
static int readRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                        const uint8_t size, uint8_t **parms, int *servoError) {
  *servoError = 0; // start clean
  // write packet
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_READ)
    return INCORRECT_WRITE_PACKET_SIZE;
  // send
  createReadPacket(w, ID, addr, size);
  if (sendPacket(w, PACKET_SIZE_READ, h))
    return PACKET_SEND_FAILED;
  // start reading packet
  uint8_t r_size;
  uint8_t *r = getPacketRX(&r_size);
  if (r_size < PACKET_SIZE_OVERHEAD + size)
    return INCORRECT_READ_PACKET_SIZE;
  int re = readPacket(r, PACKET_SIZE_OVERHEAD + size, READ_TIMEOUT_MS, h);
  if (re)
    return PACKET_TIMEOUT;
  uint8_t outID;
  uint8_t outError;
  int error = parseSinglePacket(r, r_size, size, &outID, &outError, parms);
  if (error)
    return error;
  if (outError & SERVO_ERROR_ANY) {
    *servoError = outError;
    return SERVO_ERROR;
  }
  if (outID != ID)
    return UNEXPECTED_ID;
  return 0;
}

static int writeRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                         const uint8_t size, uint8_t **parms, int *servoError) {
  *servoError = 0; // start clean
  // write packet
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_WRITE_BASE + size)
    return INCORRECT_WRITE_PACKET_SIZE;
  createWritePacket(w, ID, size, *parms, addr);
  if (sendPacket(w, PACKET_SIZE_WRITE_BASE + size, h))
    return PACKET_SEND_FAILED;
  // start reading packet
  if (ID == 0xFE)
    return 0;
  uint8_t r_size;
  uint8_t *r = getPacketRX(&r_size);
  if (r_size < PACKET_SIZE_OVERHEAD)
    return INCORRECT_READ_PACKET_SIZE;
  int re = readPacket(r, PACKET_SIZE_OVERHEAD, READ_TIMEOUT_MS, h);
  if (re)
    return PACKET_TIMEOUT;
  uint8_t outID;
  uint8_t outError;
  int error = parseSinglePacket(r, r_size, 0, &outID, &outError, nullptr);
  if (error)
    return error;
  if (outError & SERVO_ERROR_ANY) {
    *servoError = outError;
    return SERVO_ERROR;
  }
  if (outID != ID)
    return UNEXPECTED_ID;
  return 0;
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

#undef MAKE_GETTER_U8
#undef MAKE_GETTER_UNSIGNED
#undef MAKE_GETTER_SIGNED

/* setters */

// unlock, write, relock. relocks even if the write failed so the servo isn't
// left unlocked; the write's error wins over the relock's
static int writeEEPROM(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                       const uint8_t size, uint8_t **parms, int *servoError) {
  uint8_t lock = 0;
  uint8_t *l = &lock;
  int error = writeRegister(h, ID, ADDR_LOCK, SIZE_LOCK, &l, servoError);
  if (error)
    return error;
  error = writeRegister(h, ID, addr, size, parms, servoError);
  lock = 1;
  int relock = writeRegister(h, ID, ADDR_LOCK, SIZE_LOCK, &l, servoError);
  return error ? error : relock;
}

/* EEPROM: set* is temporary, setSave* persists. setID is written by hand below
 */

// baud needs to be written in a special way because it also needs to be change
// on the platform side before reading the response from the motor
static int writeBaud(PORT_HANDLE h, const uint8_t ID, uint8_t index,
                     const bool save, int *servoError) {
  // broadcast can't be read back or verified
  if (ID == 0xFE || !baudFromIndex(index))
    return INVALID_ARGUMENT;
  uint8_t old;
  int error = getBaudRate(h, ID, &old, servoError);
  if (error)
    return error; // servo unreachable, nothing changed
  if (save) {
    error = setLock(h, ID, 0, servoError);
    if (error)
      return error;
  }
  uint8_t *p = &index;
  // result ignored: the reply may arrive at either rate. the check below
  // decides ponytail: no settle delay before the next packet, add one to
  // platform.h if the check fails on a servo that did switch
  writeRegister(h, ID, ADDR_BAUD_RATE, SIZE_BAUD_RATE, &p, servoError);
  if (setPlatformBaudRate(index, h))
    return CONNECTION_LOST;
  // the first exchange at the new rate proves the servo followed
  error = save ? setLock(h, ID, 1, servoError) : ping(h, ID, servoError);
  if (error) {
    setPlatformBaudRate(old, h); // revert the system baud if necessary.
    if (save)
      setLock(h, ID, 1, servoError); // don't leave it unlocked
  }
  return error;
}

int setBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value,
                int *servoError) {
  return writeBaud(h, ID, value, false, servoError);
}

int setSaveBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value,
                    int *servoError) {
  return writeBaud(h, ID, value, true, servoError);
}
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

#undef MAKE_SETTER_U8
#undef MAKE_SETTER_U16
#undef MAKE_SETTER_S16
#undef MAKE_SAVE_SETTER_U8
#undef MAKE_SAVE_SETTER_U16
#undef MAKE_SAVE_SETTER_S16

// Register 40 Write 128: Arbitrary current position correction to 2048.
int setCalibrateCurrentPos(PORT_HANDLE h, const uint8_t ID, int *servoError) {
  uint8_t v = 128;
  uint8_t *vp = &v;
  return writeRegister(h, ID, ADDR_TORQUE_ENABLE, SIZE_TORQUE_ENABLE, &vp,
                       servoError);
}

// relock goes to the new ID: the servo answers only to `value` after the write
int setID(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError) {
  // 0xFE is broadcast: as the target it would rename every servo, as the value
  // it makes a servo that can't be addressed on its own
  if (ID == 0xFE || value == 0xFE)
    return INVALID_ARGUMENT;
  // the new ID is free only if nobody answers. any other result (a servo, a
  // faulted servo, garbled bytes) means something is there.
  // only sees servos that are powered and at the host's baud rate
  if (value != ID && ping(h, value, servoError) != PACKET_TIMEOUT)
    return ID_IN_USE;
  int r = setLock(h, ID, 0, servoError);
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

// read one status packet carrying `size` data bytes from servo ID
static int readStatus(PORT_HANDLE h, const uint8_t ID, const uint8_t size,
                      uint8_t **parms, int *servoError) {
  *servoError = 0; // start clean
  uint8_t r_size;
  uint8_t *r = getPacketRX(&r_size);
  if (r_size < PACKET_SIZE_OVERHEAD + size)
    return INCORRECT_READ_PACKET_SIZE;
  if (readPacket(r, PACKET_SIZE_OVERHEAD + size, READ_TIMEOUT_MS, h))
    return PACKET_TIMEOUT;
  uint8_t outID;
  uint8_t outError;
  int error = parseSinglePacket(r, r_size, size, &outID, &outError, parms);
  if (error)
    return error;
  if (outError & SERVO_ERROR_ANY) {
    *servoError = outError;
    return SERVO_ERROR;
  }
  if (outID != ID)
    return UNEXPECTED_ID;
  return 0;
}

int ping(PORT_HANDLE h, const uint8_t ID, int *servoError) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_PING)
    return INCORRECT_WRITE_PACKET_SIZE;
  createPingPacket(w, ID);
  if (sendPacket(w, PACKET_SIZE_PING, h))
    return PACKET_SEND_FAILED;
  return readStatus(h, ID, 0, nullptr, servoError);
}

int regWrite(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
             const uint8_t size, uint8_t *data, int *servoError) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_REG_WRITE_BASE + size)
    return INCORRECT_WRITE_PACKET_SIZE;
  createRegWritePacket(w, ID, size, data, addr);
  if (sendPacket(w, PACKET_SIZE_REG_WRITE_BASE + size, h))
    return PACKET_SEND_FAILED;
  if (ID == 0xFE)
    return 0;
  return readStatus(h, ID, 0, nullptr, servoError);
}

int action(PORT_HANDLE h) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_ACTION)
    return INCORRECT_WRITE_PACKET_SIZE;
  createActionPacket(w, 0xFE);
  return sendPacket(w, PACKET_SIZE_ACTION, h) ? PACKET_SEND_FAILED : 0;
}

int syncWrite(PORT_HANDLE h, const uint8_t addr, const uint8_t size,
              const uint8_t count, uint8_t *commandBuffer) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  // 16 bit: count * (size + 1) can pass 255
  uint16_t n = PACKET_SIZE_SYNC_WRITE_BASE + (uint16_t)count * (size + 1);
  if (n > w_size)
    return INCORRECT_WRITE_PACKET_SIZE;
  createSyncWritePacket(w, count, addr, size, commandBuffer);
  return sendPacket(w, (uint8_t)n, h) ? PACKET_SEND_FAILED : 0;
}

int syncRead(PORT_HANDLE h, const uint8_t addr, const uint8_t size,
             const uint8_t count, uint8_t *IDs, uint8_t *out, uint8_t *results,
             uint8_t *servoErrors) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  uint16_t n = PACKET_SIZE_SYNC_READ_BASE + (uint16_t)count;
  if (n > w_size)
    return INCORRECT_WRITE_PACKET_SIZE;
  uint8_t r_size;
  uint8_t *r = getPacketRX(&r_size);
  if (r_size < PACKET_SIZE_OVERHEAD + size)
    return INCORRECT_READ_PACKET_SIZE;
  createSyncReadPacket(w, count, IDs, addr, size);
  if (sendPacket(w, (uint8_t)n, h))
    return PACKET_SEND_FAILED;
  for (uint8_t i = 0; i < count; i++) {
    results[i] = PACKET_TIMEOUT;
    servoErrors[i] = 0;
  }
  // one status packet per servo. each is filed by the ID inside it, not by
  // arrival order, so a silent servo doesn't shift the ones after it
  for (uint8_t k = 0; k < count; k++) {
    if (readPacket(r, PACKET_SIZE_OVERHEAD + size, READ_TIMEOUT_MS, h))
      break; // bus went quiet, nothing more is coming
    uint8_t id;
    uint8_t err;
    uint8_t *p;
    if (parseSinglePacket(r, r_size, size, &id, &err, &p))
      continue; // corrupt: can't trust its ID, that servo stays PACKET_TIMEOUT
    for (uint8_t i = 0; i < count; i++) {
      if (IDs[i] != id)
        continue;
      for (uint8_t j = 0; j < size; j++)
        out[(uint16_t)i * size + j] = p[j];
      servoErrors[i] = err;
      results[i] = (err & SERVO_ERROR_ANY) ? SERVO_ERROR : 0;
      break;
    }
  }
  for (uint8_t i = 0; i < count; i++)
    if (results[i])
      return results[i];
  return 0;
}

// The same thing with your functions
// 1. setTorqueEnable(h, id, 0, &se), then move the joint to centre by hand.
// 2. setSavePositionOffset(h, id, 0, &se), setSaveMinAngleLimit(h, id, 0, &se),
// setSaveMaxAngleLimit(h, id, 4095, &se).
// 3. getPresentPosition(h, id, &raw, &se).
// 4. setSavePositionOffset(h, id, raw - 2047, &se).
// 5. Check: getPresentPosition should now read 2047.
