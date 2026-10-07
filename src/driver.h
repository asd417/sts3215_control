#pragma once
#include <stdint.h>

#include "driver_errors.h"
#include "platforms/platform.h"
#include "servo_errors.h"

// every function returns 0 on success or a driver_errors.h code;
// on SERVO_ERROR, *servoError holds the servo's error byte (servo_errors.h)

/* getters */
int getFirmwareMajor(PORT_HANDLE h, const uint8_t ID, uint8_t *major, int *servoError);

/* 1 byte */
int getFirmwareMinor(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getID(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getPhase(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getPGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getDGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getIGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getTorqueEnable(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getAcceleration(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getLock(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getPresentVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getPresentTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getStatus(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);
int getMoving(PORT_HANDLE h, const uint8_t ID, uint8_t *value, int *servoError);

/* 2 bytes, unsigned, low byte first */
int getModelNumber(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getTorqueLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);
int getPresentCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t *value, int *servoError);

/* 2 bytes, sign-magnitude */
int getPositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getGoalPosition(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getGoalTime(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getGoalSpeed(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getPresentPosition(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getPresentSpeed(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);
int getPresentLoad(PORT_HANDLE h, const uint8_t ID, int16_t *value, int *servoError);

/* setters. set* on an EEPROM register (ID..SPEED_I_GAIN) is lost at power-off
   unless the servo is unlocked; setSave* unlocks, writes and relocks.
   setID always saves. it refuses 0xFE (INVALID_ARGUMENT) and a new ID that
   something on the bus already answers to (ID_IN_USE) */

/* 1 byte */
int setID(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setPhase(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setDGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setTorqueEnable(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setAcceleration(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setLock(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);

/* 2 bytes, unsigned, low byte first */
int setMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setTorqueLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);

/* 2 bytes, sign-magnitude */
int setPositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t value, int *servoError);
int setGoalPosition(PORT_HANDLE h, const uint8_t ID, int16_t value, int *servoError);
int setGoalTime(PORT_HANDLE h, const uint8_t ID, int16_t value, int *servoError);
int setGoalSpeed(PORT_HANDLE h, const uint8_t ID, int16_t value, int *servoError);

/* setSave*: EEPROM only, persists across power-off */

/* 1 byte */
int setSaveBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSavePhase(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSavePGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveDGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);
int setSaveSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, int *servoError);

/* 2 bytes, unsigned, low byte first */
int setSaveMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setSaveMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setSaveMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setSaveMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);
int setSaveProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t value, int *servoError);

/* 2 bytes, sign-magnitude */
int setSavePositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t value, int *servoError);

/* instructions. addr/size are the ADDR_x / SIZE_x pairs from register.h, data is
   raw register bytes (low byte first, see convert.h) */

int ping(PORT_HANDLE h, const uint8_t ID, int *servoError);

// buffered write: the servo holds it until action()
int regWrite(PORT_HANDLE h, const uint8_t ID, const uint8_t addr, const uint8_t size,
             uint8_t *data, int *servoError);
// broadcast: every servo runs its pending regWrite. no reply
int action(PORT_HANDLE h);

// commandBuffer is ID1 d1..dn ID2 d1..dn ..., count entries of size+1 bytes. no reply
int syncWrite(PORT_HANDLE h, const uint8_t addr, const uint8_t size, const uint8_t count,
              uint8_t *commandBuffer);
// out receives count*size bytes, servo i's data at out[i*size]. a failing servo
// doesn't stop the rest; results and servoErrors hold count entries each:
//   results[i] == 0              data valid
//   results[i] == SERVO_ERROR    data valid, servoErrors[i] is the servo's error byte
//   results[i] == PACKET_TIMEOUT no valid reply (silent or corrupted), out untouched
// returns 0 if every servo succeeded, else the first nonzero results[i]
int syncRead(PORT_HANDLE h, const uint8_t addr, const uint8_t size, const uint8_t count,
             uint8_t *IDs, uint8_t *out, uint8_t *results, uint8_t *servoErrors);
