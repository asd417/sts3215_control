#pragma once
#include <stdint.h>

#include "driver_errors.h"
#include "platforms/platform.h"
#include "servo_errors.h"

// every function returns 0 on success or a driver_errors.h code;
// on SERVO_ERROR, *servoError holds the servo's error byte (servo_errors.h)

/* getters */
uint8_t getFirmwareMajor(PORT_HANDLE h, const uint8_t ID, uint8_t *major, uint8_t *servoError);

/* 1 byte */
uint8_t getFirmwareMinor(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getID(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getPhase(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getPGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getDGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getIGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getTorqueEnable(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getAcceleration(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getLock(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getPresentVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getPresentTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getStatus(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);
uint8_t getMoving(PORT_HANDLE h, const uint8_t ID, uint8_t *value, uint8_t *servoError);

/* 2 bytes, unsigned, low byte first */
uint8_t getModelNumber(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getTorqueLimit(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);
uint8_t getPresentCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t *value, uint8_t *servoError);

/* 2 bytes, sign-magnitude */
uint8_t getPositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getGoalPosition(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getGoalTime(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getGoalSpeed(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getPresentPosition(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getPresentSpeed(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);
uint8_t getPresentLoad(PORT_HANDLE h, const uint8_t ID, int16_t *value, uint8_t *servoError);

/* setters. set* on an EEPROM register (ID..SPEED_I_GAIN) is lost at power-off
   unless the servo is unlocked; setSave* unlocks, writes and relocks.
   setID always saves. it refuses 0xFE (INVALID_ARGUMENT) and a new ID that
   something on the bus already answers to (ID_IN_USE) */

/* 1 byte */
uint8_t setID(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setPhase(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setDGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setTorqueEnable(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setAcceleration(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setLock(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);

/* 2 bytes, unsigned, low byte first */
uint8_t setMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setTorqueLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);

/* 2 bytes, sign-magnitude */
uint8_t setPositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t value, uint8_t *servoError);
uint8_t setGoalPosition(PORT_HANDLE h, const uint8_t ID, int16_t value, uint8_t *servoError);
uint8_t setGoalTime(PORT_HANDLE h, const uint8_t ID, int16_t value, uint8_t *servoError);
uint8_t setGoalSpeed(PORT_HANDLE h, const uint8_t ID, int16_t value, uint8_t *servoError);

/* setSave*: EEPROM only, persists across power-off */

/* 1 byte */
uint8_t setSaveBaudRate(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveReturnDelay(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveResponseLevel(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveMaxTemperature(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveMaxVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveMinVoltage(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSavePhase(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveUnloadingCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveLedAlarmCondition(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSavePGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveDGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveCwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveCcwDeadZone(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveAngularResolution(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveOperatingMode(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveProtectiveTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveProtectionTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveOverloadTorque(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveSpeedPGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveOvercurrentTime(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);
uint8_t setSaveSpeedIGain(PORT_HANDLE h, const uint8_t ID, uint8_t value, uint8_t *servoError);

/* 2 bytes, unsigned, low byte first */
uint8_t setSaveMinAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setSaveMaxAngleLimit(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setSaveMaxTorque(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setSaveMinStartupForce(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);
uint8_t setSaveProtectionCurrent(PORT_HANDLE h, const uint8_t ID, uint16_t value, uint8_t *servoError);

/* 2 bytes, sign-magnitude */
uint8_t setSavePositionOffset(PORT_HANDLE h, const uint8_t ID, int16_t value, uint8_t *servoError);

/* instructions */

uint8_t ping(PORT_HANDLE h, const uint8_t ID, uint8_t *servoError);

/* synced getters: one SYNC_READ for count servos. value[i] belongs to IDs[i].
   a failing servo doesn't stop the rest; results and servoErrors hold count
   entries each:
     results[i] == 0              value[i] valid
     results[i] == SERVO_ERROR    value[i] valid, servoErrors[i] is the servo's error byte
     results[i] == PACKET_TIMEOUT no valid reply (silent or corrupted), value[i] untouched
   returns 0 if every servo succeeded, else the first nonzero results[i] */

/* 1 byte */
uint8_t getSyncedTorqueEnable(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedAcceleration(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentVoltage(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentTemperature(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedStatus(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedMoving(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint8_t *value, uint8_t *results, uint8_t *servoErrors);

/* 2 bytes, unsigned, low byte first */
uint8_t getSyncedTorqueLimit(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentCurrent(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, uint16_t *value, uint8_t *results, uint8_t *servoErrors);

/* 2 bytes, sign-magnitude */
uint8_t getSyncedGoalPosition(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedGoalTime(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedGoalSpeed(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentPosition(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentSpeed(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);
uint8_t getSyncedPresentLoad(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, int16_t *value, uint8_t *results, uint8_t *servoErrors);

/* synced setters: one SYNC_WRITE, values[i] goes to IDs[i], no reply. RAM
   registers only. RANGE_ERROR if any value is out of range (nothing is sent),
   INCORRECT_WRITE_PACKET_SIZE if count servos don't fit in one packet */
uint8_t setSyncedTorqueEnable(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const uint8_t *values);
uint8_t setSyncedAcceleration(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const uint8_t *values);
uint8_t setSyncedTorqueLimit(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const uint16_t *values);
uint8_t setSyncedGoalPosition(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const int16_t *values);
uint8_t setSyncedGoalTime(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const int16_t *values);
uint8_t setSyncedGoalSpeed(PORT_HANDLE h, const uint8_t count, uint8_t *IDs, const int16_t *values);

