#pragma once
#include <stdint.h>

#include "platforms/platform.h"

// internal helpers shared by driver.cpp, not part of the public API.
// error returns follow driver.h: 0 or a driver_errors.h code

//  timeout = reply_bytes * time_per_byte + return_delay + platform_latency +
//  margin
// ponytail: fixed worst case, compute from the formula above if the bus needs
// tighter timing
#define READ_TIMEOUT_MS 50

// where the ID/data section of a sync write goes in the TX buffer; nullptr if
// count entries of size bytes don't fit
uint8_t *syncWriteBuffer(const uint8_t count, const uint8_t size);

// on success *parms points at `size` data bytes in the RX buffer (valid until
// the next read)
uint8_t readRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                     const uint8_t size, uint8_t **parms, uint8_t *servoError);
uint8_t writeRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                      const uint8_t size, uint8_t **parms, uint8_t *servoError);
// unlock, write, relock
uint8_t writeEEPROM(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                    const uint8_t size, uint8_t **parms, uint8_t *servoError);
// changes the servo's baud and the host's together, reverting the host on failure
uint8_t writeBaud(PORT_HANDLE h, const uint8_t ID, uint8_t index,
                  const bool save, uint8_t *servoError);
// read one status packet carrying `size` data bytes from servo ID
uint8_t readStatus(PORT_HANDLE h, const uint8_t ID, const uint8_t size,
                   uint8_t **parms, uint8_t *servoError);

// raw instructions. addr/size are the ADDR_x / SIZE_x pairs from register.h,
// data is raw register bytes (low byte first, see convert.h). the typed
// getSynced*/setSynced* in driver.h wrap syncRead/syncWrite

// buffered write: the servo holds it until action()
uint8_t regWrite(PORT_HANDLE h, const uint8_t ID, const uint8_t addr, const uint8_t size,
             uint8_t *data, uint8_t *servoError);
// broadcast: every servo runs its pending regWrite. no reply
uint8_t action(PORT_HANDLE h);

// commandBuffer is ID1 d1..dn ID2 d1..dn ..., count entries of size+1 bytes. no reply
uint8_t syncWrite(PORT_HANDLE h, const uint8_t addr, const uint8_t size, const uint8_t count,
              uint8_t *commandBuffer);
// out receives count*size bytes, servo i's data at out[i*size]. a failing servo
// doesn't stop the rest; results and servoErrors hold count entries each:
//   results[i] == 0              data valid
//   results[i] == SERVO_ERROR    data valid, servoErrors[i] is the servo's error byte
//   results[i] == PACKET_TIMEOUT no valid reply (silent or corrupted), out untouched
// returns 0 if every servo succeeded, else the first nonzero results[i]
uint8_t syncRead(PORT_HANDLE h, const uint8_t addr, const uint8_t size, const uint8_t count,
             uint8_t *IDs, uint8_t *out, uint8_t *results, uint8_t *servoErrors);
