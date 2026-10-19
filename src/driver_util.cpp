#include "driver_util.h"
#include "convert.h"
#include "driver.h"
#include "driver_errors.h"
#include "packet.h"
#include "platforms/platform.h"
#include "register.h"
#include "servo_errors.h"

// the ID/data section of a sync write is assembled straight in the TX buffer,
// where createSyncWritePacket puts it anyway (its copy is then a no-op), so no
// second buffer is needed. nullptr if count entries of size bytes don't fit
uint8_t *syncWriteBuffer(const uint8_t count, const uint8_t size) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (PACKET_SIZE_SYNC_WRITE_BASE + (uint16_t)count * (size + 1) > w_size)
    return nullptr;
  return w + PACKET_SIZE_SYNC_WRITE_BASE - 1; // after FF FF FE LEN INST ADDR N
}

// shared read flow: on success *parms points at `size` data bytes in the RX
// buffer (valid until the next read)
uint8_t readRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                        const uint8_t size, uint8_t **parms, uint8_t *servoError) {
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
  uint8_t error = parseSinglePacket(r, r_size, size, &outID, &outError, parms);
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

uint8_t writeRegister(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                         const uint8_t size, uint8_t **parms, uint8_t *servoError) {
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
  uint8_t error = parseSinglePacket(r, r_size, 0, &outID, &outError, nullptr);
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

// unlock, write, relock. relocks even if the write failed so the servo isn't
// left unlocked; the write's error wins over the relock's
uint8_t writeEEPROM(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
                       const uint8_t size, uint8_t **parms, uint8_t *servoError) {
  uint8_t lock = 0;
  uint8_t *l = &lock;
  uint8_t error = writeRegister(h, ID, ADDR_LOCK, SIZE_LOCK, &l, servoError);
  if (error)
    return error;
  error = writeRegister(h, ID, addr, size, parms, servoError);
  lock = 1;
  uint8_t relock = writeRegister(h, ID, ADDR_LOCK, SIZE_LOCK, &l, servoError);
  return error ? error : relock;
}

// baud needs to be written in a special way because it also needs to be change
// on the platform side before reading the response from the motor
uint8_t writeBaud(PORT_HANDLE h, const uint8_t ID, uint8_t index,
                     const bool save, uint8_t *servoError) {
  // broadcast can't be read back or verified
  if (ID == 0xFE || !baudFromIndex(index))
    return INVALID_ARGUMENT;
  uint8_t old;
  uint8_t error = getBaudRate(h, ID, &old, servoError);
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

// read one status packet carrying `size` data bytes from servo ID
uint8_t readStatus(PORT_HANDLE h, const uint8_t ID, const uint8_t size,
                      uint8_t **parms, uint8_t *servoError) {
  *servoError = 0; // start clean
  uint8_t r_size;
  uint8_t *r = getPacketRX(&r_size);
  if (r_size < PACKET_SIZE_OVERHEAD + size)
    return INCORRECT_READ_PACKET_SIZE;
  if (readPacket(r, PACKET_SIZE_OVERHEAD + size, READ_TIMEOUT_MS, h))
    return PACKET_TIMEOUT;
  uint8_t outID;
  uint8_t outError;
  uint8_t error = parseSinglePacket(r, r_size, size, &outID, &outError, parms);
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

uint8_t regWrite(PORT_HANDLE h, const uint8_t ID, const uint8_t addr,
             const uint8_t size, uint8_t *data, uint8_t *servoError) {
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

uint8_t action(PORT_HANDLE h) {
  uint8_t w_size;
  uint8_t *w = getPacketTX(&w_size);
  if (w_size < PACKET_SIZE_ACTION)
    return INCORRECT_WRITE_PACKET_SIZE;
  createActionPacket(w, 0xFE);
  return sendPacket(w, PACKET_SIZE_ACTION, h) ? PACKET_SEND_FAILED : 0;
}

uint8_t syncWrite(PORT_HANDLE h, const uint8_t addr, const uint8_t size,
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

uint8_t syncRead(PORT_HANDLE h, const uint8_t addr, const uint8_t size,
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
