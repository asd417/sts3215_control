
#if ARDUINO
#include "Arduino.h"
#include <assert.h>
#include "HardwareSerial.h"
#include "platform.h"
#include "driver_errors.h"
#include "../register.h"

#define TXPACKET_MAX_LEN 250
#define RCPACKET_MAX_LEN 250

static uint8_t tx[TXPACKET_MAX_LEN];
static uint8_t rx[RCPACKET_MAX_LEN];

uint8_t *getPacketTX(uint8_t *sizeOut) {
  *sizeOut = TXPACKET_MAX_LEN;
  return tx;
}

uint8_t *getPacketRX(uint8_t *sizeOut) {
  *sizeOut = RCPACKET_MAX_LEN;
  return rx;
}

#ifdef __AVR_ATmega328P__ // only 1 UART port
int openPort(uint32_t baudRate, PORT_HANDLE* out, const char *name) {
  Serial.begin(baudRate);
  *out = &Serial;
  return 0;
}
#endif

int sendPacket(uint8_t *outgoing, const uint8_t size, PORT_HANDLE h) {
  while (h->available())
    h->read(); // clear
  if (h->availableForWrite() >= size)
    h->write(outgoing, size);
  else
    return PF_SEND_FAIL;
  return 0;
}

int readPacket(uint8_t *buffer, const uint8_t size, uint32_t timeout,
               PORT_HANDLE h) {
  h->setTimeout(timeout);
  size_t s = h->readBytes(buffer, size);
  if (s != size)
    return PF_TIMEOUT_FAIL;
  return 0;
}

int setPlatformBaudRate(uint8_t baudIndex, PORT_HANDLE h) {
  h->flush();
  uint32_t b = baudFromIndex(baudIndex);
  if (b) {
    h->begin(b);
    return 0;
  } else
    return PF_BAUD_SET_FAIL;
}

#endif