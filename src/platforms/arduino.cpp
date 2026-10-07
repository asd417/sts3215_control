
#if ARDUINO
#include "Arduino.h"
#include "HardwareSerial.h"
#include "driver_errors.h"
#include "platform.h"
#include "register.h"
#include <assert.h>

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
PORT_HANDLE openPort(uint32_t baudRate, const char *name) {
  Serial.begin(baudRate);
  return &Serial;
}
#endif

int sendPacket(uint8_t *outgoing, const uint8_t size, PORT_HANDLE h) {
  while (h->available())
    h->read(); // clear
  if (h->availableForWrite() >= size)
    h->write(outgoing, size);
  else
    return PACKET_SEND_FAILED;
  return 0;
}

int readPacket(uint8_t *buffer, const uint8_t size, uint32_t timeout,
               PORT_HANDLE h) {
  h->setTimeout(timeout);
  size_t s = h->readBytes(buffer, size);
  if (s != size)
    return PACKET_TIMEOUT;
  return 0;
}

int setPlatformBaudRate(uint8_t baudIndex, PORT_HANDLE h) {
  h->flush();
  uint32_t b = baudFromIndex(baudIndex);
  if (b) {
    h->begin(b);
    return 0;
  } else
    return 1;
}

#endif