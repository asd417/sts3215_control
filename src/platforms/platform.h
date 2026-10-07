#pragma once
#include <stdint.h>

#ifdef _WIN32
typedef void *PORT_HANDLE;
#endif
#ifdef ARDUINO
#include <Arduino.h> 
typedef HardwareSerial *PORT_HANDLE;
#endif

PORT_HANDLE openPort(uint32_t baudRate, const char *name = nullptr);
void closePort(PORT_HANDLE h);

//get the pointer to the packet buffer
uint8_t* getPacketTX(uint8_t* sizeOut);
//get the pointer to the packet buffer
uint8_t* getPacketRX(uint8_t* sizeOut);
//send the TX packet
int sendPacket(uint8_t* outgoing, const uint8_t size, PORT_HANDLE h);
int readPacket(uint8_t *buffer, const uint8_t size, uint32_t timeout, PORT_HANDLE h);
int setPlatformBaudRate(uint8_t baudIndex, PORT_HANDLE h);
