#pragma once
#include <stdint.h>

#ifdef _WIN32
typedef void *PORT_HANDLE;
#endif
#ifdef ARDUINO

#endif

//get the pointer to the packet buffer
uint8_t* getPacketTX(uint8_t* sizeOut);
//send the TX packet
int sendPacket(PORT_HANDLE h, uint8_t* outgoing, const uint8_t size);
//get the pointer to the packet buffer
uint8_t* getPacketRX(uint8_t* sizeOut);
int readRX(PORT_HANDLE h, uint8_t *buffer, const uint8_t size, uint32_t timeout);
int setPlatformBaudRate(PORT_HANDLE h, uint8_t baudIndex);

PORT_HANDLE openPort(const char* name, uint32_t baudRate );
void closePort(PORT_HANDLE h);