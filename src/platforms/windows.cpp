#if _WIN32
#include "../register.h"
#include "platform.h"

#include <cassert>
#include <string>
#include <windows.h>

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

PORT_HANDLE openPort(uint32_t baudRate, const char* name) {
  std::string name_s = std::string(name);
  name_s = "\\\\.\\" + name_s;
  PORT_HANDLE h = CreateFileA(name_s.c_str(), // \\.\COM3 at runtime
                              GENERIC_READ | GENERIC_WRITE, // read and write
                              0,             // no sharing (required for COM)
                              NULL,          // default security
                              OPEN_EXISTING, // device must already exist
                              0,             // normal blocking I/O
                              NULL);

  if (h == INVALID_HANDLE_VALUE) {
    printf("open failed, error %lu\n", GetLastError());
    return nullptr;
  }
  DCB dcb = {0};
  dcb.DCBlength = sizeof(DCB);
  GetCommState(h, &dcb);
  dcb.BaudRate = (DWORD)baudRate; // 1000000
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  if (!SetCommState(h, &dcb)) {
    printf("SetCommState failed, error %lu\n", GetLastError());
    CloseHandle(h);
    return nullptr;
  }
  // ReadFile gives up after 50 ms total (all values in milliseconds)
  COMMTIMEOUTS t = {0};
  t.ReadIntervalTimeout = 0;
  t.ReadTotalTimeoutConstant = 50;
  t.ReadTotalTimeoutMultiplier = 0;
  t.WriteTotalTimeoutConstant = 50;
  if (!SetCommTimeouts(h, &t)) {
    printf("SetCommTimeouts failed, error %lu\n", GetLastError());
    CloseHandle(h);
    return nullptr;
  }

  return h;
}
void closePort(PORT_HANDLE h) { CloseHandle(h); }
int sendPacket(uint8_t *outgoing, const uint8_t size, PORT_HANDLE h) {
  //assert(h != nullptr && "PORT_HANDLE can not be nullptr");
  PurgeComm(h, PURGE_RXCLEAR); // drop stale bytes before sending
  DWORD written = 0;
  if (!WriteFile(h, outgoing, size, &written, NULL) || written != size) {
    printf("write failed, error %lu\n", GetLastError());
    return 1;
  }
  return 0;
}
int readPacket(uint8_t *buffer, const uint8_t size, uint32_t timeout,
           PORT_HANDLE h) {
  //assert(h != nullptr && "PORT_HANDLE can not be nullptr");
  // each ReadFile call waits at most `timeout` ms
  COMMTIMEOUTS t = {0};
  t.ReadTotalTimeoutConstant = timeout;
  t.WriteTotalTimeoutConstant = 50;
  if (!SetCommTimeouts(h, &t)) {
    printf("SetCommTimeouts failed, error %lu\n", GetLastError());
    return 1;
  }
  DWORD got = 0;
  while (got < size) {
    DWORD n = 0;
    if (!ReadFile(h, buffer + got, size - got, &n, NULL)) {
      printf("read failed, error %lu\n", GetLastError());
      return 1;
    }
    if (n == 0)
      return 1; // timeout: servo sent fewer than size bytes
    got += n;
  }
  return 0;
}

int setPlatformBaudRate(uint8_t baudIndex, PORT_HANDLE h) {
  //assert(h != nullptr && "PORT_HANDLE can not be nullptr");
  DCB dcb = {0};
  dcb.DCBlength = sizeof(DCB);
  GetCommState(h, &dcb);
  dcb.BaudRate = (DWORD)baudFromIndex(baudIndex); // 1000000
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  if (!SetCommState(h, &dcb)) {
    printf("SetCommState failed, error %lu\n", GetLastError());
    return 1; // port stays open at its old rate
  }
  return 0;
}

#endif