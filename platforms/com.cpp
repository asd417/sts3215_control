#include <windows.h>
#include <string>
int openCOM(std::string name)
{
    name = "\\\\.\\" + name;
    HANDLE h = CreateFileA(name.c_str(),                   // \\.\COM3 at runtime
                           GENERIC_READ | GENERIC_WRITE,   // read and write
                           0,                              // no sharing (required for COM)
                           NULL,                           // default security
                           OPEN_EXISTING,                  // device must already exist
                           0,                              // normal blocking I/O
                           NULL);

    if (h == INVALID_HANDLE_VALUE) {
      printf("open failed, error %lu\n", GetLastError());
      return 1;
    }
    printf("opened %s", name.c_str());
    // line format: 1 Mbps, 8N1, no flow control
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(h, &dcb);
    dcb.BaudRate = 1000000;
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
      return 1;
    }
    // ReadFile gives up after 50 ms total (all values in milliseconds)
    COMMTIMEOUTS t = {0};
    t.ReadIntervalTimeout = 0;
    t.ReadTotalTimeoutConstant = 50;
    t.ReadTotalTimeoutMultiplier = 0;
    t.WriteTotalTimeoutConstant = 50;
    SetCommTimeouts(h, &t);

    // PING servo 1: FF FF ID LEN INST CHECKSUM, checksum = ~(1 + 2 + 1) = 0xFB
    uint8_t ping[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};

    PurgeComm(h, PURGE_RXCLEAR);  // drop stale bytes before sending
    DWORD written = 0;
    if (!WriteFile(h, ping, sizeof(ping), &written, NULL) || written != sizeof(ping)) {
      printf("write failed, error %lu\n", GetLastError());
      return 1;
    }

    // one ReadFile may return fewer bytes than asked: keep reading until full or timed out
    uint8_t reply[6];
    DWORD got = 0;
    while (got < sizeof(reply)) {
      DWORD n = 0;
      if (!ReadFile(h, reply + got, sizeof(reply) - got, &n, NULL)) {
        printf("read failed, error %lu\n", GetLastError());
        return 1;
      }
      if (n == 0) break;  // timeout: nothing more arrived
      got += n;
    }

    printf("got %lu bytes:", got);
    for (DWORD i = 0; i < got; i++) printf(" %02X", reply[i]);
    printf("\n");
    return 0;
}

void closeCOM(HANDLE h)
{
    CloseHandle(h);
}   