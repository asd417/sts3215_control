// windows test program, servo expected at 1 Mbaud.
//   main                  packet checks only, no hardware needed
//   main COM4 [id]        ping and read the servo's state, read only (id defaults to 1)
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <random>

#include "driver.h"
#include "packet.h"
#include "register.h"

static void checkPackets() {
  uint8_t b[32];

  createPingPacket(b, 1);
  const uint8_t ping_[] = {0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB};
  assert(!memcmp(b, ping_, sizeof ping_));

  createReadPacket(b, 1, 0x38, 2); // present position
  const uint8_t read_[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE};
  assert(!memcmp(b, read_, sizeof read_));

  uint8_t cmd[] = {1, 0x00, 0x08, 2, 0x00, 0x08}; // IDs 1 and 2 -> 2048
  createSyncWritePacket(b, 2, 0x2A, 2, cmd);
  const uint8_t sync_[] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x2A, 0x02,
                           0x01, 0x00, 0x08, 0x02, 0x00, 0x08, 0x35};
  assert(!memcmp(b, sync_, sizeof sync_));

  uint8_t status[] = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x00, 0x08, 0xF2};
  uint8_t id, err, *p;
  assert(parseSinglePacket(status, sizeof status, 2, &id, &err, &p) == 0);
  assert(id == 1 && err == 0 && p[0] == 0x00 && p[1] == 0x08);
  status[7] ^= 1; // corrupt the checksum
  assert(parseSinglePacket(status, sizeof status, 2, &id, &err, &p) == CHECKSUM_ERROR);

  assert(baudFromIndex(0) == 1000000 && baudFromIndex(4) == 115200);
  assert(baudFromIndex(7) == 38400 && baudFromIndex(8) == 0);
}

// prints one reading; result is a driver_errors.h code, 0 = ok
static int show(const char *name, int result, long value, int servoError) {
  if (result)
    printf("%-20s FAILED: result %d, servo error 0x%02X\n", name, result, servoError);
  else
    printf("%-20s %ld\n", name, value);
  return result;
}

int main(int argc, char **argv) {
  checkPackets();
  printf("packet checks ok\n");
  if (argc < 2)
    return 0;

  uint8_t id = argc > 2 ? (uint8_t)atoi(argv[2]) : 1;
  PORT_HANDLE h = openPort(1000000, argv[1]);
  if (!h)
    return 1;

  int se = 0;
  int e = ping(h, id, &se);
  show("ping", e, id, se);
  if (e) { // nothing else will work either
    closePort(h);
    return e;
  }

  // read only
  uint8_t u8 = 0;
  uint16_t u16 = 0;
  int16_t s16 = 0;
  // read into r first: argument evaluation order is unspecified, so passing
  // the getter and its output to show() together can print the stale value
  int r;
  r = getModelNumber(h, id, &u16, &se);
  show("model number", r, u16, se);
  r = getFirmwareMajor(h, id, &u8, &se);
  show("firmware major", r, u8, se);
  r = getBaudRate(h, id, &u8, &se);
  show("baud (bps)", r, baudFromIndex(u8), se);
  r = getPresentVoltage(h, id, &u8, &se);
  show("voltage (0.1 V)", r, u8, se);
  r = getPresentTemperature(h, id, &u8, &se);
  show("temperature (C)", r, u8, se);
  r = getTorqueEnable(h, id, &u8, &se);
  show("torque enable", r, u8, se);
  r = getPresentPosition(h, id, &s16, &se);
  show("present position", r, s16, se);
  r = getPGain(h, id, &u8, &se);
  show("P gain", r, u8, se);
  r = getIGain(h, id, &u8, &se);
  show("I gain", r, u8, se);
  r = getDGain(h, id, &u8, &se);
  show("D gain", r, u8, se);

  r = setPGain(h, id, 20, &se);
  show("set P gain", r, u8, se);
  r = getPGain(h, id, &u8, &se);
  show("P gain", r, u8, se);

  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, 4095); // both ends inclusive
  int target = dist(rng);
  setGoalPosition(h, 1, target, &se);
  int16_t pos;
  r = getPresentPosition(h, id, &pos, &se);
  int16_t delta = 1;
  while (delta != 0 || abs(target - pos)>2)
  {
    int16_t newp = 0;
    r = getPresentPosition(h, id, &newp, &se);
    delta = newp - pos;
    pos = newp;
    show("present position", r, pos, se);
    show("  position delta", r, delta, se);
  }
  std::cout << "done" << std::endl;

  closePort(h);
  return 0;
}
