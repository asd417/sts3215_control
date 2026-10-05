// offline test: runs the real driver.cpp against a simulated bus of three STS
// servos (IDs 1, 2, 3). no hardware is touched.
// this file IS the platform, so build it without platforms/windows.cpp:
//   g++ -std=c++11 test_driver.cpp driver.cpp packet.cpp convert.cpp register.cpp -o test_driver.exe
//
// the simulated servo checks every packet the driver sends (header, length,
// checksum, and that a write covers exactly one or more whole writable
// registers) and the tests check that each call changed only the bytes it
// should have.
#include <stdio.h>
#include <string.h>
#include <vector>

#include "driver.h"
#include "register.h"

/* ---- memory table typed in from the Feetech STS documentation, deliberately
        NOT taken from register.h, so a wrong address there gets caught ---- */
struct Reg {
  const char *name;
  uint8_t addr, size;
  bool writable;
};
static const Reg TABLE[] = {
    {"FIRMWARE_MAJOR", 0, 1, false},     {"FIRMWARE_MINOR", 1, 1, false},
    {"MODEL_NUM", 3, 2, false},          {"ID", 5, 1, true},
    {"BAUD_RATE", 6, 1, true},           {"RETURN_DELAY", 7, 1, true},
    {"RESPONSE_LEVEL", 8, 1, true},      {"MIN_ANGLE_LIMIT", 9, 2, true},
    {"MAX_ANGLE_LIMIT", 11, 2, true},    {"MAX_TEMPERATURE", 13, 1, true},
    {"MAX_VOLTAGE", 14, 1, true},        {"MIN_VOLTAGE", 15, 1, true},
    {"MAX_TORQUE", 16, 2, true},         {"PHASE", 18, 1, true},
    {"UNLOADING_CONDITION", 19, 1, true}, {"LED_ALARM_CONDITION", 20, 1, true},
    {"P_GAIN", 21, 1, true},             {"D_GAIN", 22, 1, true},
    {"I_GAIN", 23, 1, true},             {"MIN_STARTUP_FORCE", 24, 2, true},
    {"CW_DEAD_ZONE", 26, 1, true},       {"CCW_DEAD_ZONE", 27, 1, true},
    {"PROTECTION_CURRENT", 28, 2, true}, {"ANGULAR_RESOLUTION", 30, 1, true},
    {"POSITION_OFFSET", 31, 2, true},    {"OPERATING_MODE", 33, 1, true},
    {"PROTECTIVE_TORQUE", 34, 1, true},  {"PROTECTION_TIME", 35, 1, true},
    {"OVERLOAD_TORQUE", 36, 1, true},    {"SPEED_P_GAIN", 37, 1, true},
    {"OVERCURRENT_TIME", 38, 1, true},   {"SPEED_I_GAIN", 39, 1, true},
    {"TORQUE_ENABLE", 40, 1, true},      {"ACCELERATION", 41, 1, true},
    {"GOAL_POSITION", 42, 2, true},      {"GOAL_TIME", 44, 2, true},
    {"GOAL_SPEED", 46, 2, true},         {"TORQUE_LIMIT", 48, 2, true},
    {"LOCK", 55, 1, true},               {"PRESENT_POSITION", 56, 2, false},
    {"PRESENT_SPEED", 58, 2, false},     {"PRESENT_LOAD", 60, 2, false},
    {"PRESENT_VOLTAGE", 62, 1, false},   {"PRESENT_TEMPERATURE", 63, 1, false},
    {"STATUS", 65, 1, false},            {"MOVING", 66, 1, false},
    {"PRESENT_CURRENT", 69, 2, false},
};
static const int TABLE_N = sizeof TABLE / sizeof TABLE[0];

struct DriverReg {
  const char *name;
  uint8_t addr, size;
};
#define AS_ROW(name, addr, size) {#name, addr, size},
static const DriverReg DRIVER_TABLE[] = {REGISTERS(AS_ROW)};
static const int DRIVER_TABLE_N = sizeof DRIVER_TABLE / sizeof DRIVER_TABLE[0];

/* ---- simulated bus ---- */
#define MEM 72
#define EEPROM_END 40 // addresses below this are EEPROM
#define A_ID 5
#define A_BAUD 6
#define A_LOCK 55

struct Servo {
  bool present;
  uint8_t mem[MEM];   // what the servo is running with now
  uint8_t saved[MEM]; // what survives power-off (written only while unlocked)
  uint8_t err;        // error byte it puts in every reply
  bool pending;       // REG_WRITE waiting for ACTION
  uint8_t pAddr, pSize, pData[8];
};
struct Write {
  uint8_t id, addr, size, data[8];
};

static Servo servos[3];
static std::vector<Write> writes; // every write the servos accepted, in order
static std::vector<uint8_t> rxq;
static size_t rxHead;
static uint8_t hostBaud; // baud index the host port is set to
static int violations;
static int packetsSent;
// behaviours real hardware may or may not have; the tests try both
static bool replyNewIdAfterIdWrite;
static bool baudReplyAtNewRate;
static bool ignoreBaudWrite; // a servo that refuses to change rate
static bool ignoreIdWrite;   // a servo that refuses to change ID
static int corruptRepliesFromId = -1;

static void violation(const char *msg) {
  violations++;
  printf("  PROTOCOL VIOLATION: %s\n", msg);
}

static void resetBus() {
  for (int i = 0; i < 3; i++) {
    Servo &s = servos[i];
    memset(&s, 0, sizeof s);
    s.present = true;
    s.mem[A_ID] = (uint8_t)(i + 1);
    s.mem[A_LOCK] = 1;
    memcpy(s.saved, s.mem, MEM);
  }
  writes.clear();
  rxq.clear();
  rxHead = 0;
  hostBaud = 0;
  replyNewIdAfterIdWrite = baudReplyAtNewRate = ignoreBaudWrite = ignoreIdWrite = false;
  corruptRepliesFromId = -1;
}

// true if [addr, addr+n) is exactly one or more whole registers
static bool wholeRegisters(uint8_t addr, uint8_t n, bool needWritable) {
  int a = addr, end = addr + n;
  if (n == 0)
    return false;
  while (a < end) {
    const Reg *r = nullptr;
    for (int i = 0; i < TABLE_N; i++)
      if (TABLE[i].addr == a)
        r = &TABLE[i];
    if (!r || (needWritable && !r->writable) || a + r->size > end)
      return false;
    a += r->size;
  }
  return true;
}

static void pushReply(const Servo &s, uint8_t id, const uint8_t *data, uint8_t n) {
  uint8_t sum = (uint8_t)(id + (n + 2) + s.err);
  rxq.push_back(0xFF);
  rxq.push_back(0xFF);
  rxq.push_back(id);
  rxq.push_back((uint8_t)(n + 2));
  rxq.push_back(s.err);
  for (uint8_t i = 0; i < n; i++) {
    rxq.push_back(data[i]);
    sum = (uint8_t)(sum + data[i]);
  }
  uint8_t cs = (uint8_t)~sum;
  if (corruptRepliesFromId == id)
    cs ^= 0x01;
  rxq.push_back(cs);
}

static void applyWrite(Servo &s, uint8_t addr, uint8_t n, const uint8_t *data) {
  if (n > 8 || !wholeRegisters(addr, n, true)) {
    violation("write does not cover whole writable registers");
    return;
  }
  Write w;
  w.id = s.mem[A_ID];
  w.addr = addr;
  w.size = n;
  memcpy(w.data, data, n);
  writes.push_back(w);
  for (uint8_t i = 0; i < n; i++) {
    uint8_t a = (uint8_t)(addr + i);
    if ((a == A_BAUD && ignoreBaudWrite) || (a == A_ID && ignoreIdWrite))
      continue;
    s.mem[a] = data[i];
    if (a < EEPROM_END && s.mem[A_LOCK] == 0)
      s.saved[a] = data[i];
  }
}

static void handle(Servo &s, uint8_t inst, const uint8_t *p, uint8_t np, bool broadcast) {
  switch (inst) {
  case 1: // PING
    if (np != 0)
      violation("ping with parameters");
    if (!broadcast)
      pushReply(s, s.mem[A_ID], nullptr, 0);
    break;
  case 2: // READ
    if (np != 2 || broadcast || !wholeRegisters(p[0], p[1], false)) {
      violation("bad read (broadcast, wrong size, or not whole registers)");
      break;
    }
    pushReply(s, s.mem[A_ID], &s.mem[p[0]], p[1]);
    break;
  case 3: { // WRITE
    if (np < 2) {
      violation("write without data");
      break;
    }
    uint8_t oldId = s.mem[A_ID], oldBaud = s.mem[A_BAUD];
    applyWrite(s, p[0], (uint8_t)(np - 1), p + 1);
    if (broadcast)
      break;
    uint8_t replyBaud = baudReplyAtNewRate ? s.mem[A_BAUD] : oldBaud;
    if (replyBaud == hostBaud) // otherwise the host hears nothing usable
      pushReply(s, replyNewIdAfterIdWrite ? s.mem[A_ID] : oldId, nullptr, 0);
    break;
  }
  case 4: // REG_WRITE
    if (np < 2 || np - 1 > 8 || !wholeRegisters(p[0], (uint8_t)(np - 1), true)) {
      violation("bad reg write");
      break;
    }
    s.pending = true;
    s.pAddr = p[0];
    s.pSize = (uint8_t)(np - 1);
    memcpy(s.pData, p + 1, s.pSize);
    if (!broadcast)
      pushReply(s, s.mem[A_ID], nullptr, 0);
    break;
  case 5: // ACTION
    if (np != 0)
      violation("action with parameters");
    if (s.pending)
      applyWrite(s, s.pAddr, s.pSize, s.pData);
    s.pending = false;
    break;
  default:
    violation("unknown instruction");
  }
}

/* ---- platform.h, implemented by the simulator ---- */
static uint8_t txbuf[250], rxbuf[250];
uint8_t *getPacketTX(uint8_t *sizeOut) {
  *sizeOut = sizeof txbuf;
  return txbuf;
}
uint8_t *getPacketRX(uint8_t *sizeOut) {
  *sizeOut = sizeof rxbuf;
  return rxbuf;
}
PORT_HANDLE openPort(const char *, uint32_t) { return (PORT_HANDLE)1; }
void closePort(PORT_HANDLE) {}
int setPlatformBaudRate(PORT_HANDLE, uint8_t baudIndex) {
  if (!baudFromIndex(baudIndex))
    return 1;
  hostBaud = baudIndex;
  return 0;
}

int sendPacket(PORT_HANDLE, uint8_t *out, const uint8_t size) {
  packetsSent++;
  rxq.clear();
  rxHead = 0;
  if (size < 6 || out[0] != 0xFF || out[1] != 0xFF) {
    violation("bad header or packet too short");
    return 0;
  }
  if (out[3] != size - 4) {
    violation("LEN byte does not match the packet size");
    return 0;
  }
  uint8_t sum = 0;
  for (int i = 2; i < size - 1; i++)
    sum = (uint8_t)(sum + out[i]);
  if ((uint8_t)~sum != out[size - 1]) {
    violation("bad checksum");
    return 0;
  }
  uint8_t id = out[2], inst = out[4], np = (uint8_t)(out[3] - 2);
  const uint8_t *p = out + 5;
  bool broadcast = id == 0xFE;

  if (inst == 0x82) { // SYNC_READ: addr, n, id1..idk -> replies in that order
    if (!broadcast || np < 3 || !wholeRegisters(p[0], p[1], false)) {
      violation("bad sync read");
      return 0;
    }
    for (int k = 2; k < np; k++)
      for (int i = 0; i < 3; i++) {
        Servo &s = servos[i];
        if (s.present && s.mem[A_BAUD] == hostBaud && s.mem[A_ID] == p[k])
          pushReply(s, s.mem[A_ID], &s.mem[p[0]], p[1]);
      }
    return 0;
  }
  if (inst == 0x83) { // SYNC_WRITE: addr, n, then (id, n data bytes) per servo
    if (!broadcast || np < 2 || (np - 2) % (p[1] + 1) != 0) {
      violation("bad sync write");
      return 0;
    }
    for (int k = 2; k < np; k += p[1] + 1)
      for (int i = 0; i < 3; i++) {
        Servo &s = servos[i];
        if (s.present && s.mem[A_BAUD] == hostBaud && s.mem[A_ID] == p[k])
          applyWrite(s, p[0], p[1], p + k + 1);
      }
    return 0;
  }
  for (int i = 0; i < 3; i++) {
    Servo &s = servos[i];
    if (s.present && s.mem[A_BAUD] == hostBaud && (broadcast || s.mem[A_ID] == id))
      handle(s, inst, p, np, broadcast);
  }
  return 0;
}

int readRX(PORT_HANDLE, uint8_t *buffer, const uint8_t size, uint32_t) {
  if (rxq.size() - rxHead < size)
    return 1; // timeout
  memcpy(buffer, &rxq[rxHead], size);
  rxHead += size;
  return 0;
}

/* ---- test harness ---- */
static int checks, failed;
static const char *ctx = "";
#define CHECK(cond)                                                            \
  do {                                                                         \
    checks++;                                                                  \
    if (!(cond)) {                                                             \
      failed++;                                                                \
      printf("FAIL line %d [%s]: %s\n", __LINE__, ctx, #cond);                 \
    }                                                                          \
  } while (0)

static const PORT_HANDLE H = (PORT_HANDLE)1;
static int se;
static Servo before[3];
static int violationsBefore;

static void begin(const char *name) {
  ctx = name;
  resetBus();
  se = 0;
  memcpy(before, servos, sizeof servos);
  violationsBefore = violations;
}

// after a setter on servo 1: exactly `bytes` landed at addr, nothing else moved
static void expectWrite(int result, uint8_t addr, const uint8_t *bytes, uint8_t n,
                        bool save) {
  Servo &s = servos[0];
  CHECK(result == 0);
  CHECK(violations == violationsBefore);
  uint8_t want[MEM];
  memcpy(want, before[0].mem, MEM);
  memcpy(want + addr, bytes, n);
  CHECK(memcmp(s.mem, want, MEM) == 0); // only the target bytes changed
  CHECK(memcmp(servos[1].mem, before[1].mem, MEM) == 0); // other servos untouched
  CHECK(memcmp(servos[2].mem, before[2].mem, MEM) == 0);
  if (!save) {
    CHECK(writes.size() == 1 && writes[0].addr == addr && writes[0].size == n);
    if (addr < EEPROM_END)
      CHECK(memcmp(s.saved, before[0].saved, MEM) == 0); // locked: not persisted
  } else {
    CHECK(writes.size() == 3);
    if (writes.size() == 3) { // unlock, write, relock
      CHECK(writes[0].addr == A_LOCK && writes[0].size == 1 && writes[0].data[0] == 0);
      CHECK(writes[1].addr == addr && writes[1].size == n);
      CHECK(writes[2].addr == A_LOCK && writes[2].size == 1 && writes[2].data[0] == 1);
    }
    CHECK(s.mem[A_LOCK] == 1);
    CHECK(memcmp(s.saved + addr, bytes, n) == 0); // persisted
  }
}

#define SET8(Name, addr, v)                                                    \
  do {                                                                         \
    begin("set" #Name);                                                        \
    uint8_t b[1] = {v};                                                        \
    expectWrite(set##Name(H, 1, v, &se), addr, b, 1, false);                   \
  } while (0)
#define SAVE8(Name, addr, v)                                                   \
  do {                                                                         \
    begin("setSave" #Name);                                                    \
    uint8_t b[1] = {v};                                                        \
    expectWrite(setSave##Name(H, 1, v, &se), addr, b, 1, true);                \
  } while (0)
// the setter refuses the value and puts nothing on the bus
#define REJECT(call)                                                             do {                                                                             int sent_ = packetsSent;                                                       CHECK((call) == RANGE_ERROR);                                                  CHECK(packetsSent == sent_);                                                 } while (0)

// max from the vendor memory table: max is written, max + 1 is refused
#define BOTH8(Name, addr, max)                                                   SET8(Name, addr, max);                                                         SAVE8(Name, addr, max);                                                        if ((max) < 255) {                                                               begin("set" #Name " out of range");                                            REJECT(set##Name(H, 1, (uint8_t)((max) + 1), &se));                            REJECT(setSave##Name(H, 1, (uint8_t)((max) + 1), &se));                      }

#define SET16(Name, addr, v)                                                   \
  do {                                                                         \
    begin("set" #Name);                                                        \
    uint8_t b[2] = {(uint8_t)((v)&0xFF), (uint8_t)((v) >> 8)};                 \
    expectWrite(set##Name(H, 1, v, &se), addr, b, 2, false);                   \
  } while (0)
#define SAVE16(Name, addr, v)                                                  \
  do {                                                                         \
    begin("setSave" #Name);                                                    \
    uint8_t b[2] = {(uint8_t)((v)&0xFF), (uint8_t)((v) >> 8)};                 \
    expectWrite(setSave##Name(H, 1, v, &se), addr, b, 2, true);                \
  } while (0)
#define BOTH16(Name, addr, max)                                                  SET16(Name, addr, max);                                                        SAVE16(Name, addr, max);                                                       begin("set" #Name " out of range");                                            REJECT(set##Name(H, 1, (uint16_t)((max) + 1), &se));                           REJECT(setSave##Name(H, 1, (uint16_t)((max) + 1), &se))

// signed: the expected wire bytes are written out by hand (low, high)
#define SETS16(Name, addr, v, lo, hi)                                          \
  do {                                                                         \
    begin("set" #Name " " #v);                                                 \
    uint8_t b[2] = {lo, hi};                                                   \
    expectWrite(set##Name(H, 1, v, &se), addr, b, 2, false);                   \
  } while (0)
#define SAVES16(Name, addr, v, lo, hi)                                         \
  do {                                                                         \
    begin("setSave" #Name " " #v);                                             \
    uint8_t b[2] = {lo, hi};                                                   \
    expectWrite(setSave##Name(H, 1, v, &se), addr, b, 2, true);                \
  } while (0)

#define GET8(Name, addr)                                                       \
  do {                                                                         \
    begin("get" #Name);                                                        \
    servos[0].mem[addr] = 0xA5;                                                \
    uint8_t v = 0;                                                             \
    CHECK(get##Name(H, 1, &v, &se) == 0);                                      \
    CHECK(v == 0xA5);                                                          \
    CHECK(writes.empty() && violations == violationsBefore);                   \
  } while (0)
#define GET16(Name, addr)                                                      \
  do {                                                                         \
    begin("get" #Name);                                                        \
    servos[0].mem[addr] = 0x34;                                                \
    servos[0].mem[addr + 1] = 0x12;                                            \
    uint16_t v = 0;                                                            \
    CHECK(get##Name(H, 1, &v, &se) == 0);                                      \
    CHECK(v == 0x1234);                                                        \
    CHECK(writes.empty() && violations == violationsBefore);                   \
  } while (0)
#define GETS16(Name, addr, lo, hi, want)                                       \
  do {                                                                         \
    begin("get" #Name " " #want);                                              \
    servos[0].mem[addr] = lo;                                                  \
    servos[0].mem[addr + 1] = hi;                                              \
    int16_t v = 0;                                                             \
    CHECK(get##Name(H, 1, &v, &se) == 0);                                      \
    CHECK(v == (want));                                                        \
    CHECK(writes.empty() && violations == violationsBefore);                   \
  } while (0)

static void testRegisterTable() {
  ctx = "register.h vs Feetech table";
  CHECK(DRIVER_TABLE_N == TABLE_N);
  for (int i = 0; i < DRIVER_TABLE_N && i < TABLE_N; i++) {
    bool same = !strcmp(DRIVER_TABLE[i].name, TABLE[i].name) &&
                DRIVER_TABLE[i].addr == TABLE[i].addr &&
                DRIVER_TABLE[i].size == TABLE[i].size;
    if (!same)
      printf("  register.h %s @%d size %d, expected %s @%d size %d\n",
             DRIVER_TABLE[i].name, DRIVER_TABLE[i].addr, DRIVER_TABLE[i].size,
             TABLE[i].name, TABLE[i].addr, TABLE[i].size);
    CHECK(same);
  }
}

static void testGetters() {
  GET8(FirmwareMajor, 0);
  GET8(FirmwareMinor, 1);
  GET8(ReturnDelay, 7);
  GET8(ResponseLevel, 8);
  GET8(MaxTemperature, 13);
  GET8(MaxVoltage, 14);
  GET8(MinVoltage, 15);
  GET8(Phase, 18);
  GET8(UnloadingCondition, 19);
  GET8(LedAlarmCondition, 20);
  GET8(PGain, 21);
  GET8(DGain, 22);
  GET8(IGain, 23);
  GET8(CwDeadZone, 26);
  GET8(CcwDeadZone, 27);
  GET8(AngularResolution, 30);
  GET8(OperatingMode, 33);
  GET8(ProtectiveTorque, 34);
  GET8(ProtectionTime, 35);
  GET8(OverloadTorque, 36);
  GET8(SpeedPGain, 37);
  GET8(OvercurrentTime, 38);
  GET8(SpeedIGain, 39);
  GET8(TorqueEnable, 40);
  GET8(Acceleration, 41);
  GET8(Lock, 55);
  GET8(PresentVoltage, 62);
  GET8(PresentTemperature, 63);
  GET8(Status, 65);
  GET8(Moving, 66);

  // ID and baud can't be poked to 0xA5 without moving the servo off the bus
  begin("getID");
  uint8_t v = 0;
  CHECK(getID(H, 2, &v, &se) == 0 && v == 2);
  begin("getBaudRate");
  v = 9;
  CHECK(getBaudRate(H, 1, &v, &se) == 0 && v == 0);

  GET16(ModelNumber, 3);
  GET16(MinAngleLimit, 9);
  GET16(MaxAngleLimit, 11);
  GET16(MaxTorque, 16);
  GET16(MinStartupForce, 24);
  GET16(ProtectionCurrent, 28);
  GET16(TorqueLimit, 48);
  GET16(PresentCurrent, 69);

  // sign-magnitude: sign bit 15 unless noted
  GETS16(PositionOffset, 31, 0x64, 0x00, 100);
  GETS16(PositionOffset, 31, 0x64, 0x08, -100); // bit 11
  GETS16(GoalPosition, 42, 0xE8, 0x03, 1000);
  GETS16(GoalPosition, 42, 0xE8, 0x83, -1000);
  GETS16(GoalTime, 44, 0xF4, 0x01, 500);
  GETS16(GoalTime, 44, 0xF4, 0x05, -500); // bit 10
  GETS16(GoalSpeed, 46, 0xD0, 0x07, 2000);
  GETS16(GoalSpeed, 46, 0xD0, 0x87, -2000);
  GETS16(PresentPosition, 56, 0xFF, 0x0F, 4095);
  GETS16(PresentPosition, 56, 0x64, 0x80, -100);
  GETS16(PresentSpeed, 58, 0xD0, 0x07, 2000);
  GETS16(PresentSpeed, 58, 0xD0, 0x87, -2000);
  GETS16(PresentLoad, 60, 0xF4, 0x01, 500);
  GETS16(PresentLoad, 60, 0xF4, 0x05, -500); // bit 10
}

static void testSetters() {
  // ranges from the Waveshare/Feetech ST3215 memory table V3.7
  BOTH8(ReturnDelay, 7, 254);
  BOTH8(ResponseLevel, 8, 1);
  BOTH8(MaxTemperature, 13, 100);
  BOTH8(MaxVoltage, 14, 254);
  BOTH8(MinVoltage, 15, 254);
  BOTH8(Phase, 18, 254);
  BOTH8(UnloadingCondition, 19, 254);
  BOTH8(LedAlarmCondition, 20, 254);
  BOTH8(PGain, 21, 254);
  BOTH8(DGain, 22, 254);
  BOTH8(IGain, 23, 254);
  BOTH8(CwDeadZone, 26, 32);
  BOTH8(CcwDeadZone, 27, 32);
  BOTH8(AngularResolution, 30, 3);
  BOTH8(OperatingMode, 33, 3);
  BOTH8(ProtectiveTorque, 34, 100);
  BOTH8(ProtectionTime, 35, 254);
  BOTH8(OverloadTorque, 36, 100);
  BOTH8(SpeedPGain, 37, 100);
  BOTH8(OvercurrentTime, 38, 254);
  BOTH8(SpeedIGain, 39, 254);
  SET8(TorqueEnable, 40, 1);
  SET8(Acceleration, 41, 254);
  SET8(Lock, 55, 0);

  BOTH16(MinAngleLimit, 9, 32767);
  BOTH16(MaxAngleLimit, 11, 32767);
  BOTH16(MaxTorque, 16, 1000);
  BOTH16(MinStartupForce, 24, 1000);
  BOTH16(ProtectionCurrent, 28, 511);
  SET16(TorqueLimit, 48, 1000);

  SETS16(PositionOffset, 31, 2047, 0xFF, 0x07);
  SETS16(PositionOffset, 31, -2047, 0xFF, 0x0F); // bit 11
  SAVES16(PositionOffset, 31, -100, 0x64, 0x08);
  SETS16(GoalPosition, 42, 0, 0x00, 0x00);
  SETS16(GoalPosition, 42, 2048, 0x00, 0x08);
  SETS16(GoalPosition, 42, 4095, 0xFF, 0x0F);
  SETS16(GoalPosition, 42, -1000, 0xE8, 0x83);
  SETS16(GoalTime, 44, 1000, 0xE8, 0x03);
  SETS16(GoalTime, 44, -1000, 0xE8, 0x07); // bit 10
  SETS16(GoalSpeed, 46, 3400, 0x48, 0x0D);
  SETS16(GoalSpeed, 46, -3400, 0x48, 0x8D);

  begin("out of range, nothing sent");
  REJECT(setAngularResolution(H, 1, 0, &se)); // minimum is 1
  REJECT(setTorqueEnable(H, 1, 129, &se));
  REJECT(setAcceleration(H, 1, 255, &se));
  REJECT(setLock(H, 1, 2, &se));
  REJECT(setTorqueLimit(H, 1, 1001, &se));
  REJECT(setPositionOffset(H, 1, 2048, &se)); // would land on sign bit 11
  REJECT(setPositionOffset(H, 1, -2048, &se));
  REJECT(setSavePositionOffset(H, 1, 2048, &se));
  REJECT(setGoalPosition(H, 1, 30720, &se));
  REJECT(setGoalPosition(H, 1, -30720, &se));
  REJECT(setGoalTime(H, 1, 1001, &se));
  REJECT(setGoalSpeed(H, 1, -3401, &se));
  CHECK(writes.empty());
}

static void testErrors() {
  begin("no servo at that ID");
  uint8_t v = 0x77;
  CHECK(getPGain(H, 9, &v, &se) == PACKET_TIMEOUT);
  CHECK(v == 0x77); // output untouched on failure
  CHECK(setPGain(H, 9, 1, &se) == PACKET_TIMEOUT);
  CHECK(ping(H, 9, &se) == PACKET_TIMEOUT);
  CHECK(writes.empty());

  begin("servo reports overload");
  servos[0].err = 0x20;
  CHECK(getPGain(H, 1, &v, &se) == SERVO_ERROR && se == 0x20);
  se = 0;
  CHECK(ping(H, 1, &se) == SERVO_ERROR && se == 0x20);

  // a faulted servo still hands over its data, one getter of each width
  servos[0].mem[21] = 0x42;
  v = 0;
  CHECK(getPGain(H, 1, &v, &se) == SERVO_ERROR && v == 0x42);
  servos[0].mem[48] = 0x34;
  servos[0].mem[49] = 0x12;
  uint16_t v16 = 0;
  CHECK(getTorqueLimit(H, 1, &v16, &se) == SERVO_ERROR && v16 == 0x1234);
  servos[0].mem[56] = 0x64;
  servos[0].mem[57] = 0x80;
  int16_t s16 = 0;
  CHECK(getPresentPosition(H, 1, &s16, &se) == SERVO_ERROR && s16 == -100 && se == 0x20);
  // and a healthy read afterwards clears the stale error byte
  CHECK(getPresentPosition(H, 2, &s16, &se) == 0 && se == 0);

  begin("corrupted reply");
  corruptRepliesFromId = 1;
  v = 0x77;
  CHECK(getPGain(H, 1, &v, &se) == CHECKSUM_ERROR && v == 0x77);
  CHECK(setPGain(H, 1, 5, &se) == CHECKSUM_ERROR);

  begin("broadcast setter reaches every servo, waits for no reply");
  CHECK(setTorqueEnable(H, 0xFE, 1, &se) == 0);
  CHECK(servos[0].mem[40] == 1 && servos[1].mem[40] == 1 && servos[2].mem[40] == 1);
}

static void testInstructions() {
  begin("ping");
  CHECK(ping(H, 1, &se) == 0 && ping(H, 3, &se) == 0);
  CHECK(writes.empty() && violations == violationsBefore);

  begin("regWrite then action");
  uint8_t goal[2] = {0xE8, 0x03};
  CHECK(regWrite(H, 2, 42, 2, goal, &se) == 0);
  CHECK(memcmp(servos[1].mem, before[1].mem, MEM) == 0); // held until action
  CHECK(action(H) == 0);
  CHECK(servos[1].mem[42] == 0xE8 && servos[1].mem[43] == 0x03);
  CHECK(memcmp(servos[0].mem, before[0].mem, MEM) == 0);
  CHECK(violations == violationsBefore);

  begin("syncWrite");
  uint8_t cmd[] = {1, 0xE8, 0x03, 2, 0xD0, 0x07, 3, 0xB8, 0x0B};
  CHECK(syncWrite(H, 42, 2, 3, cmd) == 0);
  CHECK(servos[0].mem[42] == 0xE8 && servos[0].mem[43] == 0x03);
  CHECK(servos[1].mem[42] == 0xD0 && servos[1].mem[43] == 0x07);
  CHECK(servos[2].mem[42] == 0xB8 && servos[2].mem[43] == 0x0B);
  CHECK(writes.size() == 3 && violations == violationsBefore);

  uint8_t ids[3] = {1, 2, 3}, out[6], res[3], errs[3];

  begin("syncRead, all present");
  for (int i = 0; i < 3; i++) {
    servos[i].mem[56] = (uint8_t)(0x10 + i);
    servos[i].mem[57] = (uint8_t)(0x01 + i);
  }
  CHECK(syncRead(H, 56, 2, 3, ids, out, res, errs) == 0);
  const uint8_t want[6] = {0x10, 0x01, 0x11, 0x02, 0x12, 0x03};
  CHECK(memcmp(out, want, 6) == 0);
  CHECK(res[0] == 0 && res[1] == 0 && res[2] == 0);

  begin("syncRead, servo 2 unplugged, servo 3 overheated");
  servos[1].present = false;
  servos[2].err = 0x04;
  servos[0].mem[56] = 0x11;
  servos[2].mem[56] = 0x33;
  memset(out, 0xEE, sizeof out);
  CHECK(syncRead(H, 56, 2, 3, ids, out, res, errs) == PACKET_TIMEOUT);
  CHECK(res[0] == 0 && out[0] == 0x11);
  CHECK(res[1] == PACKET_TIMEOUT && out[2] == 0xEE && out[3] == 0xEE);
  CHECK(res[2] == SERVO_ERROR && errs[2] == 0x04 && out[4] == 0x33);

  begin("syncRead, servo 1's reply corrupted");
  corruptRepliesFromId = 1;
  servos[1].mem[56] = 0x22;
  CHECK(syncRead(H, 56, 2, 3, ids, out, res, errs) == PACKET_TIMEOUT);
  CHECK(res[0] == PACKET_TIMEOUT && res[1] == 0 && res[2] == 0 && out[2] == 0x22);
}

static void baudCase(const char *name, bool save, bool replyAtNew, bool ignore) {
  begin(name);
  baudReplyAtNewRate = replyAtNew;
  ignoreBaudWrite = ignore;
  Servo &s = servos[0];
  int r = save ? setSaveBaudRate(H, 1, 4, &se) : setBaudRate(H, 1, 4, &se);
  CHECK(violations == violationsBefore);
  CHECK(s.mem[A_LOCK] == 1); // never left unlocked
  if (!ignore) {
    CHECK(r == 0);
    CHECK(hostBaud == 4 && s.mem[A_BAUD] == 4);
    CHECK(s.saved[A_BAUD] == (save ? 4 : 0));
  } else {
    CHECK(r != 0);
    CHECK(hostBaud == 0 && s.mem[A_BAUD] == 0); // host went back to the old rate
    CHECK(ping(H, 1, &se) == 0);                // and can still talk to it
  }
}

static void testBaud() {
  baudCase("setBaudRate, reply at old rate", false, false, false);
  baudCase("setBaudRate, reply at new rate", false, true, false);
  baudCase("setSaveBaudRate, reply at old rate", true, false, false);
  baudCase("setSaveBaudRate, reply at new rate", true, true, false);
  baudCase("setBaudRate, servo refuses", false, false, true);
  baudCase("setSaveBaudRate, servo refuses", true, false, true);

  begin("baud: bad arguments send nothing");
  int sent = packetsSent;
  CHECK(setBaudRate(H, 1, 8, &se) == INVALID_ARGUMENT);
  CHECK(setSaveBaudRate(H, 0xFE, 1, &se) == INVALID_ARGUMENT);
  CHECK(packetsSent == sent && hostBaud == 0);

  begin("baud: servo not there");
  CHECK(setSaveBaudRate(H, 9, 4, &se) == PACKET_TIMEOUT);
  CHECK(hostBaud == 0 && writes.empty());
}

static void testSetID() {
  begin("setID, servo replies with its old ID");
  Servo &s = servos[0];
  CHECK(setID(H, 1, 7, &se) == 0);
  CHECK(s.mem[A_ID] == 7 && s.saved[A_ID] == 7 && s.mem[A_LOCK] == 1);
  CHECK(servos[1].mem[A_ID] == 2 && servos[2].mem[A_ID] == 3);
  CHECK(violations == violationsBefore);

  begin("setID, servo replies with its new ID");
  replyNewIdAfterIdWrite = true;
  CHECK(setID(H, 1, 7, &se) == 0);
  CHECK(s.mem[A_ID] == 7 && s.saved[A_ID] == 7 && s.mem[A_LOCK] == 1);

  begin("setID to the ID it already has");
  CHECK(setID(H, 1, 1, &se) == 0);
  CHECK(s.mem[A_ID] == 1 && s.mem[A_LOCK] == 1);

  begin("setID, broadcast refused, nothing sent");
  int sent = packetsSent;
  CHECK(setID(H, 1, 0xFE, &se) == INVALID_ARGUMENT);
  CHECK(setID(H, 0xFE, 7, &se) == INVALID_ARGUMENT);
  CHECK(packetsSent == sent);

  begin("setID to an ID another servo has");
  CHECK(setID(H, 1, 2, &se) == ID_IN_USE);
  CHECK(writes.empty()); // refused before touching anything
  CHECK(s.mem[A_ID] == 1 && servos[1].mem[A_ID] == 2);

  begin("setID to an ID held by a faulted servo");
  servos[1].err = 0x20;
  CHECK(setID(H, 1, 2, &se) == ID_IN_USE && writes.empty());

  begin("setID, servo refuses the write");
  ignoreIdWrite = true;
  CHECK(setID(H, 1, 7, &se) == PACKET_TIMEOUT);
  CHECK(s.mem[A_ID] == 1 && s.mem[A_LOCK] == 1); // still there, relocked

  begin("setID, no servo at the old ID");
  CHECK(setID(H, 9, 7, &se) == PACKET_TIMEOUT && writes.empty());
}

int main() {
  testRegisterTable();
  testGetters();
  testSetters();
  testErrors();
  testInstructions();
  testBaud();
  testSetID();

  ctx = "whole run";
  CHECK(violations == 0);
  printf("\n%d checks, %d failed, %d protocol violations, %d packets sent\n",
         checks, failed, violations, packetsSent);
  return failed ? 1 : 0;
}
