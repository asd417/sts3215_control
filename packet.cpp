#include "packet.h"
#include "driver_errors.h"

#include <stdint.h>

  // ┌───────┬────────────┬──────────────────────────────────┬────────────┬─────────────────────────────────┐
  // │ INSTR │    Name    │              Params              │    LEN     │              Reply              │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 01    │ PING       │ none                             │ 2          │ status, no data                 │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 02    │ READ       │ ADDR, N                          │ 4          │ status with N bytes             │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 03    │ WRITE      │ ADDR, d1..dn                     │ n + 3      │ status, no data                 │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 04    │ REG_WRITE  │ ADDR, d1..dn                     │ n + 3      │ status, no data                 │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 05    │ ACTION     │ none                             │ 2          │ none (use FE)                   │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 82    │ SYNC_READ  │ ADDR, N, ID1..IDk                │ k + 4      │ one status per ID, N bytes each │
  // ├───────┼────────────┼──────────────────────────────────┼────────────┼─────────────────────────────────┤
  // │ 83    │ SYNC_WRITE │ ADDR, N, ID1, d×N, ..., IDk, d×N │ k(N+1) + 4 │ none                            │
  // └───────┴────────────┴──────────────────────────────────┴────────────┴─────────────────────────────────┘



// INSTR
constexpr uint8_t INST_PING = 1;
constexpr uint8_t INST_READ = 2; // any range; the reply contains LEN bytes
constexpr uint8_t INST_WRITE = 3;
constexpr uint8_t INST_REG_WRITE = 4;
constexpr uint8_t INST_ACTION = 5; // usually sent to ID FE
constexpr uint8_t INST_SYNC_WRITE =
    131; // 0x83  same address and length for every servo
constexpr uint8_t INST_SYNC_READ =
    130; // 0x82  each servo gets exactly LEN bytes
// ERROR
constexpr uint8_t ERRBIT_VOLTAGE = 1;
constexpr uint8_t ERRBIT_ANGLE = 2;
constexpr uint8_t ERRBIT_OVERHEAT = 4;
constexpr uint8_t ERRBIT_OVERELE = 8;
constexpr uint8_t ERRBIT_OVERLOAD = 32;


inline uint8_t checksum(uint8_t ID, uint8_t INST, uint8_t parmLength,
                        uint8_t *parms) {
  uint8_t _checksum = ID + parmLength + 2 + INST;
  for (int i = 0; i < parmLength; i++) {
    _checksum += parms[i];
  }
  return ~_checksum & 0xFF;
}

inline void copyParams(uint8_t *out, uint8_t start, uint8_t parmCount,
                       uint8_t *parms) {
  for (int i = 0; i < parmCount; i++) out[i + start] = parms[i];
}


void createActionPacket(uint8_t *out, uint8_t ID) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = ID;
  out[3] = 2;
  out[4] = INST_ACTION;
  out[5] = checksum(ID, INST_ACTION, 0, nullptr);
  }


void createPingPacket(uint8_t *out, uint8_t ID) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = ID;
  out[3] = 2;
  out[4] = INST_PING;
  out[5] = checksum(ID, INST_PING, 0, nullptr);
  }


void createReadPacket(uint8_t *out, uint8_t ID, uint8_t targetRead,
                      uint8_t targetReadSize) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = ID; // dont use broadcast(0xFE) for read
  out[3] = 4;  // INSTR, ADDR, N, CS
  out[4] = INST_READ;
  out[5] = targetRead;
  out[6] = targetReadSize;
  out[7] = checksum(ID, INST_READ, 2, &out[5]);
}

void createWritePacket(uint8_t *out, const uint8_t ID, const uint8_t dataSize,
                       uint8_t *data, uint8_t targetAddr) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = ID;
  out[3] = dataSize + 3;
  out[4] = INST_WRITE;
  out[5] = targetAddr;
  copyParams(out, 6, dataSize, data);
  out[6 + dataSize] = checksum(ID, INST_WRITE, dataSize + 1, &out[5]);
}

// wait for ACTION
void createRegWritePacket(uint8_t *out, const uint8_t ID, const uint8_t dataSize,
                       uint8_t *data, uint8_t targetAddr) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = ID;
  out[3] = dataSize + 3;
  out[4] = INST_REG_WRITE;
  out[5] = targetAddr;
  copyParams(out, 6, dataSize, data);
  out[6 + dataSize] = checksum(ID, INST_REG_WRITE, dataSize + 1, &out[5]);
}

void createSyncReadPacket(uint8_t *out, const uint8_t IDCount, uint8_t *IDs,
                          const uint8_t targetRead, const uint8_t targetReadSize) {
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = 0xFE;
  out[3] = IDCount + 4; // INSTR, ADDR, N, IDs, CS
  out[4] = INST_SYNC_READ;
  out[5] = targetRead;
  out[6] = targetReadSize;
  copyParams(out, 7, IDCount, IDs);
  out[7 + IDCount] = checksum(0xFE, INST_SYNC_READ, IDCount + 2, &out[5]);
}

void createSyncWritePacket(uint8_t* out, const uint8_t IDCount, uint8_t targetWrite, uint8_t targetWriteSize, uint8_t* commandBuffer)
{
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = 0xFE;
  out[3] = IDCount * (targetWriteSize + 1) + 4;
  out[4] = INST_SYNC_WRITE;
  out[5] = targetWrite;
  out[6] = targetWriteSize;
  copyParams(out, 7, IDCount * (targetWriteSize+1), commandBuffer);
  out[7 + IDCount * (targetWriteSize + 1)] = checksum(0xFE, INST_SYNC_WRITE, IDCount * (targetWriteSize + 1) + 2, &out[5]);
}

//   Status packet (servo → you)
//   FF FF | ID | LEN | ERROR | data... | CHECKSUM
//   LEN = data + 2. ERROR sits where INSTR was. The checksum is ~(ID + LEN + ERROR + data) & 0xFF.

//   ┌───────────────┬────────────────┬────────────────────┐
//   │     Byte      │ Counted in LEN │ Summed in checksum │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ FF FF         │ no             │ no                 │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ ID            │ no             │ yes                │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ LEN           │ no             │ yes                │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ INSTR / ERROR │ yes            │ yes                │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ params / data │ yes            │ yes                │
//   ├───────────────┼────────────────┼────────────────────┤
//   │ CHECKSUM      │ yes            │ no                 │
//   └───────────────┴────────────────┴────────────────────┘
int parseSinglePacket(uint8_t* in, const uint8_t bufferSize, const uint8_t expectedParmSize, uint8_t* outID, uint8_t* outINST, uint8_t** outParm)
{
  if(bufferSize < expectedParmSize + 2 + 4) return HOST_RX_BUFFER_SMALL; //host gave too little
  uint8_t h1 = in[0];
  uint8_t h2 = in[1];
  if (h1 != 0xFF || h2 != 0xFF) return INCORRECT_RX_HEADER;
  uint8_t ID = in[2];
  uint8_t LEN = in[3];
  if (LEN < 2 || expectedParmSize + 2 != LEN) return LEN_MISMATCH; //because LEN can never be less than 2
  uint8_t INST = in[4]; // could be ERROR
  uint8_t *parms = &in[5];
  uint8_t CS = in[expectedParmSize + 2 + 3];
  if (CS != checksum(ID, INST, LEN - 2, parms))
  {
    return CHECKSUM_ERROR;
  };
  *outID = ID;
  *outINST = INST;
  if(outParm) *outParm = parms; // can pass a nullptr to outparm to just not read from it.
  return 0;
}