#pragma once
#include <stdint.h>



#define PACKET_SIZE_OVERHEAD 6
#define PACKET_SIZE_PING 6
#define PACKET_SIZE_ACTION 6
#define PACKET_SIZE_READ 8
#define PACKET_SIZE_REG_WRITE_BASE 7 // 7 + data size in bytes
#define PACKET_SIZE_WRITE_BASE 7 // 7 + data size in bytes

#define PACKET_SIZE_SYNC_READ_BASE 8 // 8 + 1 per ID
// command buffer is ID1 d1..dx ID2 d1..dx etc
#define PACKET_SIZE_SYNC_WRITE_BASE 8 // 8 + (2 or 3 per ID) depending on data

void createActionPacket(uint8_t *out, uint8_t ID);
void createPingPacket(uint8_t *out, uint8_t ID);
void createReadPacket(uint8_t *out, uint8_t ID, uint8_t targetRead, uint8_t targetReadSize);
void createWritePacket(uint8_t *out, const uint8_t ID, const uint8_t dataSize, uint8_t *data, uint8_t targetAddr);
void createRegWritePacket(uint8_t *out, const uint8_t ID, const uint8_t dataSize, uint8_t *data, uint8_t targetAddr);
void createSyncReadPacket(uint8_t *out, const uint8_t IDCount, uint8_t *IDs, const uint8_t targetRead, const uint8_t targetReadSize);
void createSyncWritePacket(uint8_t *out, const uint8_t IDCount, uint8_t targetWrite, uint8_t targetWriteSize, uint8_t *commandBuffer);
int parseSinglePacket(uint8_t* in, const uint8_t bufferSize, const uint8_t expectedParmSize, uint8_t* outID, uint8_t* outINST, uint8_t** outParm);
