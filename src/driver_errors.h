#pragma once
#define INCORRECT_WRITE_PACKET_SIZE 1
#define INCORRECT_READ_PACKET_SIZE 2
#define INCORRECT_RX_HEADER 3
#define CHECKSUM_ERROR 4
#define HOST_RX_BUFFER_SMALL 5
#define UNEXPECTED_ID 6
#define SERVO_ERROR 7
#define PACKET_TIMEOUT 8
#define PACKET_SEND_FAILED 9
#define CONNECTION_LOST 10
#define INVALID_ARGUMENT 11
#define ID_IN_USE 12
#define LEN_MISMATCH 13
#define RANGE_ERROR 14

//generic
#define PF_READ_FAIL 15
#define PF_SEND_FAIL 16
#define PF_TIMEOUT_FAIL 17
#define PF_TIMEOUT_SET_FAIL 18
#define PF_BAUD_SET_FAIL 19

//windows
#define PF_PORT_OPEN_FAIL 20
#define PF_COMMSTATE_FAIL 21

#ifndef __AVR__ // not enough ram. just use error codes directly
const char* errorToString(int error);
#endif