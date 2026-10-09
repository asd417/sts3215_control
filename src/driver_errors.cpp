#include "driver_errors.h"

#ifndef __AVR__
const char* errorToString(int error) {
  switch (error) {
  case 0:
    return "OK";
  case INCORRECT_WRITE_PACKET_SIZE:
    return "Incorrect write packet size";
  case INCORRECT_READ_PACKET_SIZE:
    return "Incorrect read packet size";
  case INCORRECT_RX_HEADER:
    return "Reply header is not 0xFF 0xFF";
  case CHECKSUM_ERROR:
    return "Reply checksum mismatch";
  case HOST_RX_BUFFER_SMALL:
    return "Host receive buffer too small";
  case UNEXPECTED_ID:
    return "Reply came from an unexpected ID";
  case SERVO_ERROR:
    return "Servo reported an error (see servo error byte)";
  case PACKET_TIMEOUT:
    return "No valid reply from servo";
  case PACKET_SEND_FAILED:
    return "Packet send failed";
  case CONNECTION_LOST:
    return "Connection lost after baud rate change";
  case INVALID_ARGUMENT:
    return "Invalid argument";
  case ID_IN_USE:
    return "ID already in use on the bus";
  case LEN_MISMATCH:
    return "Reply length does not match request";
  case RANGE_ERROR:
    return "Value out of range for register";
  case PF_READ_FAIL:
    return "Port read failed";
  case PF_SEND_FAIL:
    return "Port write failed";
  case PF_TIMEOUT_FAIL:
    return "Port read timed out";
  case PF_TIMEOUT_SET_FAIL:
    return "Setting port timeouts failed";
  case PF_BAUD_SET_FAIL:
    return "Setting port baud rate failed";
  case PF_PORT_OPEN_FAIL:
    return "Opening port failed";
  case PF_COMMSTATE_FAIL:
    return "Configuring port failed";
  default:
    return "Unknown error";
  }
}
#endif
