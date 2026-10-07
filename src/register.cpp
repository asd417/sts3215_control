#include "register.h"

// 0 for an index the servo doesn't have
uint32_t baudFromIndex(uint8_t baudIndex)
{
  static const uint32_t rates[] = {1000000, 500000, 250000, 128000,
                                   115200,  76800,  57600,  38400};
  return baudIndex < sizeof rates / sizeof rates[0] ? rates[baudIndex] : 0;
}
