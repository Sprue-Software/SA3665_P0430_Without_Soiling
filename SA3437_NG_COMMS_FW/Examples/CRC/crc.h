#ifndef NG_COMMS_CRC_H
#define NG_COMMS_CRC_H

#include <stdint.h>

extern uint16_t nextGenComms_CalculateCRC(const uint8_t buffer[], const uint8_t length);

#endif //NG_COMMS_CRC_H
