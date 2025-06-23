#ifndef NG_COMMS_P0200_CAL_H
#define NG_COMMS_P0200_CAL_H

#include "stdint.h"
#include "nextgen_protocol.h"

#define P0200_CAL_COMMAND_COUNT 20u
extern const nextGenCommsCommand_t P0200_CALCommandTable[P0200_CAL_COMMAND_COUNT];
#define P0200_CAL_COMMAND_TABLE_SIZE ((size_t )(sizeof(P0200_CALCommandTable) / sizeof(P0200_CALCommandTable[0u])))

#endif //NG_COMMS_P0200_CAL_H
