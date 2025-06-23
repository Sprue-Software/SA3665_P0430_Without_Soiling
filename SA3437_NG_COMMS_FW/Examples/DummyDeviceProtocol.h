#ifndef NG_COMMS_DUMMY_DEVICE_PROTOCOL_H
#define NG_COMMS_DUMMY_DEVICE_PROTOCOL_H

#include "stdint.h"
#include "nextgen_protocol.h"

#define DUMMY_DEVICE_PROTOCOL_COMMAND_COUNT 6u
extern const nextGenCommsCommand_t DummyDeviceProtocolCommandTable[DUMMY_DEVICE_PROTOCOL_COMMAND_COUNT];
#define DUMMY_DEVICE_PROTOCOL_COMMAND_TABLE_SIZE ((size_t )(sizeof(DummyDeviceProtocolCommandTable) / sizeof(DummyDeviceProtocolCommandTable[0u])))

#endif //NG_COMMS_DUMMY_DEVICE_PROTOCOL_H
