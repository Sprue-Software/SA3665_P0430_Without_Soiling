#include "DummyDeviceProtocol.h"

typedef enum
{
	DUMMY_IOReadPin = 0x1701u,
	DUMMY_IOSetPin = 0x1702u,
	DUMMY_IOResetPin = 0x1703u,
	DUMMY_IOReadPins = 0x1704u,
	DUMMY_IOSetPins = 0x1705u,
	DUMMY_IOResetPins = 0x1706u
} P0200_CALCommand_t;

const nextGenCommsCommand_t DummyDeviceProtocolCommandTable[DUMMY_DEVICE_PROTOCOL_COMMAND_COUNT] = {
	{.command = DUMMY_IOReadPin, .rxSizeMin = 64u, .rxSizeMax = 128u, .handler = nextGenComms_CommandNotImplemented},
	{.command = DUMMY_IOSetPin, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = nextGenComms_CommandNotImplemented},
	{.command = DUMMY_IOResetPin, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = nextGenComms_CommandNotImplemented},
	{.command = DUMMY_IOReadPins, .rxSizeMin = 8u, .rxSizeMax = 8u, .handler = nextGenComms_CommandNotImplemented},
	{.command = DUMMY_IOSetPins, .rxSizeMin = 8u, .rxSizeMax = 8u, .handler = nextGenComms_CommandNotImplemented},
	{.command = DUMMY_IOResetPins, .rxSizeMin = 8u, .rxSizeMax = 8u, .handler = nextGenComms_CommandNotImplemented}
};
