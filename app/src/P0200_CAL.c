#include "P0200_CAL.h"

typedef enum
{
	CAL_CORead = 0x0406u,
	CAL_COCFValueRead = 0x0407u,
	CAL_COCFValueWrite = 0x0408u,
	CAL_COCalibrationWrite = 0x0409u,
	CAL_COCalibrationRead = 0x040Au,
	CAL_SmokeThresholdRead = 0x0507u,
	CAL_SmokeThresholdWrite = 0x0508u,
	CAL_BatteryLowThresholdWrite = 0x0705u,
	CAL_BatteryLowThresholdRead = 0x0706u,
	CAL_HumidityRead = 0x0903u,
	CAL_TemperatureRead = 0x0A03u,
	CAL_TemperatureCompensationWrite = 0x0A04u,
	CAL_TemperatureCompensationRead = 0x0A05u,
	CAL_ReflectionThresholdRead = 0x0C06u,
	CAL_ReflectionThresholdWrite = 0x0C07u,
	CAL_EEPROMReadAll = 0x0F03u,
	CAL_EEPROMWriteAll = 0x0F05u,
	CAL_FLASHRead = 0x1501u,
	CAL_FLASHWrite = 0x1502u,
	CAL_ReadAllSensors = 0xE001u,
} P0200_CALCommand_t;

const nextGenCommsCommand_t P0200_CALCommandTable[P0200_CAL_COMMAND_COUNT] = {
	{.command = CAL_CORead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_COCFValueRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_COCFValueWrite, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_COCalibrationWrite, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_COCalibrationRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_SmokeThresholdRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_SmokeThresholdWrite, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_BatteryLowThresholdWrite, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_BatteryLowThresholdRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_HumidityRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_TemperatureRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_TemperatureCompensationWrite, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_TemperatureCompensationRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_ReflectionThresholdRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_ReflectionThresholdWrite, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_EEPROMReadAll, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_EEPROMWriteAll, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_FLASHRead, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_FLASHWrite, .rxSizeMin = 8u, .rxSizeMax = 8u, .handler = nextGenComms_CommandNotImplemented},
	{.command = CAL_ReadAllSensors, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = nextGenComms_CommandNotImplemented}
};
