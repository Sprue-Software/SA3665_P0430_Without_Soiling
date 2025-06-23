/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef FAULT_HANDLER_H_
#define FAULT_HANDLER_H_

/********************************************************************************************************
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
********************************************************************************************************/

#include "debug.h"
#include "stdint.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

#define DEF_NO_FAULT 							          (0x00000000u)
#define DEF_SMOKE_CHAMBER_HW_FAULT 				  (0x00000001u)			/* Major Faults */
#define DEF_OBSTACLE_DET_HW_FAUT 				    (0x00000002u)
#define DEF_SOILING_DET_HW_FAULT 				    (0x00000004u)
#define DEF_DEMOUNTING_DET_HW_FAULT				  (0x00000008u)
#define DEF_CO_SENSOR_HW_FAULT	 				    (0x00000010u)
#define DEF_TEMP_SENSOR_HW_FAULT 				    (0x00000020u)
#define DEF_HUMIDITY_SENSOR_HW_FAULT			  (0x00000040u)
#define DEF_HEAT_SENSOR_HW_FAULT 				    (0x00000080u)
#define DEF_BUZER_HW_FAULT						      (0x00000100u)
#define DEF_BATTERY_FAULT 						      (0x00000200u)
#define DEF_CO_EOL_FAULT 						        (0x00000400u)
#define DEF_MCU1_RAM_FAULT	 					      (0x00000800u)
#define DEF_RADIO_FAULT		 					        (0x00001000u)    /* this fault is tuned in minor fault as per DCR0095 */
#define DEF_MCU2_RAM_FAULT	 					      (0x00002000u)
#define DEF_MCU2_SPI_COMMS_TIMEOUT			    (0x00004000u)
#define DEF_DEG_SMOKE_CHAMBER_FAULT				  (0x00010000u)			/* Minor Faults */
#define DEF_OBSTACLE_DET_FAULT 					    (0x00020000u)
#define DEF_SOILING_DET_FAULT 					    (0x00040000u)
#define DEF_COVERAGE_DET_FAULT 					    (0x00080000u)
#define DEF_TEMP_SENSOR_OOB_FAULT				    (0x00100000u)
#define DEF_HUMIDITY_SENSOR_OOB_FAULT 	    (0x00200000u)
#define DEF_TEST_BUTTON_FAULT 					    (0x00400000u)
#define DEF_DEM_TOO_LONG_FAULT 					    (0X00800000u)
#define DEF_EEPROM_CAL_DATA_CORRUPT_FAULT		(0x01000000u)

#define DEF_MAJOR_FAULT 						( DEF_SMOKE_CHAMBER_HW_FAULT		\
												| DEF_OBSTACLE_DET_HW_FAUT			\
												| DEF_SOILING_DET_HW_FAULT			\
												| DEF_DEMOUNTING_DET_HW_FAULT		\
												| DEF_CO_SENSOR_HW_FAULT			\
												| DEF_TEMP_SENSOR_HW_FAULT			\
												| DEF_HUMIDITY_SENSOR_HW_FAULT		\
												| DEF_HEAT_SENSOR_HW_FAULT			\
												| DEF_BUZER_HW_FAULT				\
												| DEF_BATTERY_FAULT					\
												| DEF_CO_EOL_FAULT					\
												| DEF_MCU1_RAM_FAULT				\
												| DEF_MCU2_RAM_FAULT				\
												| DEF_MCU2_SPI_COMMS_TIMEOUT)

#define DEF_MINOR_FAULT							( DEF_DEG_SMOKE_CHAMBER_FAULT		\
												| DEF_OBSTACLE_DET_FAULT			\
												| DEF_SOILING_DET_FAULT				\
												| DEF_COVERAGE_DET_FAULT			\
												| DEF_TEMP_SENSOR_OOB_FAULT			\
												| DEF_HUMIDITY_SENSOR_OOB_FAULT		\
												| DEF_TEST_BUTTON_FAULT				\
												| DEF_DEM_TOO_LONG_FAULT			\
												| DEF_RADIO_FAULT         \
												| DEF_EEPROM_CAL_DATA_CORRUPT_FAULT)

/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef enum {
	SmokeChamberDiodeHwFault,		 /* Major Faults */
	ObstacleDetectionHwFault,
	SoilingDetectionHwFault,
	DemountingDetectionHwFault,
	COSenserHwFault,
	TemperatureSensorHwFault,
	HumiditySensorHwFault,
	HeatSensorHwFault,
	BuzzerHwFault,
	BatteryFault,
	COEndOfLifeFault,
	MCU1RAMFault,
	RadioFault,                   /* this fault is tuned in minor fault as per DCR0095 */
	MCU2RAMFault,
	MCU2SPICommsTimeoutFault,
	EmptyBitSpaceForFutureUse,
	DegradedSmokeChamberFault,		/* Minor Faults */
	ObstacleDetectedFault,
	SoilingDetectedFault,
	CoverageDetectedFault,
	TempSensorOutOfBoundsFault,
	HumiditySensorOutOfBoundsFault,
	TestButtonFault,
	DemountedTooLongFault,
	EEPROMCalDataCorruptionFault
} FAULTHANDLER_FAULT;

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

void FaultHandler_FaultSet(FAULTHANDLER_FAULT fault);
void FaultHandler_FaultClear(FAULTHANDLER_FAULT fault);
void FaultHandler_FaultClearAll(void);
uint32_t FaultHandler_GetFaultFlags(void);
uint32_t FaultHandler_FaultCodeGet(FAULTHANDLER_FAULT fault);
void FaultHandler_Activate_Pattern(void);

/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* FAULT_HANDLER_H_ */
