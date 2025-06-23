/********************************************************************************************************
*
* 											FAULT HANDLER
*
* Filename			: fault_handler.c
* Version			: V1.00
* Programmers(s)	: AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "fault_handler.h"
#include "led_buzzer.h"
#include "data_logging.h"
#include "comms_handler.h"
#include "battery_measurement.h"

/********************************************************************************************************
*********************************************************************************************************
*                                              VARIABLES
*********************************************************************************************************
********************************************************************************************************/

static uint32_t FaultHandler_FaultFlags = DEF_NO_FAULT;

/********************************************************************************************************
*********************************************************************************************************
*                                             PROTOTYPES
*********************************************************************************************************
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

/****************************************************************************************************//**
*                                       FaultHandler_FaultSet()
*
* @brief   Sets fault in the local memory, log fault in the EEPROM, indicate fault to the user interface
*
* @param	fault	fault code to log
********************************************************************************************************/
void FaultHandler_FaultSet(FAULTHANDLER_FAULT fault) {
	uint32_t fault_code;												/* fault code to use 						*/
	uint32_t fault_flags;												/* local copy of fault flags				*/


	fault_code = FaultHandler_FaultCodeGet(fault);
	DEBUG_FAULT_HANDLER("\n Fault Set", true, fault_code);

	fault_flags = FaultHandler_FaultFlags;							 	/* make a local copy of fault flags   		*/
	fault_flags |= fault_code;											/* set the bit in the fault flags variable 	*/
	if (fault_flags != FaultHandler_FaultFlags) {
		FaultHandler_FaultFlags = fault_flags;							/* update fault flags 						*/
		DEBUG_FAULT_HANDLER("\n Fault Flag Set", true, (uint32_t)FaultHandler_FaultFlags);

		DataLogging_SetFault(fault_code);								/* log fault in EEPROM 						*/
	}
	else {
		/* do nothing - fault is already logged */
	}

	if(((fault_code & DEF_BATTERY_FAULT) != 0u) && (get_low_battery_status() == true))
	{
      /* Already related pattern is posted */
	}
	else
	{
	    if ((fault_code & DEF_MAJOR_FAULT) != 0u)
	    {
	        LEDBuzz_Post(PatternMajorFault); /* indicate major fault						*/
	    }
	    else
	    {
	        LEDBuzz_Post(PatternMinorFault); /* indicate minor fault						*/
	    }
	}
}

/****************************************************************************************************//**
*                                       FaultHandler_FaultClear()
*
* @brief   Clear minor fault
*
* @param	fault	fault code to log
********************************************************************************************************/
void FaultHandler_FaultClear(FAULTHANDLER_FAULT fault) {
	uint32_t fault_code;


	fault_code = FaultHandler_FaultCodeGet(fault);
	DEBUG_FAULT_HANDLER("\n Fault Clear", true, fault_code);

	if ((fault_code & DEF_MINOR_FAULT) != 0u) {
		FaultHandler_FaultFlags &= ~fault_code;								/* clear the fault flag 					*/
		DEBUG_FAULT_HANDLER("\n Fault Flag Cleared", true, (uint32_t)FaultHandler_FaultFlags);
	}
	else {
		/* do nothing, major fault cannot be cleared */
	}

	if ((FaultHandler_FaultFlags & DEF_MINOR_FAULT) == 0u) /* If all minor faults are cleared, then stop the minor fault indication*/
	{
		LEDBuzz_Post(PatternStopMinorFault);
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                      FaultHandler_GetFaultFlags()
*
* @brief	Clear minor fault
*
* @return	fault flags
********************************************************************************************************/
uint32_t FaultHandler_GetFaultFlags(void) {
	return FaultHandler_FaultFlags;
}

/****************************************************************************************************//**
*                                      FaultHandler_FaultCodeGet()
*
* @brief	Get the fault code
*
* @return	fault code
********************************************************************************************************/
uint32_t FaultHandler_FaultCodeGet(FAULTHANDLER_FAULT fault) {
	uint32_t fault_code;

	switch (fault) {
		case SmokeChamberDiodeHwFault:
			fault_code = DEF_SMOKE_CHAMBER_HW_FAULT;
			break;

		case ObstacleDetectionHwFault:
			fault_code = DEF_OBSTACLE_DET_HW_FAUT;
			break;

		case DemountingDetectionHwFault:
			fault_code = DEF_DEMOUNTING_DET_HW_FAULT;
			break;

		case COSenserHwFault:
			fault_code = DEF_CO_SENSOR_HW_FAULT;
			break;

		case TemperatureSensorHwFault:
			fault_code = DEF_TEMP_SENSOR_HW_FAULT;
			break;

		case HumiditySensorHwFault:
			fault_code = DEF_HUMIDITY_SENSOR_HW_FAULT;
			break;

		case HeatSensorHwFault:
			fault_code = DEF_HEAT_SENSOR_HW_FAULT;
			break;

		case BuzzerHwFault:
			fault_code = DEF_BUZER_HW_FAULT;
			break;

		case BatteryFault:
			fault_code = DEF_BATTERY_FAULT;
			break;

		case COEndOfLifeFault:
			fault_code = DEF_CO_EOL_FAULT;
			break;

		case MCU1RAMFault:
			fault_code = DEF_MCU1_RAM_FAULT;
			break;

		case RadioFault:
			fault_code = DEF_RADIO_FAULT;
			break;

		case MCU2RAMFault:
			fault_code = DEF_MCU2_RAM_FAULT;
			break;

		case DegradedSmokeChamberFault:
			fault_code = DEF_DEG_SMOKE_CHAMBER_FAULT;
			break;

		case ObstacleDetectedFault:
			fault_code = DEF_OBSTACLE_DET_FAULT;
			break;

		case CoverageDetectedFault:
			fault_code = DEF_COVERAGE_DET_FAULT;
			break;
		case TempSensorOutOfBoundsFault:
			fault_code = DEF_TEMP_SENSOR_OOB_FAULT;
			break;

		case HumiditySensorOutOfBoundsFault:
			fault_code = DEF_HUMIDITY_SENSOR_OOB_FAULT;
			break;

		case TestButtonFault:
			fault_code = DEF_TEST_BUTTON_FAULT;
			break;

		case DemountedTooLongFault:
			fault_code = DEF_DEM_TOO_LONG_FAULT;
			break;

		case EEPROMCalDataCorruptionFault:
			fault_code = DEF_EEPROM_CAL_DATA_CORRUPT_FAULT;
			break;

		case MCU2SPICommsTimeoutFault:
			fault_code = DEF_MCU2_SPI_COMMS_TIMEOUT;
			break;
			
		default:
			fault_code = DEF_NO_FAULT;
			break;
		}

	return fault_code;
}

void FaultHandler_FaultClearAll(void)
{
	FaultHandler_FaultFlags = DEF_NO_FAULT;
	LEDBuzz_Post(PatternLowBattStop);
	LEDBuzz_Post(PatternStopMajorFault);
	LEDBuzz_Post(PatternStopMinorFault);
}

void FaultHandler_Activate_Pattern(void)
{
  if(((FaultHandler_FaultFlags & DEF_BATTERY_FAULT ) != 0u) && (get_low_battery_status() == true))
  {
      LEDBuzz_Post(PatternLowBatt);
  }
  else if (((FaultHandler_FaultFlags & DEF_MAJOR_FAULT) != 0u))
	{
		LEDBuzz_Post(PatternMajorFault); /* indicate major fault						*/
	}
	else 
	{
		/*Do nothing*/
	}
	
	if (((FaultHandler_FaultFlags & DEF_MINOR_FAULT) != 0u))
	{
		LEDBuzz_Post(PatternMinorFault); /* indicate minor fault						*/
	}
	else 
	{
		/*Do nothing*/
	}
}
