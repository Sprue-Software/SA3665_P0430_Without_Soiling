/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef DATA_LOGGING_H_
#define DATA_LOGGING_H_

/********************************************************************************************************
*********************************************************************************************************
*                                            	INCLUDES
*********************************************************************************************************
********************************************************************************************************/
#include "debug.h"
#include "stdint.h"
#include "stdbool.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

/*Number of supported Smoke, CO, Heat and Fault Events*/
#define MAX_SUPPORTED_EVENTS 10u

/*Length defines*/
#define DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX 20u
#define DEF_LEN_EVENT_LOGBOOK_DATA	8u
#define DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX 169u
#define LASER_CALIBRATION_DATA_LEN	64u
#define RADIO_CALIBRATION_DATA_LEN	16u

/* Events Types */
#define EVENT_TYPE_LOCAL	0u
#define EVENT_TYPE_REMOTE	1u

/* Mounting Events Types */
#define EVENT_TYPE_MOUNTED		0u
#define EVENT_TYPE_DEMOUNTED	1u

/* Logbook events */
#define DEF_LBE_RESET								     				(0u)
#define DEF_LBE_STANDBY_MODE_START						 	(1u)				
#define DEF_LBE_STANDBY_MODE_END						 		(2u)    // Deprecated, use start event only
#define DEF_LBE_COMM_MODE_START	     					 	(3u)				
#define DEF_LBE_COMM_MODE_END	     					 		(4u)    // Deprecated, use start event only
#define DEF_LBE_OPERATING_MODE_START					 	(5u)				
#define DEF_LBE_OPERATING_MODE_END	  					(6u)    // Deprecated, use start event only
#define DEF_LBE_TRANSPORT_MODE_START					 	(7u)				
#define DEF_LBE_TRANSPORT_MODE_END	    				(8u)    // Deprecated, use start event only
#define DEF_LBE_DEMOUNTED_START							 		(9u)
#define DEF_LBE_DEMOUNTED_TOO_LONG_START	    	(10u)
#define DEF_LBE_DEMOUNTED_END										(11u)
#define DEF_LBE_DEMOUNTED_DET_HW_ERR_START			(12u)
#define DEF_LBE_DEMOUNTED_DET_HW_ERR_END				(13u)   // Deprecated, use start event only
#define DEF_LBE_USER_BIST							    			(14u)
#define DEF_LBE_SMOKE_DET_START									(15u)
#define DEF_LBE_SMOKE_DET_END										(16u)
#define DEF_LBE_SMOKE_CHAM_CONT_START						(17u)
#define DEF_LBE_SMOKE_CHAM_CONT_END							(18u)
#define DEF_LBE_SMOKE_CHAM_HW_ERR_START					(19u)
#define DEF_LBE_SMOKE_CHAM_HW_ERR_END						(20u)   // Deprecated, use start event only
#define DEF_LBE_SUPER_SMOKE_START					 			(21u)
#define DEF_LBE_SUPER_SMOKE_END					 				(22u)
#define DEF_LBE_SOILED_START					 					(23u)
#define DEF_LBE_SOILED_END					 		    		(24u)
#define DEF_LBE_SOILING_DET_HW_ERR_START     		(25u)
#define DEF_LBE_SOILING_DET_HW_ERR_END			 		(26u)   // Deprecated, use start event only
#define DEF_LBE_CO_DET_START					 					(27u)
#define DEF_LBE_CO_DET_END					 		    		(28u)
#define DEF_LBE_CO_DET_HW_ERR_START     				(29u)
#define DEF_LBE_CO_SENSOR_EOL_START     				(30u)
#define DEF_LBE_SUPER_CO_START					 				(31u)
#define DEF_LBE_SUPER_CO_END					 					(32u)
#define DEF_LBE_HEAT_DET_START					 				(33u)
#define DEF_LBE_HEAT_DET_END					 					(34u)
#define DEF_LBE_HEAT_DET_HW_ERR_START						(35u)
#define DEF_LBE_HEAT_DET_HW_ERR_END					 		(36u)   // Deprecated, use start event only
#define DEF_LBE_SUPER_HEAT_START					 			(37u)
#define DEF_LBE_SUPER_HEAT_END						 			(38u)
#define DEF_LBE_ALARM_MUTED_START					 			(39u)
#define DEF_LBE_ALARM_MUTED_END						 			(40u)
#define DEF_LBE_REMOTE_ALARM_TEST								(41u)
#define DEF_LBE_REMOTE_ALARM_SILENCE						(42u)
#define DEF_LBE_REMOTE_ALARM_RECEIVED						(43u)			// Not Implemented
#define DEF_LBE_TEMP_OOR_START					 				(44u)
#define DEF_LBE_TEMP_OOR_END						 				(45u)
#define DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_START		 		(46u)
#define DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_END	 	(47u)   // Deprecated, use start event only
#define DEF_LBE_HUMID_OOR_START				 					(48u)
#define DEF_LBE_HUMID_OOR_END					 					(49u)
#define DEF_LBE_HUMID_SENSOR_HW_ERR_START	 			(50u)
#define DEF_LBE_HUMID_SENSOR_HW_ERR_END			 		(51u)   // Deprecated, use start event only
#define DEF_LBE_OBSTACLE_DET_START				 			(52u)
#define DEF_LBE_OBSTACLE_DET_END					 			(53u)
#define DEF_LBE_OBSTACLE_DET_HW_ERR_START	 			(56u)
#define DEF_LBE_OBSTACLE_DET_HW_ERR_END			 		(57u)   // Deprecated, use start event only
#define DEF_LBE_COVERAGE_DET_START				 			(58u)
#define DEF_LBE_COVERAGE_DET_END					 			(59u)
#define DEF_LBE_COVERAGE_HW_ERR_START				 		(62u)			// Been deleted, but still in FTM
#define DEF_LBE_COVERAGE_HW_ERR_END					 		(63u)			// Been deleted, but still in FTM
#define DEF_LBE_BUZZER_CHECK_HW_ERR_START			 	(68u)
#define DEF_LBE_BUZZER_CHECK_HW_ERR_END				 	(69u)   // Been deleted, but still in FTM // Deprecated, use start event only
#define DEF_LBE_BATTERY_ERR_START					 			(70u)
#define DEF_LBE_BATTERY_SHUTDOWN_START					(71u)
#define DEF_LBE_FAULT_MUTED_START								(72u)
#define DEF_LBE_FAULT_MUTED_END	  							(73u)
#define DEF_LBE_AMB_LIGHT_7_DAYS_DARK						(74u)
#define DEF_LBE_EEPROM_CONFIG_CHANGE_SIT				(79u)			// Not Implemented
#define DEF_LBE_SMOKE_REMOTE_ALARM            	(80u)
#define DEF_LBE_HEAT_REMOTE_ALARM             	(81u)
#define DEF_LBE_CO_REMOTE_ALARM               	(82u)

/**
 * @brief Maximum number of logbook events types
 */
#define DEF_LBE_MAX               							(83u)

/* Device Coniguration */
#define DEVICE_CONFIG_SMOKE											(0x00u)
#define DEVICE_CONFIG_NO_SMOKE									(0x01u)

/**
 * @brief Start event parameter value
 * 
 * @see DataLogging_SetEventLogbookRecordStartEnd()
 */
#define DEF_LBE_START														true

/**
 * @brief End event parameter value
 * 
 * @see DataLogging_SetEventLogbookRecordStartEnd()
 */
#define DEF_LBE_END															false

/**
 * @brief Start logbook event macro
 * 
 * @see DataLogging_SetEventLogbookRecordStartEnd()
 */
#define DL_SET_EVENT_LOGBOOK_RECORD_START(E,M) 	DataLogging_SetEventLogbookRecordStartEnd((E),(M),DEF_LBE_START)

/**
 * @brief End logbook event macro
 * 
 * @see DataLogging_SetEventLogbookRecordStartEnd()
 */
#define DL_SET_EVENT_LOGBOOK_RECORD_END(E,M) 		DataLogging_SetEventLogbookRecordStartEnd((E),(M),DEF_LBE_END)

/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef enum
{
	RstPOR,
	RstPIN,
	RstEM4,
	RstWDOG0,
	RstWDOG1,
	RstLOCKUP,
	RstSYSREQ,
	RstDVDDBOD,
	RstDVDDLEBOD,
	RstDECBOD,
	RstAVDDBOD,
	RstIOVDDBOD,
} dl_reset_reason_t;

typedef enum
{
	FaultDegradedSmokeChamber,
	FaultObstacleDetected,
	FaultSoilDetected,
	FaultCoverageDetected,
	FaultTemperatureOutOfBound,
	FaultHumidityOutOfBound,
	FaultTestButton,
	FaultDemountedTooLong,
	FaultCalibrationDataCorrupt,
	FaultObstacleBISTOverdue,
	FaultObstacleDetectionOverdue,
	FaultBuzzerCheckOverdue,

} dl_minor_fault_t;

typedef struct
{
	uint16_t FwNumber;
	uint8_t FwRevMajor;
	uint8_t FwRevMinor;
	uint8_t FwRevBuild;
} __attribute__((packed)) dl_fw_rev_data_t;

typedef struct
{
	uint16_t TemperatureValMin;
	uint16_t TemperatureValMax;
	uint8_t TemperatureBistPeriod;
} __attribute__((packed)) dl_temperature_cfg_data_t;

typedef struct
{
	uint8_t HumidityValMin;
	uint8_t HumidityValMax;
	uint8_t HumidityBistPeriod;
} __attribute__((packed)) dl_humidity_cfg_data_t;

typedef struct
{
	uint16_t AmbientLightThreshold;
	uint8_t AmbientLightAcqPeriod;
	uint8_t AmbientLightBistPeriod;
	uint16_t AmbientLightValMin;
	uint16_t AmbientLightValMax;
} __attribute__((packed)) dl_ambient_light_cfg_data_t;

typedef struct
{
  /* 4-bytes */
  uint32_t 	SoilingDetectionThreshold 	: 8;      /* Mesh blocked percent 0 to 100% */
  uint32_t 	SoilingMillvolts0A			    : 12;     /* Millvolts 0% soiling A */
  uint32_t 	SoilingMillvolts0B          : 12;     /* Millvolts 0% soiling B */

  /* 4-bytes */
  uint32_t 	SoilingMillvoltsHighLimit   : 12;     /* Millvolts Fail High Limit */
  uint32_t 	SoilingSampleCount          : 4;      /* ADC sample count */
  uint32_t 	SoilingStrikeCount          : 4;      /* Strike count */
  uint32_t 	SoilingSensorSelect         : 2;      /* Sensor to use SOILING_SENSOR_SELECT */
  uint32_t 	SoilingSpare                : 10;     /* Spare bits */
}
__attribute__((packed)) dl_soiling_cfg_data_t;

typedef struct  
{
	uint16_t offset;
	uint16_t gain;
}
__attribute__((packed)) dl_abuf_cfg_data_t;

typedef struct
{
	uint16_t Batt_A;
	uint16_t Batt_B;
} __attribute__((packed)) dl_monthly_batt_level_t;

typedef struct
{
	uint16_t Batt_A;
	uint16_t Batt_B;
} __attribute__((packed)) dl_monthly_batt_impedance_t;

typedef struct
{
	uint32_t Timestamp;
	uint8_t EventType;
} __attribute__((packed)) dl_event_t;

typedef struct
{
	uint16_t BuzzerThreshold;
	uint8_t BuzzerBistPeriod;
} __attribute__((packed)) dl_buzzer_cfg_data_t;

typedef struct
{
	uint32_t Timestamp;
	uint32_t Code;
} __attribute__((packed)) dl_fault_event_t;

typedef struct
{
	uint16_t Count;
	uint32_t Timestamp;
	uint8_t OperatingMode;
	uint8_t ID;
} __attribute__((packed)) dl_demounting_logbook_record_t;

typedef struct
{
	uint16_t Count;
	uint32_t Timestamp;
	uint8_t OperatingMode;
	uint8_t ID;
	uint8_t Data[DEF_LEN_EVENT_LOGBOOK_DATA];
} __attribute__((packed)) dl_event_logbook_record_t;

/********************************************************************************************************
*********************************************************************************************************
*                                                APIs
*********************************************************************************************************
********************************************************************************************************/

bool data_logging_is_eeprom_ok( void );
bool DataLogging_CheckDataIntergrity(void);

/************************************* Production & Manufacturing APIs**********************************/
void DataLogging_SetFWBuild(void);
void DataLogging_SetDeviceID(uint32_t data);
uint32_t DataLogging_GetDeviceID(void);
void DataLogging_SetFw(const dl_fw_rev_data_t *pstr_fw_rev);
void DataLogging_GetFw(dl_fw_rev_data_t *pstr_fw_rev);
void DataLogging_SetDeviceConfig(uint8_t devCfg);
uint8_t DataLogging_GetDeviceConfig(void);
void DataLogging_SetDoM(uint32_t DateOfManufacture);
uint32_t DataLogging_GetDoM(void);

/************************************* CO APIs**********************************/
void data_logging_set_na_per_ppm( uint16_t na_per_ppm );
uint32_t data_logging_get_na_per_ppm( void );
void DataLogging_SetCOCal(uint16_t CoCal);
uint16_t DataLogging_GetCOCal(void);
void DataLogging_SetCOAcqPeriod(uint8_t COAcqPeriod);
uint8_t DataLogging_GetCOAcqPeriod(void);
void DataLogging_SetCOBistPeriod(uint8_t COBistPeriod);
uint8_t DataLogging_GetCOBistPeriod(void);
void DataLogging_SetCOOpenCircuitStrikeCount(uint8_t COOCStrikeCount);
uint8_t DataLogging_GetCOOpenCircuitStrikeCount(void);
void DataLogging_SetCOShortCircuitStrikeCount(uint8_t COSCStrikeCount);
uint8_t DataLogging_GetCOShortCircuitStrikeCount(void);
void DataLogging_SetCOFatigueStrikeCount(uint8_t COFatigeStrikeCount);
uint8_t DataLogging_GetCOFatigueStrikeCount(void);
void DataLogging_SetCOVarianceStrikeCount(uint16_t COVarianceStrikeCount);
uint16_t DataLogging_GetCOVarianceStrikeCount(void);
void DataLogging_SetCOFalseAlarmStrikeCount(uint8_t COFalseAlarmStrikeCount);
uint8_t DataLogging_GetCOFalseAlarmStrikeCount(void);
void DataLogging_SetCOHBThreshold(uint16_t COHBThresold);
uint16_t DataLogging_GetCOHBThreshold(void);
void DataLogging_SetCOVarianceThreshold(uint16_t COVarianceThreshold);
uint16_t DataLogging_GetCOVarianceThreshold(void);
void DataLogging_SetCOShortCircuitThreshold(uint16_t COShortCircuitThreshold);
uint16_t DataLogging_GetCOShortCircuitThreshold(void);
void DataLogging_SetCOOpenCircuitThreshold(uint16_t COOpenCircuitThreshold);
uint16_t DataLogging_GetCOOpenCircuitThreshold(void);
void DataLogging_SetSuperCOThreshold(uint16_t COSuperCOThreshold);
uint16_t DataLogging_GetSuperCOThreshold(void);

/************************************* Smoke APIs**********************************/
void DataLogging_SetSmokeCalRefThreshold(uint16_t SmokeCalRefThreshold);
uint16_t DataLogging_GetSmokeCalRefThreshold(void);
void DataLogging_SetSmokeCalThreshold(uint16_t SmokeCalThreshold);
uint16_t DataLogging_GetSmokeCalThreshold(void);
void DataLogging_SetSmokeAcqPeriod(uint8_t SmokeAcqPeriod);
uint8_t DataLogging_GetSmokeAcqPeriod(void);
void DataLogging_SetSmokeAcqPeriodInAlarm(uint8_t SmokeAcqPeriodInAlarm);
uint8_t DataLogging_GetSmokeAcqPeriodInAlarm(void);
void DataLogging_SetSmokeBISTPeriod(uint16_t SmokeBistPeriod);
uint8_t DataLogging_GetSmokeBISTPeriod(void);
void DataLogging_SetAlarmSmokeThreshold(uint16_t AlarmSmokeThreshold);
uint16_t DataLogging_GetAlarmSmokeThreshold(void);
void DataLogging_SetSmokeAFEGain(uint16_t SmokeAFEGain);
uint16_t DataLogging_GetSmokeAFEGain(void);
void DataLogging_SetSmokeAFEIntegration(uint8_t SmokeAFEIntegration);
uint8_t DataLogging_GetSmokeAFEIntegration(void);
void DataLogging_SetSmokeLEDCurrent(uint16_t SmokeLedCurrent);
uint16_t DataLogging_GetSmokeLEDCurrent(void);
void DataLogging_SetSmokeADCSettlingPeriod(uint8_t SmokeADCSettlingPeriod);
uint8_t DataLogging_GetSmokeADCSettlingPeriod(void);
void DataLogging_SetSmokeCorrectionHistorySize(uint8_t SmokeCorrectionAvgSetpoint);
uint8_t DataLogging_GetSmokeCorrectionHistorySize(void);
void DataLogging_SetSmokeCircuitFaultThreshold(uint16_t SmokeCircuitFaultThreshold);
uint16_t DataLogging_GetSmokeCircuitFaultThreshold(void);
void DataLogging_SetSmokeFastFlameThreshold(uint16_t SmokeFastFlameThreshold);
uint16_t DataLogging_GetSmokeFastFlameThreshold(void);
void DataLogging_SetMaxSmokeDustCorrect(uint8_t SmokeDustCorrectionMax);
uint8_t DataLogging_GetMaxSmokeDustCorrection(void);
void DataLogging_SetSmokeDustCorrectMin(uint8_t SmokeDustCorrectionMin);
uint8_t DataLogging_GetSmokeDustCorrectionMin(void);
void DataLogging_SetSmokeChamberFaultStrikeCount(uint8_t SmokeChamberFaultStrikeCount);
uint8_t DataLogging_GetSmokeChamberFaultStrikeCount(void);
void DataLogging_SetSmokeHwFaultStrikeCount(uint8_t SmokeHwFaultStrikeCount);
uint8_t DataLogging_GetSmokeHwFaultStrikeCount(void);

/************************************* Heat APIs**********************************/
void DataLogging_SetHeatAcqPeriod(uint8_t HeatAcqPeriod);
uint8_t DataLogging_GetHeatAcqPeriod(void);
void DataLogging_SetHeatBistPeriod(uint8_t HeatBistPeriod);
uint8_t DataLogging_GetHeatBistPeriod(void);
void DataLogging_SetHeatBatteryCompValues(uint32_t BatteryCompValues);
uint32_t DataLogging_GetHeatBatteryCompValues(void);
void DataLogging_SetSuperHeatThreshold(int16_t SuperHeatThreshold);
int16_t DataLogging_GetSuperHeatThreshold(void);
void DataLogging_SetMaxHeatOutOfBounds(uint16_t HeatOutOfBoundsMax);
uint16_t DataLogging_GetMaxHeatOutOfBounds(void);
void DataLogging_SetMinHeatOutOfBounds(uint16_t HeatOutOfBoundsMin);
uint16_t DataLogging_GetMinHeatOutOfBounds(void);
void DataLogging_SetHeatFaultStrikeCount(uint8_t HeatFaultStrikeCount);
uint8_t DataLogging_GetHeatFaultStrikeCount(void);
void DataLogging_SetCoeffThirtyDegRiseInThirtySec(int16_t ThirtyDegRiseInThirtySec);
int16_t DataLogging_GetCoeffThirtyDegRiseInThirtySec(void);
void DataLogging_SetCoeffTwentyDegRiseInThirtySec(int16_t TwentyDegRiseInThirtySec);
int16_t DataLogging_GetCoeffTwentyDegRiseInThirtySec(void);
void DataLogging_SetCoeffTenDegRiseInSixtySec(int16_t TenDegRiseInSixtySec);
int16_t DataLogging_GetCoeffTenDegRiseInSixtySec(void);
void DataLogging_SetCoeffFiveDegRiseInSixtySec(int16_t FiveDegRiseInSixtySec);
int16_t DataLogging_GetCoeffFiveDegRiseInSixtySec(void);
void DataLogging_SetCoeffThreeDegRiseInNinetySec(int16_t ThreeDegRiseInNinetySec);
int16_t DataLogging_GetCoeffThreeDegRiseInNinetySec(void);
void DataLogging_SetCoeffOneDegRiseInNinetySec(int16_t OneDegRiseInNinetySec);
int16_t DataLogging_GetCoeffOneDegRiseInNinetySec(void);
/************************************* Battery APIs**********************************/
void DataLogging_SetLowBatteryThreshold(uint16_t LowBatteryThreshold);
uint16_t DataLogging_GetLowBatteryThreshold(void);
void DataLogging_SetBatteryDropThreshold(uint16_t BatteryDropThreshold);
uint16_t DataLogging_GetBatteryDropThreshold(void);
void DataLogging_SetBatteryBistPeriod(uint8_t BatteryBistPeriod);
uint8_t DataLogging_GetBatteryBistPeriod(void);
void DataLogging_SetBatteryBistStrikeCount(uint8_t BatteryBistStrikeCount);
uint8_t DataLogging_GetBatteryBistStrikeCount(void);
void DataLogging_SetDeadBatteryThreshold(uint16_t DeadBatteryThreshold);
uint16_t DataLogging_GetDeadBatteryThreshold(void);
void DataLogging_SetDeadBatteryBistPeriod(uint8_t DeadBatteryBistPeriod);
uint8_t DataLogging_GetDeadBatteryBistPeriod(void);

/************************************* Laser Calibration APIs**********************************/
void DataLogging_SetLaserCalData(const uint8_t LaserCalibrationData[LASER_CALIBRATION_DATA_LEN]);
void DataLogging_GetLaserCalData(uint8_t LaserCalibrationData[LASER_CALIBRATION_DATA_LEN]);

/************************************* Temperature & Humidity APIs**********************************/
void DataLogging_SetTempConfig(const dl_temperature_cfg_data_t *pstr_temp_config);
void DataLogging_GetTempConfig(dl_temperature_cfg_data_t *pstr_temp_config);
void DataLogging_SetHumidityConfig(const dl_humidity_cfg_data_t *pstr_humidity_config);
void DataLogging_GetHumidityConfig(dl_humidity_cfg_data_t *pstr_humidity_config);

/************************************* Ambient Light APIs**********************************/
void DataLogging_SetAmbientLightConfig(const dl_ambient_light_cfg_data_t *pstr_ambient_light_cfg);
void DataLogging_GetAmbientLightConfig(dl_ambient_light_cfg_data_t *pstr_ambient_light_cfg);

/************************************* ABUF APIs**********************************/
void DataLogging_GetAbufConfig( dl_abuf_cfg_data_t * ptr );

/************************************* Assistance Light APIs**********************************/
void DataLogging_SetAssiatnceLightPeriod(uint16_t AssistanceLightPeriod);
uint16_t DataLogging_GetAssistanceLightPeriod(void);

/************************************* Oscillator Tunning APIs**********************************/
void DataLogging_SetOscillatorTune(uint16_t OscillatorTuning);
uint16_t DataLogging_GetOscillatorTune(void);

/************************************* Buzzer APIs**********************************/
void DataLogging_SetBuzzerConfig(const dl_buzzer_cfg_data_t *pstr_buzzer_config);
void DataLogging_GetBuzzerConfig(dl_buzzer_cfg_data_t *pstr_buzzer_config);

/************************************* CRC APIs**********************************/
void DataLogging_SetCRC(void);
uint16_t DataLogging_GetCRC(void);

/************************************* One Time Data APIs**********************************/
void DataLogging_SetFirstActivation(uint32_t FirstActivation);
uint32_t DataLogging_GetFirstActivation(void);
void DataLogging_SetFirstBatteryLevel(uint16_t FirstBatteryLevel);
uint16_t DataLogging_GetFirstBatteryLevel(void);
void DataLogging_SetFirstBatteryImpedance(uint16_t FirstBatteryImpedance);
uint16_t DataLogging_GetFirstBatteryImpedance(void);
void DataLogging_SetFeatureConfigurationFlags(uint32_t FeatureConfigFlags);
uint32_t DataLogging_GetFeatureconfigurationFlags(void);

/************************************* Indexes APIs **********************************/
void DataLogging_ResetAllIndexes(void);
uint16_t DataLogging_GetMainLogbookIndex(void);
uint8_t DataLogging_GetDemountingIndex(void);
uint8_t DataLogging_GetBatteryLevelIndex(void);
uint8_t DataLogging_GetBatteryImpedanceIndex(void);
uint8_t DataLogging_GetSmokeEventsIndex(void);
uint8_t DataLogging_GetCOEventsIndex(void);
uint8_t DataLogging_GetHeatEventsIndex(void);
uint8_t DataLogging_GetFaultsEventsIndex(void);

void DataLogging_SetOperatingState(uint8_t OperatingState);
uint8_t DataLogging_GetOperatingState(void);
void DataLogging_SetLatestTimestamp(uint32_t LatestTimeStamp);
uint32_t DataLogging_GetLatestTimestamp(void);
void DataLogging_SetLatestUserBistTimestamp(void);
uint32_t DataLogging_GetLatestUserBistTimestamp(void);

void DataLogging_SetMonthlyBatteryLevel(const dl_monthly_batt_level_t *pstr_level);
void DataLogging_GetMonthlyBatteryLevel(uint8_t BattLevelIdx, dl_monthly_batt_level_t *pstr_level);
void DataLogging_SetMonthlyBatteryImpedance(const dl_monthly_batt_impedance_t *pstr_impedance);
void DataLogging_GetMonthlyBatteryImpedance(uint8_t BattImpedanceIdx, dl_monthly_batt_impedance_t *pstr_impedance);

void DataLogging_SetBatteryID(uint8_t BattID);
uint8_t DataLogging_GetBatteryID(void);
void DataLogging_SetBatteryFRTime(uint32_t BattFaultReportingTime);
uint32_t DataLogging_GetBatteryFRTime(void);
void DataLogging_SetFaultyBatteryVoltageLevel(uint16_t FaultBattVoltageLevel);
uint16_t DataLogging_GetFaultyBatteryVoltageLevel(void);
void DataLogging_SetFaultyBatteryImpedanceLevel(uint16_t FaultBattImpedanceLevel);
uint16_t DataLogging_GetFaultyBatteryImpedanceLevel(void);

/************************************* Smoke Events APIs **********************************/
void DataLogging_SetSmokeEvent(uint8_t event_type);
void DataLogging_GetSmokeEvent(uint8_t index, dl_event_t *pstr_smoke_event);
void DataLogging_SetSmokeThermoptekState(uint16_t SmokeThermoptekState);
uint16_t DataLogging_GetSmokeThermoptekState(void);
uint16_t DataLogging_GetSmokeLocalEventCount(void);
uint16_t DataLogging_GetSmokeRemoteEventCount(void);

/************************************* CO Events APIs **********************************/
void DataLogging_SetCOEvent(uint8_t event_type);
void DataLogging_GetCOEvent(uint8_t index, dl_event_t *pstr_co_event);
void DataLogging_SetCOVariance32HrCoverage(uint16_t COVariance32HourAvg);
uint16_t DataLogging_GetCOVariance32HrCoverage(void);
uint16_t DataLogging_GetCOLocalEventCount(void);
uint16_t DataLogging_GetCORemoteEventCount(void);
uint8_t DataLogging_GetCOCalibTemperature( void );
void DataLogging_SetCOCalibTemperature( const uint8_t temperature );
uint8_t DataLogging_GetCOCalibHumidity( void );
void DataLogging_SetCOCalibHumidity( const uint8_t humidity );

/************************************* Heat Events APIs **********************************/
void DataLogging_SetHeatEvent(uint8_t event_type);
void DataLogging_GetHeatEvent(uint8_t index, dl_event_t *pstr_heat_event);
uint16_t DataLogging_GetHeatLocalEventCount(void);
uint16_t DataLogging_GetHeatRemoteEventCount(void);

/************************************* Fault APIs **********************************/
void DataLogging_SetFault(uint32_t Code);
void DataLogging_GetFault(uint8_t index, dl_fault_event_t *pstr_fault_event);
uint16_t DataLogging_GetFaultEventCount(void);
void DataLogging_SetFaultDuration(uint32_t FaultDuration);
uint32_t DataLogging_GetFaultDuration(void);

void DataLogging_LogUserTest(void);
uint16_t DataLogging_GetUserTestCount(void);
void DataLogging_LogUserExtTest(void);
uint16_t DataLogging_GetUserExtTestCount(void);
void DataLogging_LogMountingEvent(void);
uint16_t DataLogging_GetMountingEventCount(void);
void DataLogging_SetDemountedStateDuration(uint32_t DemountedStateDuration);
uint32_t DataLogging_GetDemountedStateDuration(void);
void DataLogging_SetDustCompensationLevel(uint16_t DustCompensationLevel);
uint16_t DataLogging_GetDustCompensationLevel(void);

void DataLogging_SetRadioDurationCounter(const uint8_t RadioDurationCounterAndData[RADIO_CALIBRATION_DATA_LEN]);
void DataLogging_GetRadioDurationCounter(uint8_t RadioDurationCounterAndData[RADIO_CALIBRATION_DATA_LEN]);
void DataLogging_SetMCU1toMCU2CommsDuration(uint32_t MCU1ToMCU2CommsDuration);
uint32_t DataLogging_GetMCU1toMCU2CommsDuration(void);
void DataLogging_SetMCU2toMCU1CommsDuration(uint32_t MCU2ToMCU1CommsDuration);
uint32_t DataLogging_GetMCU2toMCU1CommsDuration(void);
void DataLogging_SetMCU1toMCU2InstigatedCommsCount(uint32_t MCU1ToMCU2InstigatedCommsCount);
uint32_t DataLogging_GetMCU1toMCU2InstigatedCommsCount(void);
void DataLogging_SetMCU2toMCU1InstigatedCommsCount(uint32_t MCU2ToMCU1InstigatedCommsCound);
uint32_t DataLogging_GetMCU2toMCU1InstigatedCommsCount(void);
void DataLogging_SetAssistLightTotalDuration(uint32_t AssistanceLightTotalDuration);
uint32_t DataLogging_GetAssistLightTotalDuration(void);
void DataLogging_SetAssistLightActivCount(uint16_t AssistanceLightActivationCount);
uint16_t DataLogging_GetAssistLightActivCount(void);
void DataLogging_SetAirRecActivCount(uint32_t AiringRecommendActivationCount);
uint32_t DataLogging_GetAirRecActivCount(void);
void DataLogging_SetAirRecTotalDuration(uint32_t AiringRecommendTotalDuration);
uint32_t DataLogging_GetAirRecTotalDuration(void);
void DataLogging_SetAirConfigChangeCount(uint16_t AiringConfigChangeCount);
uint16_t DataLogging_GetAirConfigChangeCount(void);

void DataLogging_SetDemountingLogbookRecord( const uint8_t event_type );
void DataLogging_GetDemountingLogbookRecord(uint16_t index, dl_demounting_logbook_record_t *pstr_demounting_logbook);

void data_logging_logbook_init( void );

void DataLogging_SetEventLogbookRecord(uint8_t event_id, uint8_t data[DEF_LEN_EVENT_LOGBOOK_DATA]);
void DataLogging_SetEventLogbookRecordStartEnd(uint8_t event_id, uint8_t data[DEF_LEN_EVENT_LOGBOOK_DATA], const bool start);
void DataLogging_GetEventLogbookRecord(uint16_t index, dl_event_logbook_record_t *pstr_event_logbook);

void DataLogging_SetResetReason(dl_reset_reason_t RstReason);
uint16_t DataLogging_GetResetReasonCount(dl_reset_reason_t RstReason);

void DataLogging_SetMinorFault(dl_minor_fault_t MinorFault, const bool status);
uint16_t DataLogging_GetMinorFaultCount(dl_minor_fault_t MinorFault);

uint16_t DataLogging_GetLastDemountingLogbookIndex( void );
uint16_t DataLogging_GetLastMainLogbookIndex( void );

bool DataLoggingErase(uint16_t startAddress);
void DataLogging_SetAbufConfig( dl_abuf_cfg_data_t * pstr_abuf_cfg );
void Reset_EEPROM_Production(void);

uint32_t dl_get_production_complete( void );
bool dl_set_production_complete( const uint32_t magic_number );

uint32_t dl_get_production_timer(void);
bool dl_set_production_timer( const uint32_t magic_number );

uint32_t  dl_get_prod_commence_time(void);
bool  dl_set_prod_commence_time(const uint32_t magic_number);

void Set_Reset_Cause_MCU(bool status);
bool Get_Reset_Cause_MCU(void);

void DataLogging_SetLogMsgToMCU(bool status);
bool DataLogging_GetLogMsgToMCU(void);


/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* DATA_LOGGING_H_ */
