/********************************************************************************************************
*
* 											 DATA LOGGING
*
* Filename			: data_logging.c
* Version			: V1.00
* Programmers(s)	: AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "data_logging.h"
#include "hal_i2c.h"
#include "eeprom_handler.h"
#include "common_utils.h"
#include "timeHandler.h"
#include "comms_handler.h"
#include "string.h"
#include "crc.h"
#include "fault_handler.h"
#include "system_events.h"
#include "diagnostics.h"
#include "app.h"

/********************************************************************************************************
*********************************************************************************************************
*                                             Constants & Definitions
*********************************************************************************************************
********************************************************************************************************/
#define WORKING_BUFF_LEN	256u		
#define MINOR_FAULT_COUNTER 12u

#define MAX_SUPPORTED_NUM_BATTERY_LEVELS 66u
#define MAX_SUPPORTED_NUM_BATTERY_IMPEDANCES 66u
/**********  Addresses of Various Sections **********/
#define DATALOGGING_CALIB_CONFIG_ADDR_OFFSET 	(uint16_t)(0x0000u)
#define DATALOGGING_STATIC_LOCATION_ADDR_OFFSET (uint16_t)(0x0100u)
#define DATALOGGING_DEMOUNTING_LOGBOOK_ADDR_OFFSET (uint16_t)(0x04D0u)
#define DATALOGGING_EVENT_LOGBOOK_ADDR_OFFSET (uint16_t)(0x0570u)

/*Calibration & Configuration Data offsets*/
#define DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, ProductMfgData)))
#define DATALOGGING_CO_CONFIG_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, CoConfigData)))
#define DATALOGGING_SMOKE_CONFIG_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, SmokeConfigData)))
#define DATALOGGING_HEAT_CONFIG_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, HeatConfigData)))
#define DATALOGGING_BATT_CONFIG_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, BattConfigData)))
#define DATALOGGING_BUZZ_CONFIG_DATA_OFFSET (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, BuzzerConfigData)))

/*Static Data offsets*/
#define DATALOGGING_ONE_TIME_DATA_OFFSET (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, OneTimeData)))
#define DATALOGGING_INDEXES_OFFSET (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MainIndexes)))

#define LOGBOOK_EVENT_ID_MAX (0xFFFFu)

/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef struct  
{
	uint32_t DeviceID;
	uint32_t DateOfManufacture;
	dl_fw_rev_data_t FwRev;
	uint8_t DeviceConfig;
	uint8_t rsrv[2];
} __attribute__((packed)) dl_product_mfg_data_t;

typedef struct  
{
	uint16_t  na_per_ppm;
	uint16_t  COCal;
  uint8_t   CO_calib_temperature;
	uint8_t   CO_calib_humidity;
	uint16_t  COShortCircuitThreshold;
	uint16_t  COOpenCircuitThreshold;
	uint16_t  COVarianceStrikeCount;
	uint16_t  COHBThresold;
	uint16_t  COVarianceThreshold;
	uint16_t  COSuperCOThreshold;
	uint8_t   COAcqPeriod;
	uint8_t   COBistPeriod;
	uint8_t   COOCStrikeCount;
	uint8_t   COSCStrikeCount;
	uint8_t   COFatigeStrikeCount;
	uint8_t   COFalseAlarmStrikeCount;
	uint8_t   rsrv[8];
} __attribute__((packed)) dl_co_cfg_data_t;

typedef struct  
{
	uint16_t SmokeCalRefThreshold;
	uint16_t SmokeCalThreshold;
	uint16_t AlarmSmokeThreshold;
	uint16_t SmokeAFEGain;
	uint16_t SmokeLedCurrent;
	uint16_t SmokeCircuitFaultThreshold;
	uint16_t unused;
	uint16_t SmokeFastFlameThreshold;
	uint8_t SmokeAFEIntegration;
	uint8_t SmokeADCSettlingPeriod;
	uint8_t SmokeCorrectionHistorySize;
	uint8_t SmokeDustCorrectionMax;
	uint8_t SmokeDustCorrectionMin;
	uint8_t SmokeChamberFaultStrikeCount;
	uint8_t SmokeHwFaultStrikeCount;
	uint16_t SmokeBistPeriod;
	uint8_t SmokeAcqPeriod;
	uint8_t rsrv[6];
} __attribute__((packed)) dl_smoke_cfg_data_t;

typedef struct  
{
	uint32_t BatteryCompValues;
	int16_t SuperHeatThreshold;
	uint16_t HeatOutOfBoundsMax;
	uint16_t HeatOutOfBoundsMin;
	uint8_t HeatAcqPeriod;
	uint8_t HeatBistPeriod;
	uint8_t HeatFaultStrikeCount;
	uint8_t rsrv1[3];
	int16_t ThirtyDegRiseInThirtySec;
	int16_t TwentyDegRiseInThirtySec;
	int16_t TenDegRiseInSixtySec;
	int16_t FiveDegRiseInSixtySec;
	int16_t ThreeDegRiseInNinetySec;
	int16_t OneDegRiseInNinetySec;
} __attribute__((packed)) dl_heat_cfg_data_t;

typedef struct  
{
	uint16_t LowBatteryThreshold;
	uint16_t BatteryDropThreshold;
	uint16_t DeadBatteryThreshold;
	uint8_t BatteryBistPeriod;
	uint8_t BatteryBistStrikeCount;
	uint8_t DeadBatteryBistPeriod;
	uint8_t rsrv[7];
} __attribute__((packed)) dl_batt_cfg_data_t;

typedef struct  
{
	dl_product_mfg_data_t ProductMfgData;
	dl_co_cfg_data_t CoConfigData;
	dl_smoke_cfg_data_t SmokeConfigData;
	dl_heat_cfg_data_t HeatConfigData;
	dl_abuf_cfg_data_t abuf_config_data;
	uint32_t production_complete;
	uint32_t production_timer;
	uint32_t prod_commence_time;
	uint32_t FeatureConfigFlags;
	dl_batt_cfg_data_t BattConfigData;
	uint8_t LaserCalibrationData[LASER_CALIBRATION_DATA_LEN];
	dl_temperature_cfg_data_t TemperatureConfigData; 
	dl_humidity_cfg_data_t HumidityConfigData;
	dl_ambient_light_cfg_data_t AmbientLightConfigData;
	dl_soiling_cfg_data_t SoilingDetectionConfigData;
	uint16_t AssistanceLightPeriod;
	uint16_t OscillatorTuning;
	dl_buzzer_cfg_data_t BuzzerConfigData;
	uint8_t rsrv2[15];
	uint16_t CRC;
} __attribute__((packed)) dl_cfg_calib_data_type;

typedef struct  
{
	uint8_t rsrv2[4];//Used for FW build
	uint32_t FirstActivation;
	uint16_t FirstBatteryLevel;
	uint16_t FirstBatteryImpedance;
	uint8_t rsrv[4];
} __attribute__((packed)) dl_one_time_data_t;

typedef struct  
{
	uint16_t MainLogbookIndex;
	uint8_t DemountingIndex;
	uint8_t BatteryLevelIndex;
	uint8_t BatteryImpedanceIndex;
	uint8_t SmokeEventsIndex;
	uint8_t COEventsIndex;
	uint8_t HeatEventsIndex;
	uint8_t FaultEventsIndex;
	uint8_t rsrv[7];
} __attribute__((packed)) dl_main_indexes_t;

typedef struct  
{
	uint16_t RstPOR;
	uint16_t RstPIN;
	uint16_t RstEM4;
	uint16_t RstWDOG0;
	uint16_t RstWDOG1;
	uint16_t RstLOCKUP;
	uint16_t RstSYSREQ; 
	uint16_t RstDVDDBOD;
	uint16_t RstDVDDLEBOD;
	uint16_t RstDECBOD;
	uint16_t RstAVDDBOD;
	uint16_t RstIOVDDBOD;
} __attribute__((packed)) dl_reset_counters_t;

typedef struct  
{
	uint16_t FaultDegradedSmokeChamber;
	uint16_t FaultObstacleDetected;
	uint16_t FaultSoilDetected;
	uint16_t FaultCoverageDetected;
	uint16_t FaultTemperatureOutOfBound; 
	uint16_t FaultHumidityOutOfBound;
	uint16_t FaultTestButton;
	uint16_t FaultDemountedTooLong;
	uint16_t FaultCalibrationDataCorrupt;
  uint16_t FaultObstacleBISTOverdue;
  uint16_t FaultObstacleDetectionOverdue;
  uint16_t FaultBuzzerCheckOverdue;
} __attribute__((packed)) dl_minor_fault_counters_t;

typedef struct  
{
	dl_one_time_data_t OneTimeData;
	dl_main_indexes_t MainIndexes;
	uint32_t LatestTimeStamp;
	uint32_t LatestUserBISTTimeStamp;
	uint8_t OperatingState;
	uint8_t rsrv0[7];
	dl_monthly_batt_level_t MonthlyBattLevels[MAX_SUPPORTED_NUM_BATTERY_LEVELS];
	uint8_t rsrv1[2];
	dl_monthly_batt_impedance_t MonthlyBattImpedances[MAX_SUPPORTED_NUM_BATTERY_IMPEDANCES];
	uint16_t FaultBattVoltageLevel;
	uint32_t BattFaultReportingTime;
	uint16_t FaultBattImpedanceLevel;
	uint8_t BattID;
	uint8_t rsrv2[5];
	uint32_t SmokeEventTimestamps[MAX_SUPPORTED_EVENTS];
	uint8_t rsrv3[8];
	uint16_t SmokeThermoptekState;
	uint16_t SmokeLocalEventCount;
	uint16_t SmokeRemoteEventCount;
	uint8_t SmokeEventTypes[MAX_SUPPORTED_EVENTS];
	uint32_t COEventTimestamps[MAX_SUPPORTED_EVENTS];
	uint8_t COEventTypes[MAX_SUPPORTED_EVENTS];
	uint16_t COVariance32HourAvg;
	uint16_t COLocalEventCount;
	uint16_t CORemoteEventCount;
	uint32_t HeatEventTimestamps[MAX_SUPPORTED_EVENTS];
	uint8_t HeatEventTypes[MAX_SUPPORTED_EVENTS];
	uint16_t HeatLocalEventCount;
	uint16_t HeatRemoteEventCount;
	uint16_t FaultEventCount;
	uint32_t FaultEventTimestamps[MAX_SUPPORTED_EVENTS];
	uint32_t FaultEventCodes[MAX_SUPPORTED_EVENTS];
	uint32_t FaultDuration; 
	uint32_t DemountedStateDuration;
	uint16_t UserBISTCount;
	uint16_t UserBISTCountExtended;
	uint16_t MountingEventCount;
	uint16_t DustCompensationLevel;
	uint8_t RadioDurationCounterAndData[16]; 
	uint32_t MCU1ToMCU2CommsDuration;
	uint32_t MCU2ToMCU1CommsDuration;
	uint32_t MCU1ToMCU2InstigatedCommsCount;
	uint32_t MCU2ToMCU1InstigatedCommsCound;
	uint32_t AssistanceLightTotalDuration;
	uint16_t AssistanceLightActivationCount;
	uint16_t AiringConfigChangeCount;
	uint32_t AiringRecommendActivationCount;
	uint32_t AiringRecommendTotalDuration;
	uint16_t SoilingActivationCount;
	uint32_t SoilingdetectionTotalDuration;
	uint16_t SoilingDetectionObscurationLvl;
	dl_reset_counters_t ResetCounter;
	dl_minor_fault_counters_t MinorFaultCounter;
	uint8_t rsrv4[8];
} __attribute__((packed)) dl_static_location_data_t;

/********************************************************************************************************
*********************************************************************************************************
*                                            Local Global Variables
*********************************************************************************************************
********************************************************************************************************/
static uint8_t gau8_working_buff[WORKING_BUFF_LEN] = {0};

static uint16_t demounting_logbook_event_id = 0xFFFFu;

static uint16_t event_logbook_event_id = 0xFFFFu;

/**
 * @brief This is true when EEPROM CRC is correct
 */
static bool eeprom_ok;

/**
 * @brief Used to protect event/demounting logbook access
 */
static OS_MUTEX logbook_mutex;

/**
 * @brief Logbook event status flags
 * 
 * This array containts the status of the logbook events. The flag are only intended to be used
 * for start only or start/end event pairs. The flag would be maintained for the start event only
 * and not for the end event.
 * 
 * This status is then checked to stop duplicate start and end events from being added to the log.
 * There cannot be another start event unitl an end event occurs. Start only events will work for
 * example when it's terminal and there would never be an end event.
 * 
 * @note The table is organised for performance so there are some wasted entries, but this could be
 *       optimised in the future. 
 */
static bool logbook_flags[ DEF_LBE_MAX ];

/**
 * @brief This is true when reset cause log messages send to mcu2
 */
static bool resetCauseMCU = false;

/**
 * @brief This is true when general log messages send to mcu2
 */
static bool logMsgtoMCU = false;

/********************************************************************************************************
*********************************************************************************************************
*                                            Static Functions Prototypes
*********************************************************************************************************
********************************************************************************************************/

static bool DataLogging_WriteData(const void *pvdata, uint16_t address, uint16_t len);
static void DataLogging_ReadData(void* pvdata, uint16_t address, uint16_t len);
static void DataLogging_SetSmokeEventTimestamp(uint8_t index, uint32_t SmokeEventTimestamp);
static uint32_t DataLogging_GetSmokeEventTimestamp(uint8_t index);
static void DataLogging_SetSmokeLocalEventCount(uint16_t SmokeLocalEventCount);
static void DataLogging_SetSmokeRemoteEventCount(uint16_t SmokeRemoteEventCount);
static void DataLogging_SetSmokeEventType(uint8_t index, uint8_t SmokeEventTypes);
static uint8_t DataLogging_GetSmokeEventType(uint8_t index);
static void DataLogging_SetCOEventTimestamp(uint8_t index, uint32_t COEventTimestamp);
static uint32_t DataLogging_GetCOEventTimestamp(uint8_t index);
static void DataLogging_SetCOLocalEventCount(uint16_t COLocalEventCount);
static void DataLogging_SetCORemoteEventCount(uint16_t CORemoteEventCount);
static void DataLogging_SetCOEventType(uint8_t index, uint8_t COEventTypes);
static uint8_t DataLogging_GetCOEventType(uint8_t index);
static void DataLogging_SetHeatEventType(uint8_t index, uint8_t HeatEventTypes);
static uint8_t DataLogging_GetHeatEventType(uint8_t index);
static void DataLogging_SetHeatEventTimestamp(uint8_t index, uint32_t HeatEventTimestamp);
static uint32_t DataLogging_GetHeatEventTimestamp(uint8_t index);
static void DataLogging_SetHeatLocalEventCount(uint16_t HeatLocalEventCount);
static void DataLogging_SetHeatRemoteEventCount(uint16_t HeatRemoteEventCount);
static void DataLogging_SetFaultEventCount(uint16_t FaultEventCount);
static void DataLogging_SetFaultEventTimestamp(uint8_t index, uint32_t FaultEventTimestamp);
static uint32_t DataLogging_GetFaultEventTimestamp(uint8_t index);
static void DataLogging_SetFaultEventCode(uint8_t index, uint32_t Code);
static uint32_t DataLogging_GetFaultEventCode(uint8_t index);
static void DataLogging_SetMainLogbookIndex(uint16_t MainLogbookIndex);
static void DataLogging_SetDemountingIndex(uint8_t DemountingIndex);
static void DataLogging_SetBatteryLevelIndex(uint8_t BatteryLevelIndex);
static void DataLogging_SetBatteryImpedanceIndex(uint8_t BatteryImpedanceIndex);
static void DataLogging_SetSmokeEventsIndex(uint8_t SmokeEventsIndex);
static void DataLogging_SetCOEventsIndex(uint8_t COEventsIndex);
static void DataLogging_SetHeatEventsIndex(uint8_t HeatEventsIndex);
static void DataLogging_SetFaultsEventsIndex(uint8_t FaultEventsIndex);

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

/*******************************************************************************
 * @brief   Is contents of the EEPROM ok
 *
 * @details This function returns the CRC status of the EEPROM when it was last
 *          checked.
 * 
 * @return	true if ok
 */
bool data_logging_is_eeprom_ok( void )
{
	return ( eeprom_ok );
}

static void lock_logbook( void )
{
  RTOS_ERR err;

  OSMutexPend( &logbook_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err );
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

static void unlock_logbook( void )
{
  RTOS_ERR err;

  OSMutexPost( &logbook_mutex, OS_OPT_POST_NONE, &err );
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

/****************************************************************************************************//**
*                                         DataLogging_CheckDataIntergrity()
*
* @brief	Check for the data integrity of the data saved in the EEPROM
*
*
* @return	true in case of valid data, false otherwise
********************************************************************************************************/
bool DataLogging_CheckDataIntergrity( void )
{
	DataLogging_ReadData( gau8_working_buff, DATALOGGING_CALIB_CONFIG_ADDR_OFFSET, ( uint16_t )sizeof( dl_cfg_calib_data_type ) );

	const uint16_t crc = CRC_UartCalculate( gau8_working_buff, ( uint16_t )( sizeof( dl_cfg_calib_data_type ) ) );
	
	eeprom_ok = ( crc == 0u );

	return( eeprom_ok );
}


/****************************************************************************************************//**
*                                           DataLogging_SetFWBuild()
*
* @brief  Write FW build in the EEPROM
*
* @param  data  FW Build
*********************************************************************************************************/
void DataLogging_SetFWBuild(void) {
  uint8_t buff[4] = {0, FW_MAJOR_REV, FW_MINOR_REV, FW_BUILD_REV};
  uint16_t addr = DATALOGGING_STATIC_LOCATION_ADDR_OFFSET;
  DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_SetDeviceID()
*
* @brief	Write device ID in the EEPROM
*
* @param	data	device ID
********************************************************************************************************/
void DataLogging_SetDeviceID(uint32_t data) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DeviceID));
	CommonUtils_Uint32ToUint8(data, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetDeviceID()
*
* @brief	Read Device ID from the EEPROM
*
* @return	device ID
********************************************************************************************************/
uint32_t DataLogging_GetDeviceID(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DeviceID));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetFw()
*
* @brief	Write firmware revision in the EEPROM
*
* @param	pstr_fw_rev Firmware Revision Structure
********************************************************************************************************/
void DataLogging_SetFw(const dl_fw_rev_data_t *pstr_fw_rev) {
	uint8_t buff[5] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, FwRev));
	CommonUtils_Uint16ToUint8(pstr_fw_rev->FwNumber, buff);
	buff[2] = pstr_fw_rev->FwRevMajor;
	buff[3] = pstr_fw_rev->FwRevMinor;
	buff[4] = pstr_fw_rev->FwRevBuild;
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetFw()
*
* @brief	Read firmware revision from the EEPROM
*
* @param	pstr_fw_rev		pointer to fw rev structure
********************************************************************************************************/
void DataLogging_GetFw(dl_fw_rev_data_t *pstr_fw_rev) {
	uint8_t buff[5] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, FwRev));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	pstr_fw_rev->FwNumber = CommonUtils_Uint8ToUint16(buff);
	pstr_fw_rev->FwRevMajor = buff[2];
	pstr_fw_rev->FwRevMinor = buff[3];
	pstr_fw_rev->FwRevBuild = buff[4];
}

/****************************************************************************************************//**
*                                           DataLogging_SetDeviceConfig()
*
* @brief	Write device configuration in the EEPROM
*
* @param	devCfg	device configuration
********************************************************************************************************/
void DataLogging_SetDeviceConfig(uint8_t devCfg) {
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DeviceConfig));
	DataLogging_WriteData(&devCfg, addr, sizeof(devCfg));
}

/****************************************************************************************************//**
*                                           DataLogging_GetDeviceConfig()
*
* @brief	Read device configuration from the EEPROM
*
* @return	device configuration
********************************************************************************************************/
uint8_t DataLogging_GetDeviceConfig(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DeviceConfig));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                           DataLogging_SetDoM()
*
* @brief	Write date of manufacturing in the EEPROM
*
* @param	DateOfManufacture	date of manufacturing
********************************************************************************************************/
void DataLogging_SetDoM(uint32_t DateOfManufacture) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DateOfManufacture));
	CommonUtils_Uint32ToUint8(DateOfManufacture, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetDoM()
*
* @brief	Read date of manufacturing from the EEPROM
*
* @return	date of manufacturing
********************************************************************************************************/
uint32_t DataLogging_GetDoM(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_product_mfg_data_t, DateOfManufacture));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/**
 * @brief Read nA per ppm
 * 
 * @details This function reads the CO parameter nA per ppm from the EEPROM
 * 
 * @return nA per ppm
 */
uint32_t data_logging_get_na_per_ppm( void )
{
	uint8_t buf[ 2 ] = { 0 };

	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, na_per_ppm ) );

	DataLogging_ReadData( buf, addr, sizeof( buf ) );

	return CommonUtils_Uint8ToUint16( buf );
}

/**
 * @brief Write nA per ppm
 * 
 * @details This function write the CO parameter nA per ppm to the EEPROM
 * 
 * @param[in] na_per_ppm   na per ppm
 */
void data_logging_set_na_per_ppm( const uint16_t na_per_ppm )
{
	uint8_t buf[ 2 ] = { 0 };

	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, na_per_ppm ) );

	CommonUtils_Uint16ToUint8( na_per_ppm, buf );

	DataLogging_WriteData( buf, addr, sizeof( buf ) );
}

/*******************************************************************************
 * @brief   Get CO calibration temperature
 *
 * @details This function returns the calibration temperature that was calculated
 *          when the CO sensor was calibrated
 * 
 * @see DATALOGGING_CO_CONFIG_DATA_OFFSET dl_co_cfg_data_t DataLogging_ReadData()
 * 
 * @note The temperature code is defined as follows:
 * 
 * Valid Temperatures: <20.0 : 26.0> in Celsius degrees.
 * Temperature AB.XY oC is stored as an integer 10*B + X.
 * e.g. 21.31 oC should be stored as 13.
 * 
 * @return Temperature code
 */
uint8_t DataLogging_GetCOCalibTemperature( void )
{
	uint8_t data = 0U;

	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, CO_calib_temperature ) );

	DataLogging_ReadData ( &data, addr, sizeof( data ) );

	return( data );
}

/*******************************************************************************
 * @brief   Set CO calibration temperature
 *
 * @details This function sets the calibration temperature that was calculated
 *          when the CO sensor was calibrated
 * 
 * @param[in] temperature    Temperature code
 * 
 * @see DATALOGGING_CO_CONFIG_DATA_OFFSET dl_co_cfg_data_t DataLogging_WriteData()
 * 
 * @note The temperature code is defined as follows:
 * 
 * Valid Temperatures: <20.0 : 26.0> in Celsius degrees.
 * Temperature AB.XY oC is stored as an integer 10*B + X.
 * e.g. 21.31 oC should be stored as 13.
 */
void DataLogging_SetCOCalibTemperature( const uint8_t temperature )
{
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, CO_calib_temperature ) );

	DataLogging_WriteData( &temperature, addr, sizeof( temperature ) );
}

/*******************************************************************************
 * @brief   Get CO calibration humisidty
 *
 * @details This function returns the calibration humidity that was calculated
 *          when the CO sensor was calibrated
 * 
 * @see DATALOGGING_CO_CONFIG_DATA_OFFSET dl_co_cfg_data_t DataLogging_ReadData()
 * 
 * @return Humidity percentage
 */
uint8_t DataLogging_GetCOCalibHumidity( void )
{
	uint8_t data = 0U;

	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, CO_calib_humidity ) );

	DataLogging_ReadData ( &data, addr, sizeof( data ) );

	return( data );
}

/*******************************************************************************
 * @brief   Set CO calibration humisidty
 *
 * @details This function sets the calibration humidity that was calculated
 *          when the CO sensor was calibrated
 * 
 * @see DATALOGGING_CO_CONFIG_DATA_OFFSET dl_co_cfg_data_t DataLogging_WriteData()
 */
void DataLogging_SetCOCalibHumidity( const uint8_t humidity )
{
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + ( uint16_t )( offsetof( dl_co_cfg_data_t, CO_calib_humidity ) );

	DataLogging_WriteData( &humidity, addr, sizeof( humidity ) );
}

/****************************************************************************************************//**
*                                           DataLogging_SetCOCal()
*
* @brief	Write CO Cal in the EEPROM
*
* @param	CoCal	CO Cal
********************************************************************************************************/
void DataLogging_SetCOCal(uint16_t CoCal) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COCal));
	CommonUtils_Uint16ToUint8(CoCal, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOCal()
*
* @brief	Read CO Cal from the EEPROM
*
* @return	CO Cal
********************************************************************************************************/
uint16_t DataLogging_GetCOCal(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COCal));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetCOAcqPeriod()
*
* @brief	Write CO acquisition period in the EEPROM
*
* @param	COAcqPeriod	CO acquisition period
********************************************************************************************************/
void DataLogging_SetCOAcqPeriod(uint8_t COAcqPeriod) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COAcqPeriod));
	DataLogging_WriteData(&COAcqPeriod, addr, sizeof(COAcqPeriod));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOAcqPeriod()
*
* @brief	Read CO acquisition period from the EEPROM
*
* @return	CO acquisition period
********************************************************************************************************/
uint8_t DataLogging_GetCOAcqPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COAcqPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                           DataLogging_SetCOBistPeriod()
*
* @brief	Write CO bist period in the EEPROM
*
* @param	COBistPeriod	CO bist period
********************************************************************************************************/
void DataLogging_SetCOBistPeriod(uint8_t COBistPeriod) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COBistPeriod));
	DataLogging_WriteData(&COBistPeriod, addr, sizeof(COBistPeriod));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOBistPeriod()
*
* @brief	Read CO bist period from the EEPROM
*
* @return	CO bist period
********************************************************************************************************/
uint8_t DataLogging_GetCOBistPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COBistPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                      DataLogging_SetCOOpenCircuitStrikeCount()
*
* @brief	Write CO open circuit strike count in the EEPROM
*
* @param	COOCStrikeCount	CO open circuit strike count
********************************************************************************************************/
void DataLogging_SetCOOpenCircuitStrikeCount(uint8_t COOCStrikeCount) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COOCStrikeCount));
	DataLogging_WriteData(&COOCStrikeCount, addr, sizeof(COOCStrikeCount));
}

/****************************************************************************************************//**
*                                      DataLogging_GetCOOpenCircuitStrikeCount()
*
* @brief	Read CO open circuit strike count from the EEPROM
*
* @return	CO open circuit strike count
********************************************************************************************************/
uint8_t DataLogging_GetCOOpenCircuitStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COOCStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                     DataLogging_SetCOShortCircuitStrikeCount()
*
* @brief	Write CO short circuit strike count in the EEPROM
*
* @param	COSCStrikeCount	CO short circuit strike count
********************************************************************************************************/
void DataLogging_SetCOShortCircuitStrikeCount(uint8_t COSCStrikeCount) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COSCStrikeCount));
	DataLogging_WriteData(&COSCStrikeCount, addr, sizeof(COSCStrikeCount));
}

/****************************************************************************************************//**
*                                     DataLogging_GetCOShortCircuitStrikeCount()
*
* @brief	Read CO short circuit strike count from the EEPROM
*
* @return	CO short circuit strike count
********************************************************************************************************/
uint8_t DataLogging_GetCOShortCircuitStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COSCStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                        DataLogging_SetCOFatigueStrikeCount()
*
* @brief	Write CO fatigue strike count in the EEPROM
*
* @param	COFatigeStrikeCount	CO fatigue strike count
********************************************************************************************************/
void DataLogging_SetCOFatigueStrikeCount(uint8_t COFatigeStrikeCount) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COFatigeStrikeCount));
	DataLogging_WriteData(&COFatigeStrikeCount, addr, sizeof(COFatigeStrikeCount));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCOFatigueStrikeCount()
*
* @brief	Read CO fatigue strike count from the EEPROM
*
* @return	CO fatigue strike count
********************************************************************************************************/
uint8_t DataLogging_GetCOFatigueStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COFatigeStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                        DataLogging_SetCOVarianceStrikeCount()
*
* @brief	Write CO variance strike count in the EEPROM
*
* @param	COVarianceStrikeCount	CO variance strike count
********************************************************************************************************/
void DataLogging_SetCOVarianceStrikeCount(uint16_t COVarianceStrikeCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COVarianceStrikeCount));
	CommonUtils_Uint16ToUint8(COVarianceStrikeCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCOVarianceStrikeCount()
*
* @brief	Read CO variance strike count from the EEPROM
*
* @return	CO variance strike count
********************************************************************************************************/
uint16_t DataLogging_GetCOVarianceStrikeCount(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COVarianceStrikeCount));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetCOFalseAlarmStrikeCount()
*
* @brief	Write CO false alarm strike count in the EEPROM
*
* @param	COFalseAlarmStrikeCount	CO false alarm strike count
********************************************************************************************************/
void DataLogging_SetCOFalseAlarmStrikeCount(uint8_t COFalseAlarmStrikeCount) {
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COFalseAlarmStrikeCount));
	DataLogging_WriteData(&COFalseAlarmStrikeCount, addr, sizeof(COFalseAlarmStrikeCount));
}

/****************************************************************************************************//**
*                                       DataLogging_GetCOFalseAlarmStrikeCount()
*
* @brief	Read CO false alarm strike count from the EEPROM
*
* @return	CO false alarm strike count
********************************************************************************************************/
uint8_t DataLogging_GetCOFalseAlarmStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COFalseAlarmStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                           DataLogging_SetCOHBThreshold()
*
* @brief	Write CO HB threshold in the EEPROM
*
* @param	COHBThresold	CO HB threshold
********************************************************************************************************/
void DataLogging_SetCOHBThreshold(uint16_t COHBThresold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COHBThresold));
	CommonUtils_Uint16ToUint8(COHBThresold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOHBThreshold()
*
* @brief	Read CO HB threshold from the EEPROM
*
* @return	CO HB threshold
********************************************************************************************************/
uint16_t DataLogging_GetCOHBThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COHBThresold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetCOVarianceThreshold()
*
* @brief	Write CO variance threshold in the EEPROM
*
* @param	COVarianceThreshold	CO variance threshold
********************************************************************************************************/
void DataLogging_SetCOVarianceThreshold(uint16_t COVarianceThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COVarianceThreshold));
	CommonUtils_Uint16ToUint8(COVarianceThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOVarianceThreshold()
*
* @brief	Read CO variance threshold from the EEPROM
*
* @return	CO variance threshold
********************************************************************************************************/
uint16_t DataLogging_GetCOVarianceThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COVarianceThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetCOShortCircuitThreshold()
*
* @brief	Write CO short circuit threshold in the EEPROM
*
* @param	COShortCircuitThreshold	CO short circuit threshold
********************************************************************************************************/
void DataLogging_SetCOShortCircuitThreshold(uint16_t COShortCircuitThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COShortCircuitThreshold));
	CommonUtils_Uint16ToUint8(COShortCircuitThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetCOShortCircuitThreshold()
*
* @brief	Read CO short circuit threshold from the EEPROM
*
* @return	CO short circuit threshold
********************************************************************************************************/
uint16_t DataLogging_GetCOShortCircuitThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COShortCircuitThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetCOOpenCircuitThreshold()
*
* @brief	Write CO open circuit threshold in the EEPROM
*
* @param	COOpenCircuitThreshold	CO open circuit threshold
********************************************************************************************************/
void DataLogging_SetCOOpenCircuitThreshold(uint16_t COOpenCircuitThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COOpenCircuitThreshold));
	CommonUtils_Uint16ToUint8(COOpenCircuitThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCOOpenCircuitThreshold()
*
* @brief	Read CO open circuit threshold from the EEPROM
*
* @return	CO open circuit threshold
********************************************************************************************************/
uint16_t DataLogging_GetCOOpenCircuitThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COOpenCircuitThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetSuperCOThreshold()
*
* @brief	Write super CO threshold in the EEPROM
*
* @param	COSuperCOThreshold	super CO threshold
********************************************************************************************************/
void DataLogging_SetSuperCOThreshold(uint16_t COSuperCOThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COSuperCOThreshold));
	CommonUtils_Uint16ToUint8(COSuperCOThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetSuperCOThreshold()
*
* @brief	Read super CO threshold from the EEPROM
*
* @return	super CO threshold
********************************************************************************************************/
uint16_t DataLogging_GetSuperCOThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_CO_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_co_cfg_data_t, COSuperCOThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}


/****************************************************************************************************//**
*                                          DataLogging_SetSmokeCalRefThreshold()
*
* @brief	Write smoke calibration reflection threshold in the EEPROM
*
* @param	SmokeCalRefThreshold	smoke calibration reflection threshold
********************************************************************************************************/
void DataLogging_SetSmokeCalRefThreshold(uint16_t SmokeCalRefThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCalRefThreshold));
	CommonUtils_Uint16ToUint8(SmokeCalRefThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetSmokeCalRefThreshold()
*
* @brief	Read smoke calibration reflection threshold from the EEPROM
*
* @return	smoke calibration reflection threshold
********************************************************************************************************/
uint16_t DataLogging_GetSmokeCalRefThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCalRefThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetSmokeCalThreshold()
*
* @brief	Write smoke calibration threshold in the EEPROM
*
* @param	SmokeCalThreshold	smoke calibration threshold
********************************************************************************************************/
void DataLogging_SetSmokeCalThreshold(uint16_t SmokeCalThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCalThreshold));
	CommonUtils_Uint16ToUint8(SmokeCalThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetSmokeCalThreshold()
*
* @brief	Read smoke calibration threshold from the EEPROM
*
* @return	smoke calibration threshold
********************************************************************************************************/
uint16_t DataLogging_GetSmokeCalThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCalThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                             DataLogging_SetSmokeAcqPeriod()
*
* @brief	Write smoke acquisition period in the EEPROM
*
* @param	SmokeAcqPeriod	smoke acquisition period
********************************************************************************************************/
void DataLogging_SetSmokeAcqPeriod(uint8_t SmokeAcqPeriod) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAcqPeriod));
	DataLogging_WriteData(&SmokeAcqPeriod, addr, sizeof(SmokeAcqPeriod));
}

/****************************************************************************************************//**
*                                             DataLogging_GetSmokeAcqPeriod()
*
* @brief	Read smoke acquisition period from the EEPROM
*
* @return	smoke acquisition period
********************************************************************************************************/
uint8_t DataLogging_GetSmokeAcqPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAcqPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                           DataLogging_SetSmokeBISTPeriod()
*
* @brief	Write smoke bist period in the EEPROM
*
* @param	SmokeBistPeriod	smoke bist period
********************************************************************************************************/
void DataLogging_SetSmokeBISTPeriod(uint16_t SmokeBistPeriod) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeBistPeriod));
	DataLogging_WriteData(&SmokeBistPeriod, addr, sizeof(SmokeBistPeriod));
}

/****************************************************************************************************//**
*                                           DataLogging_GetSmokeBISTPeriod()
*
* @brief	Read smoke bist period from the EEPROM
*
* @return	smoke bist period
********************************************************************************************************/
uint8_t DataLogging_GetSmokeBISTPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeBistPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                           DataLogging_SetAlarmSmokeThreshold()
*
* @brief	Write alarm smoke threshold in the EEPROM
*
* @param	AlarmSmokeThreshold	alarm smoke threshold
********************************************************************************************************/
void DataLogging_SetAlarmSmokeThreshold(uint16_t AlarmSmokeThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, AlarmSmokeThreshold));
	CommonUtils_Uint16ToUint8(AlarmSmokeThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetAlarmSmokeThreshold()
*
* @brief	Read alarm smoke threshold from the EEPROM
*
* @return	alarm smoke threshold
********************************************************************************************************/
uint16_t DataLogging_GetAlarmSmokeThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, AlarmSmokeThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                              DataLogging_SetSmokeAFEGain()
*
* @brief	Write smoke AFE gain in the EEPROM
*
* @param	SmokeAFEGain	smoke AFE gain
********************************************************************************************************/
void DataLogging_SetSmokeAFEGain(uint16_t SmokeAFEGain) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAFEGain));
	CommonUtils_Uint16ToUint8(SmokeAFEGain, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                              DataLogging_GetSmokeAFEGain()
*
* @brief	Read smoke AFE gain from the EEPROM
*
* @return	smoke AFE gain
********************************************************************************************************/
uint16_t DataLogging_GetSmokeAFEGain(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAFEGain));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetSmokeAFEIntegration()
*
* @brief	Write smoke AFE integration in the EEPROM
*
* @param	SmokeAFEIntegration	smoke AFE integration
********************************************************************************************************/
void DataLogging_SetSmokeAFEIntegration(uint8_t SmokeAFEIntegration) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAFEIntegration));
	DataLogging_WriteData(&SmokeAFEIntegration, addr, sizeof(SmokeAFEIntegration));
}

/****************************************************************************************************//**
*                                           DataLogging_GetSmokeAFEIntegration()
*
* @brief	Read smoke AFE integration from the EEPROM
*
* @return	smoke AFE integration
********************************************************************************************************/
uint8_t DataLogging_GetSmokeAFEIntegration(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeAFEIntegration));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                             DataLogging_SetSmokeLEDCurrent()
*
* @brief	Write smoke led current in the EEPROM
*
* @param	SmokeLedCurrent	smoke led current
********************************************************************************************************/
void DataLogging_SetSmokeLEDCurrent(uint16_t SmokeLedCurrent) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeLedCurrent));
	CommonUtils_Uint16ToUint8(SmokeLedCurrent, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                             DataLogging_GetSmokeLEDCurrent()
*
* @brief	Read smoke led current from the EEPROM
*
* @return	smoke led current
********************************************************************************************************/
uint16_t DataLogging_GetSmokeLEDCurrent(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeLedCurrent));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetSmokeADCSettlingPeriod()
*
* @brief	Write smoke adc settling period in the EEPROM
*
* @param	SmokeADCSettlingPeriod	smoke adc settling period
********************************************************************************************************/
void DataLogging_SetSmokeADCSettlingPeriod(uint8_t SmokeADCSettlingPeriod) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeADCSettlingPeriod));
	DataLogging_WriteData(&SmokeADCSettlingPeriod, addr, sizeof(SmokeADCSettlingPeriod));
}

/****************************************************************************************************//**
*                                        DataLogging_GetSmokeADCSettlingPeriod()
*
* @brief	Read smoke adc settling period from the EEPROM
*
* @return	smoke adc settling period
********************************************************************************************************/
uint8_t DataLogging_GetSmokeADCSettlingPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeADCSettlingPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                      DataLogging_SetSmokeCorrectionHistorySize()
*
* @brief	Write smoke correction history size in the EEPROM
*
* @param	SmokeCorrectionHistorySize	smoke correction history size
********************************************************************************************************/
void DataLogging_SetSmokeCorrectionHistorySize(uint8_t SmokeCorrectionHistorySize) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCorrectionHistorySize));
	DataLogging_WriteData(&SmokeCorrectionHistorySize, addr, sizeof(SmokeCorrectionHistorySize));
}

/****************************************************************************************************//**
*                                      DataLogging_GetSmokeCorrectionHistorySize()
*
* @brief	Read smoke correction history size from the EEPROM
*
* @return	smoke correction history size 
********************************************************************************************************/
uint8_t DataLogging_GetSmokeCorrectionHistorySize(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCorrectionHistorySize));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                         DataLogging_SetSmokeCircuitFaultThreshold()
*
* @brief	Write smoke circuit fault threshold  in the EEPROM
*
* @param	SmokeCircuitFaultThreshold	smoke circuit fault threshold 
********************************************************************************************************/
void DataLogging_SetSmokeCircuitFaultThreshold(uint16_t SmokeCircuitFaultThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCircuitFaultThreshold));
	CommonUtils_Uint16ToUint8(SmokeCircuitFaultThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetSmokeCircuitFaultThreshold()
*
* @brief	Read smoke circuit fault threshold from the EEPROM
*
* @return	Smoke circuit fault threshold
********************************************************************************************************/
uint16_t DataLogging_GetSmokeCircuitFaultThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeCircuitFaultThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetSmokeFastFlameThreshold()
*
* @brief	Write smoke fast flame threshold in the EEPROM
*
* @param	SmokeFastFlameThreshold	smoke fast flame threshold
********************************************************************************************************/
void DataLogging_SetSmokeFastFlameThreshold(uint16_t SmokeFastFlameThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeFastFlameThreshold));
	CommonUtils_Uint16ToUint8(SmokeFastFlameThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));;
}

/****************************************************************************************************//**
*                                         DataLogging_GetSmokeFastFlameThreshold()
*
* @brief	Read smoke fast flame threshold from the EEPROM
*
* @return	smoke fast flame threshold
********************************************************************************************************/
uint16_t DataLogging_GetSmokeFastFlameThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeFastFlameThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                          DataLogging_SetMaxSmokeDustCorrect()
*
* @brief	Write smoke dust correction max in the EEPROM
*
* @param	SmokeDustCorrectionMax	smoke dust correction max
********************************************************************************************************/
void DataLogging_SetMaxSmokeDustCorrect(uint8_t SmokeDustCorrectionMax) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeDustCorrectionMax));
	DataLogging_WriteData(&SmokeDustCorrectionMax, addr, sizeof(SmokeDustCorrectionMax));
}

/****************************************************************************************************//**
*                                          DataLogging_GetMaxSmokeDustCorrection()
*
* @brief	Read smoke dust correction max from the EEPROM
*
* @return	smoke dust correction max
********************************************************************************************************/
uint8_t DataLogging_GetMaxSmokeDustCorrection(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeDustCorrectionMax));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetSmokeDustCorrectMin()
*
* @brief	Write smoke dust correction min in the EEPROM
*
* @param	SmokeDustCorrectionMin	smoke dust correction min
********************************************************************************************************/
void DataLogging_SetSmokeDustCorrectMin(uint8_t SmokeDustCorrectionMin) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeDustCorrectionMin));
	DataLogging_WriteData(&SmokeDustCorrectionMin, addr, sizeof(SmokeDustCorrectionMin));
}

/****************************************************************************************************//**
*                                          DataLogging_GetSmokeDustCorrectionMin()
*
* @brief	Read smoke dust correction min from the EEPROM
*
* @return	smoke dust correction min
********************************************************************************************************/
uint8_t DataLogging_GetSmokeDustCorrectionMin(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeDustCorrectionMin));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetSmokeChamberFaultStrikeCount()
*
* @brief	Write smoke chamber fault strike count in the EEPROM
*
* @param	SmokeChamberFaultStrikeCount	smoke chamber fault strike count
********************************************************************************************************/
void DataLogging_SetSmokeChamberFaultStrikeCount(uint8_t SmokeChamberFaultStrikeCount) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeChamberFaultStrikeCount));
	DataLogging_WriteData(&SmokeChamberFaultStrikeCount, addr, sizeof(SmokeChamberFaultStrikeCount));
}

/****************************************************************************************************//**
*                                        DataLogging_GetSmokeChamberFaultStrikeCount()
*
* @brief	Read smoke chamber fault strike count from the EEPROM
*
* @return	smoke chamber fault strike count
********************************************************************************************************/
uint8_t DataLogging_GetSmokeChamberFaultStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeChamberFaultStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetSmokeHwFaultStrikeCount()
*
* @brief	Write smoke hardware fault strike count in the EEPROM
*
* @param	SmokeHwFaultStrikeCount	smoke hardware fault strike count
********************************************************************************************************/
void DataLogging_SetSmokeHwFaultStrikeCount(uint8_t SmokeHwFaultStrikeCount) {
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeHwFaultStrikeCount));
	DataLogging_WriteData(&SmokeHwFaultStrikeCount, addr, sizeof(SmokeHwFaultStrikeCount));
}

/****************************************************************************************************//**
*                                       DataLogging_GetSmokeHwFaultStrikeCount()
*
* @brief	Read smoke hardware fault strike count from the EEPROM
*
* @return	smoke hardware fault strike count
********************************************************************************************************/
uint8_t DataLogging_GetSmokeHwFaultStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_SMOKE_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_smoke_cfg_data_t, SmokeHwFaultStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}


/****************************************************************************************************//**
*                                           DataLogging_SetHeatAcqPeriod()
*
* @brief	Write heat acquisition period in the EEPROM
*
* @param	HeatAcqPeriod	heat acquisition period
********************************************************************************************************/
void DataLogging_SetHeatAcqPeriod(uint8_t HeatAcqPeriod) {
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatAcqPeriod));
	DataLogging_WriteData(&HeatAcqPeriod, addr, sizeof(HeatAcqPeriod));
}

/****************************************************************************************************//**
*                                           DataLogging_GetHeatAcqPeriod()
*
* @brief	Read heat acquisition period from the EEPROM
*
* @return	heat acquisition period
********************************************************************************************************/
uint8_t DataLogging_GetHeatAcqPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatAcqPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                      		 DataLogging_SetHeatBistPeriod()
*
* @brief	Write heat bist period in the EEPROM
*
* @param	HeatBistPeriod	heat bist period
********************************************************************************************************/
void DataLogging_SetHeatBistPeriod(uint8_t HeatBistPeriod) {
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatBistPeriod));
	DataLogging_WriteData(&HeatBistPeriod, addr, sizeof(HeatBistPeriod));
}

/****************************************************************************************************//**
*                                        	 DataLogging_GetHeatBistPeriod()
*
* @brief	Read heat bist period from the EEPROM
*
* @return	heat bist period
********************************************************************************************************/
uint8_t DataLogging_GetHeatBistPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatBistPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetHeatBatteryCompValues()
*
* @brief	Write battery compensation values in the EEPROM
*
* @param	BatteryCompValues	battery compensation values
********************************************************************************************************/
void DataLogging_SetHeatBatteryCompValues(uint32_t BatteryCompValues) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, BatteryCompValues));
	CommonUtils_Uint32ToUint8(BatteryCompValues, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetHeatBatteryCompValues()
*
* @brief	Read battery compensation values from the EEPROM
*
* @return	battery compensation values
********************************************************************************************************/
uint32_t DataLogging_GetHeatBatteryCompValues(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, BatteryCompValues));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetSuperHeatThreshold()
*
* @brief	Write super heat threshold in the EEPROM
*
* @param	SuperHeatThreshold	super heat threshold
********************************************************************************************************/
void DataLogging_SetSuperHeatThreshold(int16_t SuperHeatThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, SuperHeatThreshold));
	CommonUtils_Uint16ToUint8(SuperHeatThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetSuperHeatThreshold()
*
* @brief	Read super heat threshold from the EEPROM
*
* @return	super heat threshold
********************************************************************************************************/
int16_t DataLogging_GetSuperHeatThreshold(void) {
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, SuperHeatThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}

/****************************************************************************************************//**
*                                         DataLogging_SetMaxHeatOutOfBounds()
*
* @brief	Write heat out of bounds max in the EEPROM
*
* @param	HeatOutOfBoundsMax	heat out of bounds max
********************************************************************************************************/
void DataLogging_SetMaxHeatOutOfBounds(uint16_t HeatOutOfBoundsMax) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatOutOfBoundsMax));
	CommonUtils_Uint16ToUint8(HeatOutOfBoundsMax, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetMaxHeatOutOfBounds()
*
* @brief	Read heat out of bounds max from the EEPROM
*
* @return	heat out of bounds max
********************************************************************************************************/
uint16_t DataLogging_GetMaxHeatOutOfBounds(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatOutOfBoundsMax));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetMinHeatOutOfBounds()
*
* @brief	Write heat out of bounds min in the EEPROM
*
* @param	HeatOutOfBoundsMin	heat out of bounds min
********************************************************************************************************/
void DataLogging_SetMinHeatOutOfBounds(uint16_t HeatOutOfBoundsMin) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatOutOfBoundsMin));
	CommonUtils_Uint16ToUint8(HeatOutOfBoundsMin, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetMinHeatOutOfBounds()
*
* @brief	Read heat out of bounds min from the EEPROM
*
* @return	heat out of bounds min
********************************************************************************************************/
uint16_t DataLogging_GetMinHeatOutOfBounds(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatOutOfBoundsMin));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetHeatFaultStrikeCount()
*
* @brief	Write heat fault strike count in the EEPROM
*
* @param	HeatFaultStrikeCount	heat fault strike count
********************************************************************************************************/
void DataLogging_SetHeatFaultStrikeCount(uint8_t HeatFaultStrikeCount) {
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatFaultStrikeCount));
	DataLogging_WriteData(&HeatFaultStrikeCount, addr, sizeof(HeatFaultStrikeCount));
}

/****************************************************************************************************//**
*                                        DataLogging_GetHeatFaultStrikeCount()
*
* @brief	Read heat fault strike count from the EEPROM
*
* @return	heat fault strike count
********************************************************************************************************/
uint8_t DataLogging_GetHeatFaultStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, HeatFaultStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCoeffThirtyDegRiseInThirtySec()
*
* @brief	Write Heat IIR Coeff 0
*
* @param	ThirtyDegRiseInThirtySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffThirtyDegRiseInThirtySec(int16_t ThirtyDegRiseInThirtySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, ThirtyDegRiseInThirtySec));
	CommonUtils_Uint16ToUint8(ThirtyDegRiseInThirtySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffThirtyDegRiseInThirtySec()
*
* @brief	Read Heat IIR Coeff 0
*
* @return	IIR Coeff 0
********************************************************************************************************/
int16_t DataLogging_GetCoeffThirtyDegRiseInThirtySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, ThirtyDegRiseInThirtySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCoeffTwentyDegRiseInThirtySec()
*
* @brief	Write Heat IIR Coeff 1
*
* @param	TwentyDegRiseInThirtySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffTwentyDegRiseInThirtySec(int16_t TwentyDegRiseInThirtySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, TwentyDegRiseInThirtySec));
	CommonUtils_Uint16ToUint8(TwentyDegRiseInThirtySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffTwentyDegRiseInThirtySec()
*
* @brief	Read Heat IIR Coeff 1
*
* @return	IIR Coeff 1
********************************************************************************************************/
int16_t DataLogging_GetCoeffTwentyDegRiseInThirtySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, TwentyDegRiseInThirtySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCoeffTenDegRiseInSixtySec()
*
* @brief	Write Heat IIR Coeff 2
*
* @param	TenDegRiseInSixtySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffTenDegRiseInSixtySec(int16_t TenDegRiseInSixtySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, TenDegRiseInSixtySec));
	CommonUtils_Uint16ToUint8(TenDegRiseInSixtySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffTenDegRiseInSixtySec()
*
* @brief	Read Heat IIR Coeff 2
*
* @return	IIR Coeff 2
********************************************************************************************************/
int16_t DataLogging_GetCoeffTenDegRiseInSixtySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, TenDegRiseInSixtySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}


/****************************************************************************************************//**
*                                       DataLogging_SetCoeffFiveDegRiseInSixtySec()
*
* @brief	Write Heat IIR Coeff 3
*
* @param	FiveDegRiseInSixtySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffFiveDegRiseInSixtySec(int16_t FiveDegRiseInSixtySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, FiveDegRiseInSixtySec));
	CommonUtils_Uint16ToUint8(FiveDegRiseInSixtySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffFiveDegRiseInSixtySec()
*
* @brief	Read Heat IIR Coeff 3
*
* @return	IIR Coeff 3
********************************************************************************************************/
int16_t DataLogging_GetCoeffFiveDegRiseInSixtySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, FiveDegRiseInSixtySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCoeffThreeDegRiseInNinetySec()
*
* @brief	Write Heat IIR Coeff 4
*
* @param	ThreeDegRiseInNinetySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffThreeDegRiseInNinetySec(int16_t ThreeDegRiseInNinetySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, ThreeDegRiseInNinetySec));
	CommonUtils_Uint16ToUint8(ThreeDegRiseInNinetySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffThreeDegRiseInNinetySec()
*
* @brief	Read Heat IIR Coeff 4
*
* @return	IIR Coeff 4
********************************************************************************************************/
int16_t DataLogging_GetCoeffThreeDegRiseInNinetySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, ThreeDegRiseInNinetySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCoeffOneDegRiseInNinetySec()
*
* @brief	Write Heat IIR Coeff 5
*
* @param	OneDegRiseInNinetySec	heat fault strike count
********************************************************************************************************/
void DataLogging_SetCoeffOneDegRiseInNinetySec(int16_t OneDegRiseInNinetySec)
{
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, OneDegRiseInNinetySec));
	CommonUtils_Uint16ToUint8(OneDegRiseInNinetySec, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetCoeffOneDegRiseInNinetySec()
*
* @brief	Read Heat IIR Coeff 5
*
* @return	IIR Coeff 5
********************************************************************************************************/
int16_t DataLogging_GetCoeffOneDegRiseInNinetySec(void)
{
	uint8_t buff[2] = {0};
	int16_t retVal;
	uint16_t tmpVal;
	uint16_t addr = DATALOGGING_HEAT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_heat_cfg_data_t, OneDegRiseInNinetySec));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	tmpVal = CommonUtils_Uint8ToUint16(buff);
	retVal = (*((int16_t *) &tmpVal));
	return retVal;
}


/****************************************************************************************************//**
*                                         DataLogging_SetLowBatteryThreshold()
*
* @brief	Write low battery threshold in the EEPROM
*
* @param	LowBatteryThreshold	low battery threshold
********************************************************************************************************/
void DataLogging_SetLowBatteryThreshold(uint16_t LowBatteryThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, LowBatteryThreshold));
	CommonUtils_Uint16ToUint8(LowBatteryThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetLowBatteryThreshold()
*
* @brief	Read low battery threshold from the EEPROM
*
* @return	low battery threshold
********************************************************************************************************/
uint16_t DataLogging_GetLowBatteryThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, LowBatteryThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetBatteryDropThreshold()
*
* @brief	Write battery drop threshold in the EEPROM
*
* @param	BatteryDropThreshold	battery drop threshold
********************************************************************************************************/
void DataLogging_SetBatteryDropThreshold(uint16_t BatteryDropThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryDropThreshold));
	CommonUtils_Uint16ToUint8(BatteryDropThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetBatteryDropThreshold()
*
* @brief	Read battery drop threshold from the EEPROM
*
* @return	battery drop threshold
********************************************************************************************************/
uint16_t DataLogging_GetBatteryDropThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryDropThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetBatteryBistPeriod()
*
* @brief	Write battery bist period in the EEPROM
*
* @param	BatteryBistPeriod	battery bist period
********************************************************************************************************/
void DataLogging_SetBatteryBistPeriod(uint8_t BatteryBistPeriod) {
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryBistPeriod));
	DataLogging_WriteData(&BatteryBistPeriod, addr, sizeof(BatteryBistPeriod));
}

/****************************************************************************************************//**
*                                         DataLogging_GetBatteryBistPeriod()
*
* @brief	Read battery bist period from the EEPROM
*
* @return	battery bist period
********************************************************************************************************/
uint8_t DataLogging_GetBatteryBistPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryBistPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetBatteryBistStrikeSount()
*
* @brief	Write battery bist strike count in the EEPROM
*
* @param	BatteryBistStrikeCount	battery bist strike count
********************************************************************************************************/
void DataLogging_SetBatteryBistStrikeCount(uint8_t BatteryBistStrikeCount) {
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryBistStrikeCount));
	DataLogging_WriteData(&BatteryBistStrikeCount, addr, sizeof(BatteryBistStrikeCount));
}

/****************************************************************************************************//**
*                                       DataLogging_GetBatteryBistStrikeCount()
*
* @brief	Read battery bist strike count from the EEPROM
*
* @return	battery bist strike count
********************************************************************************************************/
uint8_t DataLogging_GetBatteryBistStrikeCount(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, BatteryBistStrikeCount));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetDeadBatteryThreshold()
*
* @brief	Write dead battery threshold in the EEPROM
*
* @param	DeadBatteryThreshold	dead battery threshold
********************************************************************************************************/
void DataLogging_SetDeadBatteryThreshold(uint16_t DeadBatteryThreshold) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, DeadBatteryThreshold));
	CommonUtils_Uint16ToUint8(DeadBatteryThreshold, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetDeadBatteryThreshold()
*
* @brief	Read dead battery threshold from the EEPROM
*
* @return	dead battery threshold
********************************************************************************************************/
uint16_t DataLogging_GetDeadBatteryThreshold(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, DeadBatteryThreshold));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetDeadBatteryBistPeriod()
*
* @brief	Write dead battery bist period in the EEPROM
*
* @param	DeadBatteryBistPeriod	dead battery bist period
********************************************************************************************************/
void DataLogging_SetDeadBatteryBistPeriod(uint8_t DeadBatteryBistPeriod) {
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, DeadBatteryBistPeriod));
	DataLogging_WriteData(&DeadBatteryBistPeriod, addr, sizeof(DeadBatteryBistPeriod));
}

/****************************************************************************************************//**
*                                        DataLogging_GetDeadBatteryBistPeriod()
*
* @brief	Read dead battery bist period from the EEPROM
*
* @return	dead battery bist period
********************************************************************************************************/
uint8_t DataLogging_GetDeadBatteryBistPeriod(void) {
	uint8_t data = 0;
	uint16_t addr = DATALOGGING_BATT_CONFIG_DATA_OFFSET + (uint16_t)(offsetof(dl_batt_cfg_data_t, DeadBatteryBistPeriod));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       	DataLogging_SetLaserCalData()
*
* @brief	Write laser calibration data in the EEPROM
*
* @param	LaserCalibrationData	pointer to laser calibration data
********************************************************************************************************/
void DataLogging_SetLaserCalData(const uint8_t LaserCalibrationData[LASER_CALIBRATION_DATA_LEN]) {
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, LaserCalibrationData)));
	DataLogging_WriteData(LaserCalibrationData, addr, LASER_CALIBRATION_DATA_LEN);
}

/****************************************************************************************************//**
*                                        	DataLogging_GetLaserCalData()
*
* @brief	Read laser calibration data from the EEPROM
*
* @param	LaserCalibrationData	pointer to laser calibration data
********************************************************************************************************/
void DataLogging_GetLaserCalData(uint8_t LaserCalibrationData[LASER_CALIBRATION_DATA_LEN]) {
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, LaserCalibrationData)));
	DataLogging_ReadData(LaserCalibrationData, addr, LASER_CALIBRATION_DATA_LEN);
}

/****************************************************************************************************//**
*                                  DataLogging_SetTempConfig()
*
* @brief	Write temperature configuration in the EEPROM
*
* @param	pstr_temp_config	pointer to temperature configuration
********************************************************************************************************/
void DataLogging_SetTempConfig(const dl_temperature_cfg_data_t *pstr_temp_config) {
	uint8_t buff[5] = {0};
	CommonUtils_Uint16ToUint8(pstr_temp_config->TemperatureValMin, &buff[0]);
	CommonUtils_Uint16ToUint8(pstr_temp_config->TemperatureValMax, &buff[2]);
	buff[4] = pstr_temp_config->TemperatureBistPeriod;
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, TemperatureConfigData)));
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                  DataLogging_GetTempConfig()
*
* @brief	Read temperature configuration from the EEPROM
*
* @param	pstr_temp_config	pointer to temperature configuration
********************************************************************************************************/
void DataLogging_GetTempConfig(dl_temperature_cfg_data_t *pstr_temp_config) {
	uint8_t buff[5] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, TemperatureConfigData)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	pstr_temp_config->TemperatureValMin = CommonUtils_Uint8ToUint16(&buff[0]);
	pstr_temp_config->TemperatureValMax = CommonUtils_Uint8ToUint16(&buff[2]);
	pstr_temp_config->TemperatureBistPeriod = buff[4];
}

/****************************************************************************************************//**
*                                  DataLogging_SetHumidityConfig()
*
* @brief	Write humidity configuration in the EEPROM
*
* @param	pstr_humidity_config	pointer to humidity configuration
********************************************************************************************************/
void DataLogging_SetHumidityConfig(const dl_humidity_cfg_data_t *pstr_humidity_config) {
  uint8_t buff[3] = {0};
  buff[0] = pstr_humidity_config->HumidityValMin;
  buff[1] = pstr_humidity_config->HumidityValMax;
  buff[2] = pstr_humidity_config->HumidityBistPeriod;
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, HumidityConfigData)));
	DataLogging_WriteData(buff, addr, sizeof(dl_humidity_cfg_data_t));
}

/****************************************************************************************************//**
*                                  DataLogging_GetHumidityConfig()
*
* @brief	Read humidity configuration from the EEPROM
*
* @param	pstr_humidity_config	pointer to humidity configuration
********************************************************************************************************/
void DataLogging_GetHumidityConfig(dl_humidity_cfg_data_t *pstr_humidity_config) {
  uint8_t buff[3] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, HumidityConfigData)));
	DataLogging_ReadData(buff, addr, sizeof(dl_humidity_cfg_data_t));
	pstr_humidity_config->HumidityValMin = buff[0];
	pstr_humidity_config->HumidityValMax = buff[1];
	pstr_humidity_config->HumidityBistPeriod = buff[2];

}

/****************************************************************************************************//**
*                                       DataLogging_SetAmbientLightConfig()
*
* @brief	Write ambient light configurations in the EEPROM
*
* @param	pstr_ambient_light_cfg	pointer to ambient light config data
********************************************************************************************************/
void DataLogging_SetAmbientLightConfig(const dl_ambient_light_cfg_data_t *pstr_ambient_light_cfg) {
  uint8_t buff[8] = {0};
  CommonUtils_Uint16ToUint8(pstr_ambient_light_cfg->AmbientLightThreshold, &buff[0]);
  buff[2] = pstr_ambient_light_cfg->AmbientLightAcqPeriod;
  buff[3] = pstr_ambient_light_cfg->AmbientLightBistPeriod;
  CommonUtils_Uint16ToUint8(pstr_ambient_light_cfg->AmbientLightValMin, &buff[4]);
  CommonUtils_Uint16ToUint8(pstr_ambient_light_cfg->AmbientLightValMax, &buff[6]);
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, AmbientLightConfigData)));
	DataLogging_WriteData(buff, addr, sizeof(dl_ambient_light_cfg_data_t));
}

/****************************************************************************************************//**
*                                       DataLogging_GetAmbientLightConfig()
*
* @brief	Read ambient light threshold from the EEPROM
*
* @param	pstr_ambient_light_cfg	pointer to ambient light threshold data
********************************************************************************************************/
void DataLogging_GetAmbientLightConfig(dl_ambient_light_cfg_data_t *pstr_ambient_light_cfg) {
  uint8_t buff[8] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, AmbientLightConfigData)));
	DataLogging_ReadData(buff, addr, sizeof(dl_ambient_light_cfg_data_t));
	pstr_ambient_light_cfg->AmbientLightThreshold = CommonUtils_Uint8ToUint16(&buff[0]);
	pstr_ambient_light_cfg->AmbientLightAcqPeriod = buff[2];
	pstr_ambient_light_cfg->AmbientLightBistPeriod = buff[3];
	pstr_ambient_light_cfg->AmbientLightValMin = CommonUtils_Uint8ToUint16(&buff[4]);
	pstr_ambient_light_cfg->AmbientLightValMax = CommonUtils_Uint8ToUint16(&buff[6]);


}

/*******************************************************************************
 * @brief   Write ABUF configuration to EEPROM
 *
 * @details This function writes the ABUF configuration to EEPROM. The values
 *          converted from Little to Big Endlian.
 * 
 *          Layout in EEPROM (Big Endian)
 * 
 *                 Bit 7                  Bit 4  Bit 3                 Bit 0
 *                 ---------------------------------------------------------
 *          Byte 0 | Offset (MSB)              | Offset                    |
 *                 ---------------------------------------------------------
 * 
 *                 ---------------------------------------------------------
 *          Byte 1 | Offset                    | Offset (LSB)              |
 *                 ---------------------------------------------------------
 *  
 *                 ---------------------------------------------------------
 *          Byte 3 | Gain (MSB)                | Gain                      |
 *                 ---------------------------------------------------------
 * 
 *                 ---------------------------------------------------------
 *          Byte 4 | Gain                      | Gain (LSB)                |
 *                 ---------------------------------------------------------
 *  
 * @param[in] ptr   Pointer to ABUF configuration
 */
void DataLogging_SetAbufConfig( dl_abuf_cfg_data_t * pstr_abuf_cfg )
{
  const uint8_t * ptr = ( uint8_t * )pstr_abuf_cfg;

  dl_abuf_cfg_data_t cfg;

  cfg.offset  = CommonUtils_Uint8ToUint16( ptr );

  cfg.gain    = CommonUtils_Uint8ToUint16( &ptr[ 2 ] );

	const uint16_t addr = ( uint16_t )( DATALOGGING_CALIB_CONFIG_ADDR_OFFSET +
                        ( uint16_t )( offsetof( dl_cfg_calib_data_type, abuf_config_data ) ) );

	DataLogging_WriteData( &cfg, addr, sizeof( dl_abuf_cfg_data_t ) );
}

/*******************************************************************************
 * @brief   Read ABUF configuration from EEPROM
 *
 * @details This function reads the ABUF configuration from EEPROM. The values
 *          converted from Little to Big Endlian.
 * 
 *          Layout in EEPROM (Big Endian)
 * 
 *                 Bit 7                  Bit 4  Bit 3                 Bit 0
 *                 ---------------------------------------------------------
 *          Byte 0 | Offset (MSB)              | Offset                    |
 *                 ---------------------------------------------------------
 * 
 *                 ---------------------------------------------------------
 *          Byte 1 | Offset                    | Offset (LSB)              |
 *                 ---------------------------------------------------------
 *  
 *                 ---------------------------------------------------------
 *          Byte 3 | Gain (MSB)                | Gain                      |
 *                 ---------------------------------------------------------
 * 
 *                 ---------------------------------------------------------
 *          Byte 4 | Gain                      | Gain (LSB)                |
 *                 ---------------------------------------------------------
 *  
 * @param[in] ptr   Pointer to ABUF configuration
 */
void DataLogging_GetAbufConfig( dl_abuf_cfg_data_t * pstr_abuf_cfg )
{
  uint8_t tbuf[ sizeof( dl_abuf_cfg_data_t ) ];

	const uint16_t addr = ( uint16_t )( DATALOGGING_CALIB_CONFIG_ADDR_OFFSET +
                        ( uint16_t )( offsetof( dl_cfg_calib_data_type, abuf_config_data ) ) );

	DataLogging_ReadData( tbuf, addr, sizeof( dl_abuf_cfg_data_t ) );

  pstr_abuf_cfg->offset = CommonUtils_Uint8ToUint16( tbuf );        /* Swap offset */
  
  pstr_abuf_cfg->gain = CommonUtils_Uint8ToUint16( &tbuf[ 2 ] );    /* Swap gain   */
}

/****************************************************************************************************//**
*                                       DataLogging_SetAssiatnceLightPeriod()
*
* @brief	Write assistance light period in the EEPROM
*
* @param	AssistanceLightPeriod	pointer to assistance light period data
********************************************************************************************************/
void DataLogging_SetAssiatnceLightPeriod(uint16_t AssistanceLightPeriod) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, AssistanceLightPeriod)));
	CommonUtils_Uint16ToUint8(AssistanceLightPeriod, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetAssistanceLightPeriod()
*
* @brief	Read assistance light period from the EEPROM
*
* @param	data	pointer to assistance light period data
********************************************************************************************************/
uint16_t DataLogging_GetAssistanceLightPeriod(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, AssistanceLightPeriod)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                           DataLogging_SetOscillatorTune()
*
* @brief	Write oscillator tune in the EEPROM
*
* @param	OscillatorTuning	pointer to oscillator tune data
********************************************************************************************************/
void DataLogging_SetOscillatorTune(uint16_t OscillatorTuning) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, OscillatorTuning)));
	CommonUtils_Uint16ToUint8(OscillatorTuning, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetOscillatorTune()
*
* @brief	Read oscillator tune from the EEPROM
*
* @param	data	pointer to oscillator tune data
********************************************************************************************************/
uint16_t DataLogging_GetOscillatorTune(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, OscillatorTuning)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                     DataLogging_SetBuzzerConfig()
*
* @brief	Write buzzer bist period and threshold in the EEPROM
*
* @param	pstr_buzzer_config	pointer to buzzer bist period and threshold data
********************************************************************************************************/
void DataLogging_SetBuzzerConfig(const dl_buzzer_cfg_data_t *pstr_buzzer_config) {
	uint8_t buff[3] = {0};
	CommonUtils_Uint16ToUint8(pstr_buzzer_config->BuzzerThreshold, &buff[0]);
	buff[2] = pstr_buzzer_config->BuzzerBistPeriod;
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, BuzzerConfigData)));
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                     DataLogging_GetBuzzerConfig()
*
* @brief	Read buzzer bist period and threshold from the EEPROM
*
* @param	pstr_buzzer_config	pointer to buzzer bist period and threshold data
********************************************************************************************************/
void DataLogging_GetBuzzerConfig(dl_buzzer_cfg_data_t *pstr_buzzer_config) {
	uint8_t buff[3] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, BuzzerConfigData)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	pstr_buzzer_config->BuzzerThreshold = CommonUtils_Uint8ToUint16(&buff[0]);
	pstr_buzzer_config->BuzzerBistPeriod = buff[2];
}

/****************************************************************************************************//**
*                                      			 DataLogging_SetCRC()
*
* @brief	Write crc in the EEPROM
*
* @note		This function reads back data from EEPROM, calculate CRC and writes it in the EEPROM
********************************************************************************************************/
void DataLogging_SetCRC(void) {
	uint8_t buff[2] = {0};
	uint16_t crc;
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, CRC)));
	/* read back all calibration section upto the CRC field*/
	DataLogging_ReadData(gau8_working_buff, (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET), (uint16_t)(offsetof(dl_cfg_calib_data_type, CRC)));		
	crc = CRC_UartCalculate(gau8_working_buff, (uint16_t)(offsetof(dl_cfg_calib_data_type, CRC)));						/* calculate crc 						*/
	CommonUtils_Uint16ToUint8(crc, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        		DataLogging_GetCRC()
*
* @brief	Read crc from the EEPROM
*
* @return	crc
********************************************************************************************************/
uint16_t DataLogging_GetCRC(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, CRC)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                      	   DataLogging_SetFirstActivation()
*
* @brief	Write first activation in the EEPROM
*
* @param	FirstActivation	first activation
********************************************************************************************************/
void DataLogging_SetFirstActivation(uint32_t FirstActivation) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstActivation)));
	CommonUtils_Uint32ToUint8(FirstActivation, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetFirstActivation()
*
* @brief	Read first activation from the EEPROM
*
* @return	first activation
********************************************************************************************************/
uint32_t DataLogging_GetFirstActivation(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstActivation)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetFirstBatteryLevel()
*
* @brief	Write first battery level in the EEPROM
*
* @param	FirstBatteryLevel	first battery level
********************************************************************************************************/
void DataLogging_SetFirstBatteryLevel(uint16_t FirstBatteryLevel) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstBatteryLevel)));
	CommonUtils_Uint16ToUint8(FirstBatteryLevel, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetFirstBatteryLevel()
*
* @brief	Read first battery level from the EEPROM
*
* @return	first battery level
********************************************************************************************************/
uint16_t DataLogging_GetFirstBatteryLevel(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstBatteryLevel)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetFirstBatteryImpedance()
*
* @brief	Write first battery impedance in the EEPROM
*
* @param	FirstBatteryImpedance	first battery impedance
********************************************************************************************************/
void DataLogging_SetFirstBatteryImpedance(uint16_t FirstBatteryImpedance) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstBatteryImpedance)));
	CommonUtils_Uint16ToUint8(FirstBatteryImpedance, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetFirstBatteryImpedance()
*
* @brief	Read first battery impedance from the EEPROM
*
* @return	first battery impedance
********************************************************************************************************/
uint16_t DataLogging_GetFirstBatteryImpedance(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_ONE_TIME_DATA_OFFSET + (uint16_t)(offsetof(dl_one_time_data_t, FirstBatteryImpedance)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                      DataLogging_SetFeatureConfigurationFlags()
*
* @brief	Write feature configuration flags in the EEPROM
*
* @param	FeatureConfigFlags	feature configuration flags
********************************************************************************************************/
void DataLogging_SetFeatureConfigurationFlags(uint32_t FeatureConfigFlags) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, FeatureConfigFlags)));
	CommonUtils_Uint32ToUint8(FeatureConfigFlags, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                      DataLogging_GetFeatureconfigurationFlags()
*
* @brief	Read feature configuration flags from the EEPROM
*
* @return	feature configuration flags
********************************************************************************************************/
uint32_t DataLogging_GetFeatureconfigurationFlags(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_PRODUCTION_MANUFACTURING_DATA_OFFSET + (uint16_t)(offsetof(dl_cfg_calib_data_type, FeatureConfigFlags)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                            DataLogging_ResetAllIndexes()
*
* @brief	Reset all indexes to zero
********************************************************************************************************/
void DataLogging_ResetAllIndexes(void) {
	DataLogging_SetMainLogbookIndex(0u);
	DataLogging_SetDemountingIndex(0u);
	DataLogging_SetSmokeEventsIndex(0u);
	DataLogging_SetCOEventsIndex(0u);
	DataLogging_SetHeatEventsIndex(0u);
	DataLogging_SetFaultsEventsIndex(0u);
	DataLogging_SetBatteryLevelIndex(0u);
	DataLogging_SetBatteryImpedanceIndex(0u);
}

/****************************************************************************************************//**
*                                       	DataLogging_SetMainLogbookIndex()
*
* @brief	Write main logbook index in the EEPROM
*
* @param	MainLogbookIndex	main logbook index
********************************************************************************************************/
void DataLogging_SetMainLogbookIndex(uint16_t MainLogbookIndex) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, MainLogbookIndex)));
	CommonUtils_Uint16ToUint8(MainLogbookIndex, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetMainLogbookIndex()
*
* @brief	Read main logbook index from the EEPROM
*
* @return	main logbook index
********************************************************************************************************/
uint16_t DataLogging_GetMainLogbookIndex(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, MainLogbookIndex)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/*******************************************************************************************************
* @brief    Get index of last entered event
*
* @details  This function obtains the index of the last event written to the logbook. If the logbook
            is empty then the index will point to an empty/undefined entry
*
* @return   Index of last entry
*
* @see      DataLogging_GetMainLogbookIndex() DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX
*
********************************************************************************************************/
uint16_t DataLogging_GetLastDemountingLogbookIndex( void )
{
	const uint16_t next_free_index = DataLogging_GetDemountingIndex( );

	return( next_free_index == 0 ? ( DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX - 1 ) : ( next_free_index - 1 ) );
}

/*******************************************************************************************************
* @brief    Get index of last entered event
*
* @details  This function obtains the index of the last event written to the logbook. If the logbook
            is empty then the index will point to an empty/undefined entry
*
* @return   Index of last entry
*
* @see      DataLogging_GetMainLogbookIndex() DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX
*
********************************************************************************************************/
uint16_t DataLogging_GetLastMainLogbookIndex( void )
{
	const uint16_t next_free_index = DataLogging_GetMainLogbookIndex( );

	return( next_free_index == 0 ? ( DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX - 1 ) : ( next_free_index - 1 ) );
}

/****************************************************************************************************//**
*                                       	DataLogging_SetDemountingIndex()
*
* @brief	Write demounting index in the EEPROM
*
* @param	DemountingIndex	demounting index
********************************************************************************************************/
void DataLogging_SetDemountingIndex(uint8_t DemountingIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, DemountingIndex)));
	DataLogging_WriteData(&DemountingIndex, addr, sizeof(DemountingIndex));
}

/****************************************************************************************************//**
*                                           DataLogging_GetDemountingIndex()
*
* @brief	Read demounting index from the EEPROM
*
* @return	demounting index
********************************************************************************************************/
uint8_t DataLogging_GetDemountingIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, DemountingIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       	DataLogging_SetBatteryLevelIndex()
*
* @brief	Write battery level index in the EEPROM
*
* @param	BatteryLevelIndex	battery level index
********************************************************************************************************/
void DataLogging_SetBatteryLevelIndex(uint8_t BatteryLevelIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, BatteryLevelIndex)));
	DataLogging_WriteData(&BatteryLevelIndex, addr, sizeof(BatteryLevelIndex));
}

/****************************************************************************************************//**
*                                           DataLogging_GetBatteryLevelIndex()
*
* @brief	Read battery level index from the EEPROM
*
* @return	battery level index
********************************************************************************************************/
uint8_t DataLogging_GetBatteryLevelIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, BatteryLevelIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                       	DataLogging_SetBatteryImpedanceIndex()
*
* @brief	Write battery impedance index in the EEPROM
*
* @param	BatteryImpedanceIndex	battery impedance index
********************************************************************************************************/
void DataLogging_SetBatteryImpedanceIndex(uint8_t BatteryImpedanceIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, BatteryImpedanceIndex)));
	DataLogging_WriteData(&BatteryImpedanceIndex, addr, sizeof(BatteryImpedanceIndex));
}

/****************************************************************************************************//**
*                                           DataLogging_GetBatteryImpedanceIndex()
*
* @brief	Read battery impedance index from the EEPROM
*
* @return	battery impedance index
********************************************************************************************************/
uint8_t DataLogging_GetBatteryImpedanceIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, BatteryImpedanceIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetSmokeEventsIndex()
*
* @brief	Write smoke events index in the EEPROM
*
* @param	SmokeEventsIndex	smoke events index
********************************************************************************************************/
void DataLogging_SetSmokeEventsIndex(uint8_t SmokeEventsIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, SmokeEventsIndex)));
	DataLogging_WriteData(&SmokeEventsIndex, addr, sizeof(SmokeEventsIndex));
}

/****************************************************************************************************//**
*                                          DataLogging_GetSmokeEventsIndex()
*
* @brief	Read smoke events index from the EEPROM
*
* @return	smoke events index
********************************************************************************************************/
uint8_t DataLogging_GetSmokeEventsIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, SmokeEventsIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                      	    DataLogging_SetCOEventsIndex()
*
* @brief	Write CO events index in the EEPROM
*
* @param	COEventsIndex	CO events index
********************************************************************************************************/
void DataLogging_SetCOEventsIndex(uint8_t COEventsIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, COEventsIndex)));
	DataLogging_WriteData(&COEventsIndex, addr, sizeof(COEventsIndex));
}

/****************************************************************************************************//**
*                                           DataLogging_GetCOEventsIndex()
*
* @brief	Read CO events index from the EEPROM
*
* @return	CO events index
********************************************************************************************************/
uint8_t DataLogging_GetCOEventsIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, COEventsIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetHeatEventsIndex()
*
* @brief	Write heat events index in the EEPROM
*
* @param	HeatEventsIndex	heat events index
********************************************************************************************************/
void DataLogging_SetHeatEventsIndex(uint8_t HeatEventsIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, HeatEventsIndex)));
	DataLogging_WriteData(&HeatEventsIndex, addr, sizeof(HeatEventsIndex));
}

/****************************************************************************************************//**
*                                          DataLogging_GetHeatEventsIndex()
*
* @brief	Read heat events index from the EEPROM
*
* @return	heat events index
********************************************************************************************************/
uint8_t DataLogging_GetHeatEventsIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, HeatEventsIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                         DataLogging_SetFaultsEventsIndex()
*
* @brief	Write faults events index in the EEPROM
*
* @param	FaultEventsIndex	faults events index
********************************************************************************************************/
void DataLogging_SetFaultsEventsIndex(uint8_t FaultEventsIndex) {
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, FaultEventsIndex)));
	DataLogging_WriteData(&FaultEventsIndex, addr, sizeof(FaultEventsIndex));
}

/****************************************************************************************************//**
*                                         DataLogging_GetFaultsEventsIndex()
*
* @brief	Read faults events index from the EEPROM
*
* @return	faults events index
********************************************************************************************************/
uint8_t DataLogging_GetFaultsEventsIndex(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_INDEXES_OFFSET + (uint16_t)(offsetof(dl_main_indexes_t, FaultEventsIndex)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetOperatingState()
*
* @brief	Write operating state in the EEPROM
*
* @param	OperatingState	operating state
********************************************************************************************************/
void DataLogging_SetOperatingState(uint8_t OperatingState) {
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, OperatingState)));
	DataLogging_WriteData(&OperatingState, addr, sizeof(OperatingState));
}

/****************************************************************************************************//**
*                                          DataLogging_GetOperatingState()
*
* @brief	Read operating state from the EEPROM
*
* @return	operating state
********************************************************************************************************/
uint8_t DataLogging_GetOperatingState(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, OperatingState)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                          DataLogging_SetLatestTimestamp()
*
* @brief	Write latest time stamp in the EEPROM
*
* @param	LatestTimeStamp	latest time stamp
********************************************************************************************************/
void DataLogging_SetLatestTimestamp(uint32_t LatestTimeStamp) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, LatestTimeStamp)));
	CommonUtils_Uint32ToUint8(LatestTimeStamp, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetLatestTimestamp()
*
* @brief	Read latest time stamp from the EEPROM
*
* @return	latest time stamp
********************************************************************************************************/
uint32_t DataLogging_GetLatestTimestamp(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, LatestTimeStamp)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                          DataLogging_SetLatestUserBistTimestamp()
*
* @brief	Write latest User BIST time stamp in the EEPROM
*
* @param	LatestUserBistTimeStamp	latest User BIST time stamp
********************************************************************************************************/
void DataLogging_SetLatestUserBistTimestamp(void)
{
	uint8_t buff[4] = {0};
	uint32_t LatestUserBistTimeStamp;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, LatestUserBISTTimeStamp)));
	LatestUserBistTimeStamp = get_currentTime();
	CommonUtils_Uint32ToUint8(LatestUserBistTimeStamp, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetLatestUserBistTimestamp()
*
* @brief	Read latest User BIST time stamp from the EEPROM
*
* @return	latest User BIST time stamp
********************************************************************************************************/
uint32_t DataLogging_GetLatestUserBistTimestamp(void)
{
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, LatestUserBISTTimeStamp)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}
/****************************************************************************************************//**
*                                       DataLogging_SetMonthlyBatteryLevel()
*
* @brief	Write monthly battery level in the EEPROM
*
* @param	level	monthly battery level for both batteries
********************************************************************************************************/
void DataLogging_SetMonthlyBatteryLevel(const dl_monthly_batt_level_t *pstr_level)
{
	uint8_t buff[4] = {0};
	uint8_t index;
	uint16_t address;

	index = DataLogging_GetBatteryLevelIndex(); /* Get events index 	*/
	if((index == 0xFFu) || (index == 66u))
	{
	   index = 0u;
	}
	DEBUG_DATA_LOGGING("\n\n index", true, (uint32_t)index);
	CommonUtils_Uint16ToUint8(pstr_level->Batt_A, buff);
	CommonUtils_Uint16ToUint8(pstr_level->Batt_B, &buff[2]);
	address = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MonthlyBattLevels[index % MAX_SUPPORTED_NUM_BATTERY_LEVELS])));
	DEBUG_DATA_LOGGING("\n address", true, (uint32_t)address);
	DataLogging_WriteData(buff, address, sizeof(buff));
	index++;
	DataLogging_SetBatteryLevelIndex(index);
}

/****************************************************************************************************//**
*                                       DataLogging_GetMonthlyBatteryLevel()
*
* @brief	Read monthly battery level from the EEPROM
*
* @return	monthly battery level
********************************************************************************************************/
void DataLogging_GetMonthlyBatteryLevel(uint8_t BattLevelIdx, dl_monthly_batt_level_t *pstr_level) {
	uint8_t buff[4u] = {0};

	if (BattLevelIdx < MAX_SUPPORTED_NUM_BATTERY_LEVELS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MonthlyBattLevels[BattLevelIdx])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		pstr_level->Batt_A = CommonUtils_Uint8ToUint16(buff); /* Copy data into structure */
		pstr_level->Batt_B = CommonUtils_Uint8ToUint16(&buff[2]);
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                     DataLogging_SetMonthlyBatteryImpedance()
*
* @brief	Write monthly battery impedance in the EEPROM
*
* @param	pstr_impedance	monthly battery impedance
********************************************************************************************************/
void DataLogging_SetMonthlyBatteryImpedance(const dl_monthly_batt_impedance_t *pstr_impedance) {
	uint8_t buff[4] = {0};
	uint8_t index;
	uint16_t address;

	index = DataLogging_GetBatteryImpedanceIndex(); /* Get events index 	*/
	if((index == 0xFFu)||(index == 66u))
	{
	   index = 0u;
	}
	DEBUG_DATA_LOGGING("\n\n index", true, (uint32_t)index);

	CommonUtils_Uint16ToUint8(pstr_impedance->Batt_A, buff);
	CommonUtils_Uint16ToUint8(pstr_impedance->Batt_B, &buff[2]);
	address = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MonthlyBattImpedances[index % MAX_SUPPORTED_NUM_BATTERY_IMPEDANCES])));
	DEBUG_DATA_LOGGING("\n address", true, (uint32_t)address);
	DataLogging_WriteData(buff, address, sizeof(buff));
	index++;
	DataLogging_SetBatteryImpedanceIndex(index);
}

/****************************************************************************************************//**
*                                     DataLogging_GetMonthlyBatteryImpedance()
*
* @brief	Read monthly battery impedance from the EEPROM
*
* @return	monthly battery impedance
********************************************************************************************************/
void DataLogging_GetMonthlyBatteryImpedance(uint8_t BattImpedanceIdx, dl_monthly_batt_impedance_t *pstr_impedance) {
	uint8_t buff[4u] = {0};

	if (BattImpedanceIdx < MAX_SUPPORTED_NUM_BATTERY_IMPEDANCES) /*ToDo: Check if we should add a check to verify the index has valid data*/
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MonthlyBattImpedances[BattImpedanceIdx])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		pstr_impedance->Batt_A = CommonUtils_Uint8ToUint16(buff); /* Copy data into structure */
		pstr_impedance->Batt_B = CommonUtils_Uint8ToUint16(&buff[2]);
	}
	else
	{
		/*Do nothing*/
	}
}
/****************************************************************************************************//**
*                                       	DataLogging_SetBatteryID()
*
* @brief	Write battery id in the EEPROM
*
* @param	BattID	battery id
********************************************************************************************************/
void DataLogging_SetBatteryID(uint8_t BattID) {
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, BattID)));
	DataLogging_WriteData(&BattID, addr, sizeof(BattID));
}

/****************************************************************************************************//**
*                                           DataLogging_GetBatteryID()
*
* @brief	Read battery id from the EEPROM
*
* @return	battery id
********************************************************************************************************/
uint8_t DataLogging_GetBatteryID(void) {
	uint8_t data = 0;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, BattID)));
	DataLogging_ReadData(&data, addr, sizeof(data));
	return data;
}

/****************************************************************************************************//**
*                                         DataLogging_SetBatteryFRTime()
*
* @brief	Write battery fault reporting time in the EEPROM
*
* @param	BattFaultReportingTime	battery fault reporting time
********************************************************************************************************/
void DataLogging_SetBatteryFRTime(uint32_t BattFaultReportingTime) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, BattFaultReportingTime)));
	CommonUtils_Uint32ToUint8(BattFaultReportingTime, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetBatteryFRTime()
*
* @brief	Read battery fault reporting time from the EEPROM
*
* @return	battery fault reporting time
********************************************************************************************************/
uint32_t DataLogging_GetBatteryFRTime(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, BattFaultReportingTime)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                     DataLogging_SetFaultyBatteryVoltageLevel()
*
* @brief	Write faulty battery voltage level in the EEPROM
*
* @param	FaultBattVoltageLevel	faulty battery voltage level
********************************************************************************************************/
void DataLogging_SetFaultyBatteryVoltageLevel(uint16_t FaultBattVoltageLevel) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultBattVoltageLevel)));
	CommonUtils_Uint16ToUint8(FaultBattVoltageLevel, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                     DataLogging_GetFaultyBatteryVoltageLevel()
*
* @brief	Read faulty battery voltage level from the EEPROM
*
* @return	faulty battery voltage level
********************************************************************************************************/
uint16_t DataLogging_GetFaultyBatteryVoltageLevel(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultBattVoltageLevel)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                    DataLogging_SetFaultyBatteryImpedanceLevel()
*
* @brief	Write faulty battery impedance level in the EEPROM
*
* @param	FaultBattImpedanceLevel	faulty battery impedance level
********************************************************************************************************/
void DataLogging_SetFaultyBatteryImpedanceLevel(uint16_t FaultBattImpedanceLevel) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultBattImpedanceLevel)));
	CommonUtils_Uint16ToUint8(FaultBattImpedanceLevel, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                    DataLogging_GetFaultyBatteryImpedanceLevel()
*
* @brief	Read faulty battery impedance level from the EEPROM
*
* @return	faulty battery impedance level
********************************************************************************************************/
uint16_t DataLogging_GetFaultyBatteryImpedanceLevel(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultBattImpedanceLevel)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                    		DataLogging_SetSmokeEvent()
*
* @brief	Write smoke event in the EEPROM
*
* @param	event_type	Event Type
********************************************************************************************************/
void DataLogging_SetSmokeEvent(uint8_t event_type) {
	uint8_t index;
	uint16_t count;
	uint32_t Timestamp;
	index = DataLogging_GetSmokeEventsIndex(); /* Get events index 	*/
	DEBUG_DATA_LOGGING("\n Smoke index", true, (uint32_t)index);
	if((index == 0xFFu) || (index == 10u))
	{
	   index = 0u;
	}
	Timestamp = get_currentTime();
	DataLogging_SetSmokeEventTimestamp((index % MAX_SUPPORTED_EVENTS), Timestamp);
	DataLogging_SetSmokeEventType((index % MAX_SUPPORTED_EVENTS), event_type);

	if (event_type == EVENT_TYPE_LOCAL)
	{ /* Update event count 	*/
		count = DataLogging_GetSmokeLocalEventCount();
		DEBUG_DATA_LOGGING("\n Smoke Local count", true, (uint32_t)count);
		count++;
		DataLogging_SetSmokeLocalEventCount(count);
	}
	else
	{
		count = DataLogging_GetSmokeRemoteEventCount();
		DEBUG_DATA_LOGGING("\n Smoke Remote count", true, (uint32_t)count);
		count++;
		DataLogging_SetSmokeRemoteEventCount(count);
	}
	index++;
	DataLogging_SetSmokeEventsIndex(index);
}

/****************************************************************************************************//**
*                                    	DataLogging_GetSmokeEvent()
*
* @brief	Read all smoke events from the EEPROM
*
* @return	events structure
********************************************************************************************************/
void DataLogging_GetSmokeEvent(uint8_t index, dl_event_t *pstr_smoke_event) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		pstr_smoke_event->Timestamp = DataLogging_GetSmokeEventTimestamp(index);			   /* get timestamp 	*/
		pstr_smoke_event->EventType = DataLogging_GetSmokeEventType(index); /* get event type 	*/
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                       DataLogging_SetSmokeEventTimestamp()
*
* @brief	Write smoke event time stamp in the EEPROM
*
* @param	index	index of EEPROM location to write
* 			SmokeEventTimestamp	smoke event time stamp
********************************************************************************************************/
static void DataLogging_SetSmokeEventTimestamp(uint8_t index, uint32_t SmokeEventTimestamp) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeEventTimestamps[index])));
		CommonUtils_Uint32ToUint8(SmokeEventTimestamp, buff);
		DataLogging_WriteData(buff, addr, sizeof(buff));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                       DataLogging_GetSmokeEventTimestamp()
*
* @brief	Read smoke event time stamp from the EEPROM
*
* @param	index	index of EEPROM location to read
*
* @return	smoke event time stamp
********************************************************************************************************/
static uint32_t DataLogging_GetSmokeEventTimestamp(uint8_t index) {

  	uint32_t TimeStamp = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeEventTimestamps[index])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		TimeStamp = CommonUtils_Uint8ToUint32(buff);
	}
	else
	{
		/*Do nothing*/
	}
	return TimeStamp;
}

/****************************************************************************************************//**
*                                       	DataLogging_SetSmokeEventType()
*
* @brief	Write smoke event type in the EEPROM
*
* @param	index	index of EEPROM location to write
* 			data	smoke event type
********************************************************************************************************/
static void DataLogging_SetSmokeEventType(uint8_t index, uint8_t SmokeEventTypes) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeEventTypes[index])));
		DataLogging_WriteData(&SmokeEventTypes, addr, sizeof(SmokeEventTypes));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                           DataLogging_GetSmokeEventType()
*
* @brief	Read smoke event type from the EEPROM
*
*
* @param	index	index of EEPROM location to read
*
* @return	smoke event type
********************************************************************************************************/
static uint8_t DataLogging_GetSmokeEventType(uint8_t index) {
	uint8_t data = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeEventTypes[index])));
		DataLogging_ReadData(&data, addr, sizeof(data));
	}
	else
	{
		/*Do nothing*/
	}
	return data;
}

/****************************************************************************************************//**
*                                        DataLogging_SetSmokeThermoptekState()
*
* @brief	Write smoke thermoptek state in the EEPROM
*
* @param	SmokeThermoptekState	smoke thermoptek state
********************************************************************************************************/
void DataLogging_SetSmokeThermoptekState(uint16_t SmokeThermoptekState) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeThermoptekState)));
	CommonUtils_Uint16ToUint8(SmokeThermoptekState, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetSmokeThermoptekState()
*
* @brief	Read smoke thermoptek state from the EEPROM
*
* @return	smoke thermoptek state
********************************************************************************************************/
uint16_t DataLogging_GetSmokeThermoptekState(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeThermoptekState)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetSmokeLocalEventCount()
*
* @brief	Write smoke local event count in the EEPROM
*
* @param	SmokeLocalEventCount	smoke local event count
********************************************************************************************************/
static void DataLogging_SetSmokeLocalEventCount(uint16_t SmokeLocalEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeLocalEventCount)));
	CommonUtils_Uint16ToUint8(SmokeLocalEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetSmokeLocalEventCount()
*
* @brief	Read smoke local event count from the EEPROM
*
* @return	smoke local event count
********************************************************************************************************/
uint16_t DataLogging_GetSmokeLocalEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0u;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeLocalEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	    count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                        DataLogging_SetSmokeRemoteEventCount()
*
* @brief	Write smoke remote event count in the EEPROM
*
* @param	data	smoke remote event count
********************************************************************************************************/
static void DataLogging_SetSmokeRemoteEventCount(uint16_t SmokeRemoteEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeRemoteEventCount)));
	CommonUtils_Uint16ToUint8(SmokeRemoteEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));

}

/****************************************************************************************************//**
*                                        DataLogging_GetSmokeRemoteEventCount()
*
* @brief	Read smoke remote event count from the EEPROM
*
* @return	smoke remote event count
********************************************************************************************************/
uint16_t DataLogging_GetSmokeRemoteEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, SmokeRemoteEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	   count = 0U;
	}
  return count;
}

/****************************************************************************************************//**
*                                    		  DataLogging_SetCOEvent()
*
* @brief	Write CO event in the EEPROM
*
* @param	event_type	Co event to log
********************************************************************************************************/
void DataLogging_SetCOEvent(uint8_t event_type) {
	uint8_t index;
	uint16_t count;
	uint32_t Timestamp;
	index = DataLogging_GetCOEventsIndex(); /* Get events index 	*/
	if((index == 0xFFu) || (index == 10u))
	{
	   index = 0u;
	}
	DEBUG_DATA_LOGGING("\n CO index", true, (uint32_t)index);
	Timestamp = get_currentTime();
	DataLogging_SetCOEventTimestamp((index % MAX_SUPPORTED_EVENTS), Timestamp);
	DataLogging_SetCOEventType((index % MAX_SUPPORTED_EVENTS), event_type);

	if (event_type == EVENT_TYPE_LOCAL)
	{ /* Update event count 	*/
		count = DataLogging_GetCOLocalEventCount();
		DEBUG_DATA_LOGGING("\n CO Local count", true, (uint32_t)count);
		count++;
		DataLogging_SetCOLocalEventCount(count);
	}
	else
	{
		count = DataLogging_GetCORemoteEventCount();
		DEBUG_DATA_LOGGING("\n CO Remote count", true, (uint32_t)count);
		count++;
		DataLogging_SetCORemoteEventCount(count);
	}
	index++;
	DataLogging_SetCOEventsIndex(index);
}

/****************************************************************************************************//**
*                                    	DataLogging_GetCOEvent()
*
* @brief	Read all CO events from the EEPROM
*
* @return	events structure
********************************************************************************************************/
void DataLogging_GetCOEvent(uint8_t index, dl_event_t *pstr_co_event) {
	if (index < MAX_SUPPORTED_EVENTS)
	{
		pstr_co_event->Timestamp = DataLogging_GetCOEventTimestamp(index);				/* get timestamp 	*/
		pstr_co_event->EventType = DataLogging_GetCOEventType(index); /* get event type 	*/
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                          DataLogging_SetCOEventTimestamp()
*
* @brief	Write CO event time stamp in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	COEventTimestamp	CO event time stamp
********************************************************************************************************/
static void DataLogging_SetCOEventTimestamp(uint8_t index, uint32_t COEventTimestamp) {
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COEventTimestamps[index])));
		CommonUtils_Uint32ToUint8(COEventTimestamp, buff);
		DataLogging_WriteData(buff, addr, sizeof(buff));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                          DataLogging_GetCOEventTimestamp()
*
* @brief	Read CO event time stamp from the EEPROM
*
* @param	index	address of EEPROM location to read
*
* @return	CO event time stamp
********************************************************************************************************/
static uint32_t DataLogging_GetCOEventTimestamp(uint8_t index) {

	uint32_t TimeStamp = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COEventTimestamps[index])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		TimeStamp = CommonUtils_Uint8ToUint32(buff);
	}
	else
	{
		/*Do nothing*/
	}
	return TimeStamp;
}

/****************************************************************************************************//**
*                                       	 DataLogging_SetCOEventType()
*
* @brief	Write CO event type in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	COEventTypes	CO event type
********************************************************************************************************/
static void DataLogging_SetCOEventType(uint8_t index, uint8_t COEventTypes) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COEventTypes[index])));
		DataLogging_WriteData(&COEventTypes, addr, sizeof(COEventTypes));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                            DataLogging_GetCOEventType()
*
* @brief	Read CO event type from the EEPROM
*
* @param	index	address of EEPROM location to read
*
* @return	CO event type
********************************************************************************************************/
static uint8_t DataLogging_GetCOEventType(uint8_t index) {
	uint8_t data = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COEventTypes[index])));
		DataLogging_ReadData(&data, addr, sizeof(data));
	}
	else
	{
		/*Do nothing*/
	}
	return data;
}

/****************************************************************************************************//**
*                                       DataLogging_SetCOVariance32HrCoverage()
*
* @brief	Write CO variance 32Hr coverage in the EEPROM
*
* @param	COVariance32HourAvg	CO variance 32Hr coverage
********************************************************************************************************/
void DataLogging_SetCOVariance32HrCoverage(uint16_t COVariance32HourAvg) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COVariance32HourAvg)));
	CommonUtils_Uint16ToUint8(COVariance32HourAvg, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetCOVariance32HrCoverage()
*
* @brief	Read CO variance 32Hr coverage from the EEPROM
*
* @return	CO variance 32Hr coverage
********************************************************************************************************/
uint16_t DataLogging_GetCOVariance32HrCoverage(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COVariance32HourAvg)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetCOLocalEventCount()
*
* @brief	Write CO local event count in the EEPROM
*
* @param	COLocalEventCount	CO local event count
********************************************************************************************************/
static void DataLogging_SetCOLocalEventCount(uint16_t COLocalEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COLocalEventCount)));
	CommonUtils_Uint16ToUint8(COLocalEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetCOLocalEventCount()
*
* @brief	Read CO local event count from the EEPROM
*
* @return	CO local event count
********************************************************************************************************/
uint16_t DataLogging_GetCOLocalEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, COLocalEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count =  CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	    count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                         DataLogging_SetCORemoteEventCount()
*
* @brief	Write CO remote event count in the EEPROM
*
* @param	CORemoteEventCount	CO remote event count
********************************************************************************************************/
static void DataLogging_SetCORemoteEventCount(uint16_t CORemoteEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, CORemoteEventCount)));
	CommonUtils_Uint16ToUint8(CORemoteEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetCORemoteEventCount()
*
* @brief	Read CO remote event count from the EEPROM
*
* @return	CO remote event count
********************************************************************************************************/
uint16_t DataLogging_GetCORemoteEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, CORemoteEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
  count = CommonUtils_Uint8ToUint16(buff);
  if(count == 0xFFFFU)
  {
     count = 0U;
  }
  return count;
}

/****************************************************************************************************//**
*                                    		DataLogging_SetHeatEvent()
*
* @brief	Write heat event in the EEPROM
*
* @param	event_type	type of event to log
********************************************************************************************************/
void DataLogging_SetHeatEvent(uint8_t event_type) {
	uint8_t index;
	uint16_t count;
	uint32_t Timestamp; 
	index = DataLogging_GetHeatEventsIndex(); /* Get events index 	*/
	if((index == 0xFFu) || (index == 10u))
	{
	    index = 0u;
	}

	DEBUG_DATA_LOGGING("\n Heat index", true, (uint32_t)index);
	Timestamp = get_currentTime();
	DataLogging_SetHeatEventTimestamp((index % MAX_SUPPORTED_EVENTS), Timestamp);
	DataLogging_SetHeatEventType((index % MAX_SUPPORTED_EVENTS), event_type);

	if (event_type == EVENT_TYPE_LOCAL)
	{ /* Update event count 	*/
		count = DataLogging_GetHeatLocalEventCount();
		DEBUG_DATA_LOGGING("\n Heat Local count", true, (uint32_t)count);
		count++;
		DataLogging_SetHeatLocalEventCount(count);
	}
	else
	{
		count = DataLogging_GetHeatRemoteEventCount();
		DEBUG_DATA_LOGGING("\n Heat Remote count", true, (uint32_t)count);
		count++;
		DataLogging_SetHeatRemoteEventCount(count);
	}
	index++;
	DataLogging_SetHeatEventsIndex(index);
}

/****************************************************************************************************//**
*                                    	DataLogging_GetHeatEvent()
*
* @brief	Read all heat events from the EEPROM
*
* @return	events structure
********************************************************************************************************/
void DataLogging_GetHeatEvent(uint8_t index, dl_event_t *pstr_heat_event) {
	if (index < MAX_SUPPORTED_EVENTS)
	{
		pstr_heat_event->Timestamp = DataLogging_GetHeatEventTimestamp(index);			   /* get timestamp 	*/
		pstr_heat_event->EventType = DataLogging_GetHeatEventType(index); /* get event type 	*/
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                          DataLogging_SetHeatEventTimestamp()
*
* @brief	Write heat event time stamp in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	HeatEventTimestamp	heat event time stamp
********************************************************************************************************/
static void DataLogging_SetHeatEventTimestamp(uint8_t index, uint32_t HeatEventTimestamp) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatEventTimestamps[index])));
		CommonUtils_Uint32ToUint8(HeatEventTimestamp, buff);
		DataLogging_WriteData(buff, addr, sizeof(buff));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                          DataLogging_GetHeatEventTimestamp()
*
* @brief	Read heat event time stamp from the EEPROM
*
* @param	index	index of EEPROM location to read
*
* @return	heat event time stamp
********************************************************************************************************/
static uint32_t DataLogging_GetHeatEventTimestamp(uint8_t index) {

  uint32_t TimeStamp = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatEventTimestamps[index])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		TimeStamp = CommonUtils_Uint8ToUint32(buff);
	}
	else
	{
		/*Do nothing*/
	}
	return TimeStamp;
}

/****************************************************************************************************//**
*                                            DataLogging_SetHeatEventType()
*
* @brief	Write heat event type in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	data	heat event type
********************************************************************************************************/
static void DataLogging_SetHeatEventType(uint8_t index, uint8_t HeatEventTypes) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatEventTypes[index])));
		DataLogging_WriteData(&HeatEventTypes, addr, sizeof(HeatEventTypes));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                            DataLogging_GetHeatEventType()
*
* @brief	Read heat event type from the EEPROM
*
* @param	index	index of EEPROM location to read
*
* @return	heat event time stamp
********************************************************************************************************/
static uint8_t DataLogging_GetHeatEventType(uint8_t index) {
	uint8_t data = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatEventTypes[index])));
		DataLogging_ReadData(&data, addr, sizeof(data));
	}
	else
	{
		/*Do nothing*/
	}
	return data;
}

/****************************************************************************************************//**
*                                         DataLogging_SetHeatLocalEventCount()
*
* @brief	Write heat local event count in the EEPROM
*
* @param	HeatLocalEventCount	heat local event count
********************************************************************************************************/
static void DataLogging_SetHeatLocalEventCount(uint16_t HeatLocalEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatLocalEventCount)));
	CommonUtils_Uint16ToUint8(HeatLocalEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetHeatLocalEventCount()
*
* @brief	Read heat local event count from the EEPROM
*
* @return	heat local event count
********************************************************************************************************/
uint16_t DataLogging_GetHeatLocalEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatLocalEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	   count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                        DataLogging_SetHeatRemoteEventCount()
*
* @brief	Write heat remote event count in the EEPROM
*
* @param	HeatRemoteEventCount	heat remote event count
********************************************************************************************************/
static void DataLogging_SetHeatRemoteEventCount(uint16_t HeatRemoteEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatRemoteEventCount)));
	CommonUtils_Uint16ToUint8(HeatRemoteEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetHeatRemoteEventCount()
*
* @brief	Read heat remote event count from the EEPROM
*
* @return	heat remote event count
********************************************************************************************************/
uint16_t DataLogging_GetHeatRemoteEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, HeatRemoteEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	   count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                           DataLogging_SetFault()
*
* @brief   Log fault in the EEPROM
*
* @param	Code 	fault code to log
********************************************************************************************************/
void DataLogging_SetFault(uint32_t Code)
{
	uint8_t index;
	uint16_t count;
	uint32_t Timestamp;
	index = DataLogging_GetFaultsEventsIndex(); /* Get events index 	*/
	if(index == 0xFFu)
	{
	   index = 0u;
	}
	DEBUG_DATA_LOGGING("\n Fault index", true, (uint32_t)index);
	Timestamp = get_currentTime();
	DataLogging_SetFaultEventTimestamp((index % MAX_SUPPORTED_EVENTS), Timestamp);
	DataLogging_SetFaultEventCode((index % MAX_SUPPORTED_EVENTS), Code);

	count = DataLogging_GetFaultEventCount(); /* Update fault event count 	*/
	if(count == 0xFFFFu)
	{
	    count = 0u;
	}
	count++;
	DataLogging_SetFaultEventCount(count);
	index++;
	DataLogging_SetFaultsEventsIndex(index);
}

/****************************************************************************************************//**
*                                           DataLogging_GetFault()
*
* @brief   Read faults from the EEPROM
*
* @return	fault codes from EEPROM
********************************************************************************************************/
void DataLogging_GetFault(uint8_t index, dl_fault_event_t *pstr_fault_event){

	if (index < MAX_SUPPORTED_EVENTS)
	{
		pstr_fault_event->Timestamp = DataLogging_GetFaultEventTimestamp(index); /* get timestamp 	*/
		pstr_fault_event->Code = DataLogging_GetFaultEventCode(index);			 /* get event type 	*/
	}
	else
	{
		/*Do nothing*/
	}
}
/****************************************************************************************************//**
*                                          DataLogging_SetFaultEventCount()
*
* @brief	Write fault event count in the EEPROM
*
* @param	FaultEventCount	fault event count
********************************************************************************************************/
void DataLogging_SetFaultEventCount(uint16_t FaultEventCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventCount)));
	CommonUtils_Uint16ToUint8(FaultEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                          DataLogging_GetFaultEventCount()
*
* @brief	Read fault event count from the EEPROM
*
* @return	fault event count
********************************************************************************************************/
uint16_t DataLogging_GetFaultEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetFaultEventTimestamp()
*
* @brief	Write fault event time stamp in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	FaultEventTimestamp	fault event time stamp
********************************************************************************************************/
static void DataLogging_SetFaultEventTimestamp(uint8_t index, uint32_t FaultEventTimestamp) {

	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventTimestamps[index])));
		CommonUtils_Uint32ToUint8(FaultEventTimestamp, buff);
		DataLogging_WriteData(buff, addr, sizeof(buff));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                         DataLogging_GetFaultEventTimestamp()
*
* @brief	Read fault event time stamp from the EEPROM
*
* @param	index	address of EEPROM location to read
*
* @return	fault event time stamp
********************************************************************************************************/
static uint32_t DataLogging_GetFaultEventTimestamp(uint8_t index) {

	uint32_t TimeStamp = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventTimestamps[index])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		TimeStamp = CommonUtils_Uint8ToUint32(buff);
	}
	else
	{
		/*Do nothing*/
	}
	return TimeStamp;
}

/****************************************************************************************************//**
*                                      		 DataLogging_SetFaultEventCode()
*
* @brief	Write fault event code in the EEPROM
*
* @param	index	address of EEPROM location to write
* @param	Code	fault event code
********************************************************************************************************/
static void DataLogging_SetFaultEventCode(uint8_t index, uint32_t Code) {
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventCodes[index])));
		CommonUtils_Uint32ToUint8(Code, buff);
		DataLogging_WriteData(buff, addr, sizeof(buff));
	}
	else
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                           DataLogging_GetFaultEventCode()
*
* @brief	Read fault event code from the EEPROM
*
* @param	index	address of EEPROM location to read
*
* @return	fault event code
********************************************************************************************************/
static uint32_t DataLogging_GetFaultEventCode(uint8_t index) {

	uint32_t code = 0;
	if (index < MAX_SUPPORTED_EVENTS)
	{
		uint8_t buff[4] = {0};
		uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultEventCodes[index])));
		DataLogging_ReadData(buff, addr, sizeof(buff));
		code = CommonUtils_Uint8ToUint32(buff);
	}
	else
	{
		/*Do nothing*/
	}
	return code;
}

/****************************************************************************************************//**
*                                       	DataLogging_SetFaultDuration()
*
* @brief	Write fault duration in the EEPROM
*
* @param	FaultDuration	fault duration
********************************************************************************************************/
void DataLogging_SetFaultDuration(uint32_t FaultDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultDuration)));
	CommonUtils_Uint32ToUint8(FaultDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetFaultDuration()
*
* @brief	Read fault duration from the EEPROM
*
* @return	fault duration
********************************************************************************************************/
uint32_t DataLogging_GetFaultDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, FaultDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                       	DataLogging_LogUserTest()
*
* @brief	Log user bist test in the EEPROM
********************************************************************************************************/
void DataLogging_LogUserTest(void) {
	uint8_t buff[2] = {0};
	uint16_t UserBISTCount;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, UserBISTCount)));
	UserBISTCount = DataLogging_GetUserTestCount();
	UserBISTCount++;
	CommonUtils_Uint16ToUint8(UserBISTCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                           DataLogging_GetUserTestCount()
*
* @brief	Read user bist count from the EEPROM
*
* @return	user bist count
********************************************************************************************************/
uint16_t DataLogging_GetUserTestCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, UserBISTCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	   count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                       DataLogging_LogUserExtTest()
*
* @brief	Log user bist extended test in the EEPROM
********************************************************************************************************/
void DataLogging_LogUserExtTest(void) {
	uint8_t buff[2] = {0};
	uint16_t UserExtdBISTCount = 0u;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, UserBISTCountExtended)));
	UserExtdBISTCount = DataLogging_GetUserExtTestCount();
	UserExtdBISTCount++;
	CommonUtils_Uint16ToUint8(UserExtdBISTCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetUserExtTestCount()
*
* @brief	Read user bist extended test count from the EEPROM
*
* @return	user bist extended test count
********************************************************************************************************/
uint16_t DataLogging_GetUserExtTestCount(void) {
	uint8_t buff[2] = {0};
	uint16_t count = 0U;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, UserBISTCountExtended)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFU)
	{
	   count = 0U;
	}
	return count;
}

/****************************************************************************************************//**
*                                       DataLogging_LogMountingEvent()
*
* @brief	Log mounting event count in the EEPROM
********************************************************************************************************/
void DataLogging_LogMountingEvent(void) {
	uint8_t buff[2] = {0};
	uint16_t MountingEventCount;
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MountingEventCount)));
	MountingEventCount = DataLogging_GetMountingEventCount();
	if(MountingEventCount == 0xFFFFu)
	{
	    MountingEventCount = 0u;
	}
	MountingEventCount++;
	CommonUtils_Uint16ToUint8(MountingEventCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetMountingEventCount()
*
* @brief	Read mounting event count from the EEPROM
*
* @return	mounting event count
********************************************************************************************************/
uint16_t DataLogging_GetMountingEventCount(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MountingEventCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                      DataLogging_SetDemountedStateDuration()
*
* @brief	Write demounted state duration in the EEPROM
*
* @param	DemountedStateDuration	demounted state duration
********************************************************************************************************/
void DataLogging_SetDemountedStateDuration(uint32_t DemountedStateDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, DemountedStateDuration)));
	CommonUtils_Uint32ToUint8(DemountedStateDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                      DataLogging_GetDemountedStateDuration()
*
* @brief	Read demounted state duration from the EEPROM
*
* @return	demounted state duration
********************************************************************************************************/
uint32_t DataLogging_GetDemountedStateDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, DemountedStateDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetDustCompensationLevel()
*
* @brief	Write dust compensation level in the EEPROM
*
* @param	DustCompensationLevel	dust compensation level
********************************************************************************************************/
void DataLogging_SetDustCompensationLevel(uint16_t DustCompensationLevel) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, DustCompensationLevel)));
	CommonUtils_Uint16ToUint8(DustCompensationLevel, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetDustCompensationLevel()
*
* @brief	Read dust compensation level from the EEPROM
*
* @return	dust compensation level
********************************************************************************************************/
uint16_t DataLogging_GetDustCompensationLevel(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, DustCompensationLevel)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetRadioDurationCounter()
*
* @brief	Write radio duration counter in the EEPROM
*
* @param	data	pointer to radio duration counter buffer
********************************************************************************************************/
void DataLogging_SetRadioDurationCounter(const uint8_t RadioDurationCounterAndData[RADIO_CALIBRATION_DATA_LEN]) {
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, RadioDurationCounterAndData)));
	DataLogging_WriteData(RadioDurationCounterAndData, addr, RADIO_CALIBRATION_DATA_LEN);
}

/****************************************************************************************************//**
*                                       DataLogging_GetRadioDurationCounter()
*
* @brief	Read radio duration counter from the EEPROM
*
* @param	data	pointer to radio duration counter buffer
********************************************************************************************************/
void DataLogging_GetRadioDurationCounter(uint8_t RadioDurationCounterAndData[RADIO_CALIBRATION_DATA_LEN]) {
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, RadioDurationCounterAndData)));
	DataLogging_ReadData(RadioDurationCounterAndData, addr, RADIO_CALIBRATION_DATA_LEN);
}

/****************************************************************************************************//**
*                                      DataLogging_SetMCU1toMCU2CommsDuration()
*
* @brief	Write mcu1 to mcu2 comms duration in the EEPROM
*
* @param	MCU1ToMCU2CommsDuration	mcu1 to mcu2 comms duration
********************************************************************************************************/
void DataLogging_SetMCU1toMCU2CommsDuration(uint32_t MCU1ToMCU2CommsDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU1ToMCU2CommsDuration)));
	CommonUtils_Uint32ToUint8(MCU1ToMCU2CommsDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                      DataLogging_GetMCU1toMCU2CommsDuration()
*
* @brief	Read mcu1 to mcu2 comms duration from the EEPROM
*
* @return	mcu1 to mcu2 comms duration
********************************************************************************************************/
uint32_t DataLogging_GetMCU1toMCU2CommsDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU1ToMCU2CommsDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                      DataLogging_SetMCU2toMCU1CommsDuration()
*
* @brief	Write mcu2 to mcu1 comms duration in the EEPROM
*
* @param	MCU2ToMCU1CommsDuration	mcu2 to mcu1 comms duration
********************************************************************************************************/
void DataLogging_SetMCU2toMCU1CommsDuration(uint32_t MCU2ToMCU1CommsDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU2ToMCU1CommsDuration)));
	CommonUtils_Uint32ToUint8(MCU2ToMCU1CommsDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                      DataLogging_GetMCU2toMCU1CommsDuration()
*
* @brief	Read mcu2 to mcu1 comms duration from the EEPROM
*
* @return	mcu2 to mcu1 comms duration
********************************************************************************************************/
uint32_t DataLogging_GetMCU2toMCU1CommsDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU2ToMCU1CommsDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                   DataLogging_SetMCU1toMCU2InstigatedCommsCount()
*
* @brief	Write mcu1 to mcu2 instigated comms count in the EEPROM
*
* @param	MCU1ToMCU2InstigatedCommsCount	mcu1 to mcu2 instigated comms count
********************************************************************************************************/
void DataLogging_SetMCU1toMCU2InstigatedCommsCount(uint32_t MCU1ToMCU2InstigatedCommsCount) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU1ToMCU2InstigatedCommsCount)));
	CommonUtils_Uint32ToUint8(MCU1ToMCU2InstigatedCommsCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                   DataLogging_GetMCU1toMCU2InstigatedCommsCount()
*
* @brief	Read mcu1 to mcu2 instigated comms count from the EEPROM
*
* @return	mcu1 to mcu2 instigated comms count
********************************************************************************************************/
uint32_t DataLogging_GetMCU1toMCU2InstigatedCommsCount(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU1ToMCU2InstigatedCommsCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                   DataLogging_SetMCU2toMCU1InstigatedCommsCount()
*
* @brief	Write mcu2 to mcu1 instigated comms count in the EEPROM
*
* @param	data	mcu2 to mcu1 instigated comms count
********************************************************************************************************/
void DataLogging_SetMCU2toMCU1InstigatedCommsCount(uint32_t MCU2ToMCU1InstigatedCommsCound) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU2ToMCU1InstigatedCommsCound)));
	CommonUtils_Uint32ToUint8(MCU2ToMCU1InstigatedCommsCound, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                   DataLogging_GetMCU2toMCU1InstigatedCommsCount()
*
* @brief	Read mcu2 to mcu1 instigated comms count from the EEPROM
*
* @return	mcu2 to mcu1 instigated comms count
********************************************************************************************************/
uint32_t DataLogging_GetMCU2toMCU1InstigatedCommsCount(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MCU2ToMCU1InstigatedCommsCound)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                      DataLogging_SetAssistLightTotalDuration()
*
* @brief	Write assistance light total duration in the EEPROM
*
* @param	AssistanceLightTotalDuration	assistance light total duration
********************************************************************************************************/
void DataLogging_SetAssistLightTotalDuration(uint32_t AssistanceLightTotalDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AssistanceLightTotalDuration)));
	CommonUtils_Uint32ToUint8(AssistanceLightTotalDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                      DataLogging_GetAssistLightTotalDuration()
*
* @brief	Read assistance light total duration from the EEPROM
*
* @return	assistance light total duration
********************************************************************************************************/
uint32_t DataLogging_GetAssistLightTotalDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AssistanceLightTotalDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                       DataLogging_SetAssistLightActivCount()
*
* @brief	Write assistance light activation count in the EEPROM
*
* @param	AssistanceLightActivationCount	assistance light activation count
********************************************************************************************************/
void DataLogging_SetAssistLightActivCount(uint16_t AssistanceLightActivationCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AssistanceLightActivationCount)));
	CommonUtils_Uint16ToUint8(AssistanceLightActivationCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                       DataLogging_GetAssistLightActivCount()
*
* @brief	Read assistance light activation count from the EEPROM
*
* @return	assistance light activation count
********************************************************************************************************/
uint16_t DataLogging_GetAssistLightActivCount(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AssistanceLightActivationCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/****************************************************************************************************//**
*                                         DataLogging_SetAirRecActivCount()
*
* @brief	Write airing recommendation activation count in the EEPROM
*
* @param	AiringRecommendActivationCount	airing recommendation activation count
********************************************************************************************************/
void DataLogging_SetAirRecActivCount(uint32_t AiringRecommendActivationCount) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringRecommendActivationCount)));
	CommonUtils_Uint32ToUint8(AiringRecommendActivationCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetAirRecActivCount()
*
* @brief	Read airing recommendation activation count from the EEPROM
*
* @return	airing recommendation activation count
********************************************************************************************************/
uint32_t DataLogging_GetAirRecActivCount(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringRecommendActivationCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetAirRecTotalDuration()
*
* @brief	Write airing recommendation total duration in the EEPROM
*
* @param	AiringRecommendTotalDuration	airing recommendation total duration
********************************************************************************************************/
void DataLogging_SetAirRecTotalDuration(uint32_t AiringRecommendTotalDuration) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringRecommendTotalDuration)));
	CommonUtils_Uint32ToUint8(AiringRecommendTotalDuration, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                         DataLogging_GetAirRecTotalDuration()
*
* @brief	Read airing recommendation total duration from the EEPROM
*
* @return	airing recommendation total duration
********************************************************************************************************/
uint32_t DataLogging_GetAirRecTotalDuration(void) {
	uint8_t buff[4] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringRecommendTotalDuration)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint32(buff);
}

/****************************************************************************************************//**
*                                        DataLogging_SetAirConfigChangeCount()
*
* @brief	Write airing configuration change count in the EEPROM
*
* @param	AiringConfigChangeCount	airing configuration change count
********************************************************************************************************/
void DataLogging_SetAirConfigChangeCount(uint16_t AiringConfigChangeCount) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringConfigChangeCount)));
	CommonUtils_Uint16ToUint8(AiringConfigChangeCount, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff));
}

/****************************************************************************************************//**
*                                        DataLogging_GetAirConfigChangeCount()
*
* @brief	Read airing configuration change count from the EEPROM
*
* @return	airing configuration change count
********************************************************************************************************/
uint16_t DataLogging_GetAirConfigChangeCount(void) {
	uint8_t buff[2] = {0};
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, AiringConfigChangeCount)));
	DataLogging_ReadData(buff, addr, sizeof(buff));
	return CommonUtils_Uint8ToUint16(buff);
}

/*******************************************************************************
 * @brief   Get next demounting event id
 *
 * @details This function reads each record in the log to find the entry with
 *          the highest event id and then calculates the next free id
 *
 * @return Next free event id number
 */
static uint16_t get_next_event_id( const uint8_t record_len, const uint8_t records_max, const uint16_t offset  )
{
  /* Log by default is empty */
  bool empty = true;

  /* Maximum event id number */
  uint16_t event_id_max = 0u;

  /* Log book entry buffer */
  uint8_t buf[ sizeof( dl_event_logbook_record_t ) ] = { 0 };

  /* Read each log book entry entry */
	for( int i = 0; i < records_max; i++ )
  {
    /* Caclulate address in EEPROM based on the index number */
  	const uint16_t addr = offset + ( ( uint16_t )( record_len ) * i );

    /* Read next record */
		DataLogging_ReadData( buf, addr, record_len );

    /* Event id occupies first 2 bytes of each record */
  	const uint16_t event_id = CommonUtils_Uint8ToUint16( buf );

    /* Is this maximum/full/erased value? */
    if( event_id != LOGBOOK_EVENT_ID_MAX )
    {
      /* No, log not empty */
      empty = false;

      /* Is this event id the highest? */
      if( event_id > event_id_max )
      {
        /* Yes, lets save it */
        event_id_max = event_id;
      }
    }
  }

  /* Was the log empty? */
  if( empty == false )
  {
    /* No, get next event id */
    event_id_max++;
  }

  /* Next free event id */
  return( event_id_max );
}

/*******************************************************************************
 * @brief   Initialise demounting logbook
 *
 * @details This function initialises the data logging demounting loogbook
 *          functionality
 * 
 * @see get_next_event_id
 */
void data_logging_logbook_init( void )
{
  RTOS_ERR err;

  OSMutexCreate( &logbook_mutex, "logbook_mutex", &err );
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

  /* Get current maximum event id value */
  demounting_logbook_event_id = get_next_event_id( sizeof( dl_demounting_logbook_record_t ),
                                                   DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX,
                                                   DATALOGGING_DEMOUNTING_LOGBOOK_ADDR_OFFSET );

  /* Get current maximum event id value */
  event_logbook_event_id = get_next_event_id( sizeof( dl_event_logbook_record_t ),
                                              DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX,
                                              DATALOGGING_EVENT_LOGBOOK_ADDR_OFFSET );

  //DEBUG_PRINTF( "Next Demounting Loogbook Event id: %d\n\r", demounting_logbook_event_id );
  //DEBUG_PRINTF( "Next Event      Loogbook Event id: %d\n\r", event_logbook_event_id );

	/* Reset logbook event status flags */
	for( int i = 0, j = sizeof( logbook_flags ); i < j; i++ )
	{
		logbook_flags[ i ] = false;
	}
}

uint8_t DataLogging_SetLogbookRecord( const uint16_t event_id, const uint8_t event_type, uint8_t index, const uint8_t index_max, const uint16_t address, uint8_t * data )
{
  /* Create buffer to store entry */
  uint8_t buf[ sizeof( dl_event_logbook_record_t ) ] = { 0 };

	/* Buffer size */
	size_t buf_len = sizeof( buf );

  /* Get current time in seconds for the timestamp */
  const uint32_t current_time = get_currentTime( );

  /* Mode is saved in each entry */
  const behaviour_state_enum_System_modes mode = getBehavioural_System_Modes( false );

  /* Save event id */
  CommonUtils_Uint16ToUint8( event_id, buf );

  /* Save the timestamp */
  CommonUtils_Uint32ToUint8( current_time, &buf[ 2 ] );

  /* Save the operatrional mode */
  buf[ 6 ] = ( uint8_t )mode;

  /* Finally the event type, mounted or demounted */
  buf[ 7 ] = event_type;

	/* Is there and data? */
	if( data )
	{
		( void )memcpy( &buf[ 8 ], data, DEF_LEN_EVENT_LOGBOOK_DATA );
	}
	else
	{
		buf_len -= DEF_LEN_EVENT_LOGBOOK_DATA;
	}

  /* Save logbook entry to EEPROM */
  DataLogging_WriteData( buf, address, buf_len );

  /* Index just wraps */
  index = ( index + 1 ) % index_max;

	/* Caller need the index */
	return( index );
}

/*******************************************************************************************************
* @brief	Write demounting logbook record in the EEPROM
*
* @details This function for the given event type, writes a logbook recoed to the EEPROM. The index
*          position of the next free location is read from the EEPROM. The event id is maintained
*          internally and incremented as required.
*
* @param[in]	event_type	event type
*
********************************************************************************************************/
void DataLogging_SetDemountingLogbookRecord( const uint8_t event_type )
{
	lock_logbook( );

  /* Have we run out of ebents? */
  if( demounting_logbook_event_id < LOGBOOK_EVENT_ID_MAX )
  {
    /* Get index location to save event */
    uint8_t index = DataLogging_GetDemountingIndex( );

    /* Is this valid? */
    if( index > DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX )
    {
      /* No, go back to the start */
      index = 0u;
    }

    /* Using the current index number, calculate address of the record in the EEPROM */
    const uint16_t address = DATALOGGING_DEMOUNTING_LOGBOOK_ADDR_OFFSET +
                            ( ( uint16_t )( sizeof( dl_demounting_logbook_record_t ) ) * index );

		/* Save record */
		index = DataLogging_SetLogbookRecord( demounting_logbook_event_id, event_type, index, DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX, address, NULL );

    /* Save index of next free item */
    DataLogging_SetDemountingIndex( index );

    /* Event id of next entry */
    demounting_logbook_event_id++;
  }

	unlock_logbook( );
}

/****************************************************************************************************//**
*                                       DataLogging_GetDemountingLogbookRecord()
*
* @brief	Read demounting logbook record from the EEPROM
*
* @param	index	address of EEPROM location to read
* @param	data	pointer to demounting logbook record buffer
********************************************************************************************************/
void DataLogging_GetDemountingLogbookRecord( uint16_t index, dl_demounting_logbook_record_t * record )
{
	uint8_t buf[ sizeof( dl_demounting_logbook_record_t ) ] = { 0 };

	if( index < DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX )
  {
		const uint16_t address = DATALOGGING_DEMOUNTING_LOGBOOK_ADDR_OFFSET + ( ( uint16_t )( sizeof( dl_demounting_logbook_record_t ) ) * index );

		DataLogging_ReadData( buf, address, sizeof( buf ) );				/* Read record from EEPROM	*/
	}

	record->Count 		      = CommonUtils_Uint8ToUint16( buf );									/* Copy data into structure */
	record->Timestamp 	    = CommonUtils_Uint8ToUint32( &buf[ 2 ] );
	record->OperatingMode   = buf[ 6 ];
	record->ID 			        = buf[ 7 ];
}

/*******************************************************************************************************
* @brief		Send logbook to MCU2
*
* @details 	This function, depending on the given event type, sends the logbook to MCU2.
*
* @param[in]	event_type	event type
*
* @return		true if sent
*
********************************************************************************************************/
static bool send_logbook_to_mcu2( const uint8_t event_type )
{
	bool ok;

	switch( event_type )
	{
		/* CO */
		case DEF_LBE_CO_DET_START:
		case DEF_LBE_CO_DET_END:
		case DEF_LBE_CO_DET_HW_ERR_START:
		case DEF_LBE_CO_SENSOR_EOL_START:
		case DEF_LBE_SUPER_CO_START:
		case DEF_LBE_SUPER_CO_END:

		/* Smoke */
		case DEF_LBE_SMOKE_DET_START:
		case DEF_LBE_SMOKE_DET_END:
		case DEF_LBE_SMOKE_CHAM_CONT_START:
		case DEF_LBE_SMOKE_CHAM_CONT_END:
		case DEF_LBE_SMOKE_CHAM_HW_ERR_START:
		case DEF_LBE_SMOKE_CHAM_HW_ERR_END:
		case DEF_LBE_SUPER_SMOKE_START:
		case DEF_LBE_SUPER_SMOKE_END:

		/* Heat */
		case DEF_LBE_HEAT_DET_START:
		case DEF_LBE_HEAT_DET_END:
		case DEF_LBE_HEAT_DET_HW_ERR_START:
		case DEF_LBE_HEAT_DET_HW_ERR_END:
		case DEF_LBE_SUPER_HEAT_START:
		case DEF_LBE_SUPER_HEAT_END:

		/* */
		case DEF_LBE_ALARM_MUTED_START:
		case DEF_LBE_ALARM_MUTED_END:

		/* Fault */
		case DEF_LBE_FAULT_MUTED_START:
		case DEF_LBE_FAULT_MUTED_END:

		/* Battery */
		case DEF_LBE_BATTERY_ERR_START:
		case DEF_LBE_BATTERY_SHUTDOWN_START:

		/* Remote */
		case DEF_LBE_REMOTE_ALARM_TEST:
		case DEF_LBE_REMOTE_ALARM_SILENCE:
		case DEF_LBE_REMOTE_ALARM_RECEIVED:
		case DEF_LBE_SMOKE_REMOTE_ALARM:
		case DEF_LBE_HEAT_REMOTE_ALARM:
		case DEF_LBE_CO_REMOTE_ALARM:

		/* User BIST */
		case DEF_LBE_USER_BIST:
		case DEF_LBE_BUZZER_CHECK_HW_ERR_START:
		case DEF_LBE_BUZZER_CHECK_HW_ERR_END:

		/* Demounting */
		case DEF_LBE_DEMOUNTED_START:
		case DEF_LBE_DEMOUNTED_TOO_LONG_START:
		case DEF_LBE_DEMOUNTED_END:
		case DEF_LBE_DEMOUNTED_DET_HW_ERR_START:
		case DEF_LBE_DEMOUNTED_DET_HW_ERR_END:

		/* Obstacle */
		case DEF_LBE_OBSTACLE_DET_START:
		case DEF_LBE_OBSTACLE_DET_END:
		case DEF_LBE_OBSTACLE_DET_HW_ERR_START:
		case DEF_LBE_OBSTACLE_DET_HW_ERR_END:
		case DEF_LBE_COVERAGE_DET_START:
		case DEF_LBE_COVERAGE_DET_END:

		/* Temp Humid */
		case DEF_LBE_TEMP_OOR_START:
		case DEF_LBE_TEMP_OOR_END:
		case DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_START:
		case DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_END:
		case DEF_LBE_HUMID_OOR_START:
		case DEF_LBE_HUMID_OOR_END:
		case DEF_LBE_HUMID_SENSOR_HW_ERR_START:
		case DEF_LBE_HUMID_SENSOR_HW_ERR_END:

		/* Ambient 7 days*/
		case DEF_LBE_AMB_LIGHT_7_DAYS_DARK:

		/*reset*/
		case DEF_LBE_RESET:
		{
			ok = true;
			break;
		}
		default:
		{
			ok = false;
			break;
		}
	}

	if( ok )
	{
		SPIComms_Send_Data_to_MCU2( SPI_CMD_Logbook_value );
	}

	return( ok );
}

/****************************************************************************************************//**
*                                         DataLogging_SetEventLogbookRecord()
*
* @brief	Write event logbook record in the EEPROM
*
* @param	event_id	unique id of the event
* @param	data		pointer to data (Null if no event data)
********************************************************************************************************/
void DataLogging_SetEventLogbookRecord( const uint8_t event_type, uint8_t * data )
{
	uint8_t buf[ DEF_LEN_EVENT_LOGBOOK_DATA ] = { 0 };

	lock_logbook( );

  /* Have we run out of events? */
  if( event_logbook_event_id < LOGBOOK_EVENT_ID_MAX )
  {
    /* Get index location to save event */
    uint8_t index = DataLogging_GetMainLogbookIndex( );

    /* Is this valid? */
    if( index > DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX )
    {
      /* No, go back to the start */
      index = 0u;
    }

		/* If required, copy data */
		if( data )
		{
			memcpy( buf, data, DEF_LEN_EVENT_LOGBOOK_DATA );
		}

    /* Using the current index number, calculate address of the record in the EEPROM */
    const uint16_t address = DATALOGGING_EVENT_LOGBOOK_ADDR_OFFSET +
                            ( ( uint16_t )( sizeof( dl_event_logbook_record_t ) ) * index );

		/* Save record */
		index = DataLogging_SetLogbookRecord( event_logbook_event_id, event_type, index, DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX, address, buf );

    /* Save index of next free item */
    DataLogging_SetMainLogbookIndex( index );

    /* Event id of next entry */
    event_logbook_event_id++;

		/* Send logbook to MCU2 */
    if(DataLogging_GetLogMsgToMCU() == true)
    {
        send_logbook_to_mcu2( event_type );
    }
  }

	unlock_logbook( );
}

void DataLogging_SetEventLogbookRecordStartEnd( const uint8_t event_type, uint8_t * data, const bool start )
{
	lock_logbook( );

	if( start && !logbook_flags[ event_type ] )
	{
		/* New start event */
		DataLogging_SetEventLogbookRecord( event_type, data );

		logbook_flags[ event_type ] = true;
	}
	else if( !start && logbook_flags[ event_type ] )
	{
		/* New end event */
		/* !!! NOTE: End event must be a +1 from the start !!! */
		DataLogging_SetEventLogbookRecord( event_type + 1, data );

		logbook_flags[ event_type ] = false;
	}
	else
	{
		/* Nothing, must be a duplicate! */
	}

	unlock_logbook( );
}

/****************************************************************************************************//**
*                                         DataLogging_GetEventLogbookRecord()
*
* @brief	Read event logbook record from the EEPROM
*
* @param	address	address of EEPROM location to read
* @return	logbook record
********************************************************************************************************/
void DataLogging_GetEventLogbookRecord( const uint16_t index, dl_event_logbook_record_t * record )
{
	uint8_t buf[ sizeof( dl_event_logbook_record_t ) ] = { 0 };

	if( index < DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX )
  {
		const uint16_t address = DATALOGGING_EVENT_LOGBOOK_ADDR_OFFSET + ( ( uint16_t )( sizeof( dl_event_logbook_record_t ) ) * index );

		DataLogging_ReadData( buf, address, sizeof( buf ) );				/* Read record from EEPROM	*/
	}

	record->Count 		      = CommonUtils_Uint8ToUint16( buf );									/* Copy data into structure */
	record->Timestamp 	    = CommonUtils_Uint8ToUint32( &buf[ 2 ] );
	record->OperatingMode   = buf[ 6 ];
	record->ID 			        = buf[ 7 ];

	( void )memcpy( ( uint8_t * )&record->Data[0], &buf[ 8 ], DEF_LEN_EVENT_LOGBOOK_DATA );
}

/****************************************************************************************************//**
*                                         DataLogging_GetResetReasonCtrAddr()
*
* @brief	Get reset reason counter Address
*
* @param	RstReason	Reset reason type
* @return	reset reason counter address
********************************************************************************************************/
static uint16_t DataLogging_GetResetReasonCtrAddr(dl_reset_reason_t RstReason)
{
	uint16_t addr = DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, ResetCounter));
	switch (RstReason)
	{
	case RstPOR:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstPOR));
		break;
	}

	case RstPIN:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstPIN));
		break;
	}

	case RstEM4:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstEM4));
		break;
	}

	case RstWDOG0:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstWDOG0));
		break;
	}

	case RstWDOG1:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstWDOG1));
		break;
	}

	case RstLOCKUP:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstLOCKUP));
		break;
	}

	case RstSYSREQ:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstSYSREQ));
		break;
	}

	case RstDVDDBOD:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstDVDDBOD));
		break;
	}

	case RstDVDDLEBOD:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstDVDDLEBOD));
		break;
	}

	case RstDECBOD:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstDECBOD));
		break;
	}

	case RstAVDDBOD:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstAVDDBOD));
		break;
	}

	case RstIOVDDBOD:
	{
		addr += (uint16_t)(offsetof(dl_reset_counters_t, RstIOVDDBOD));
		break;
	}
	
	default:
	{
		break;
	}
	}
	return addr;
}

/****************************************************************************************************//**
*                                         DataLogging_SetResetReason()
*
* @brief	Increment Reset Reason counter
*
* @param	RstReason	reset Reason
* @return	None
********************************************************************************************************/
void DataLogging_SetResetReason(dl_reset_reason_t RstReason)
{
	uint8_t buff[2u] = {0};
	uint16_t count;
	uint16_t addr = DataLogging_GetResetReasonCtrAddr(RstReason);
	/*Read current counter*/
	DataLogging_ReadData(buff, addr, 2u);
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFu)
	{
	   count = 0u;
	}
	count++; /*Increment reset reason counter*/
	CommonUtils_Uint16ToUint8(count, buff);
	DataLogging_WriteData(buff, addr, sizeof(buff)); /*Write back*/
	Set_Reset_Cause_MCU(true);
}

/****************************************************************************************************//**
*                                         DataLogging_GetResetReasonCount()
*
* @brief	Get Reset Reason counter
*
* @param	RstReason	reset Reason
* @return	reset reason counter
********************************************************************************************************/
uint16_t DataLogging_GetResetReasonCount(dl_reset_reason_t RstReason)
{
	uint8_t buff[2u] = {0};
	uint16_t count;
	uint16_t addr = DataLogging_GetResetReasonCtrAddr(RstReason);
	/*Read current counter*/
	DataLogging_ReadData(buff, addr, 2u);
	count = CommonUtils_Uint8ToUint16(buff);
	if(count == 0xFFFFu)
	{
	   count = 0u;
	}
	return count;
}

/****************************************************************************************************//**
*                                         DataLogging_GetMinorFaultCounterAddr()
*
* @brief	Get minor fault counter Address
*
* @param	MinorFault	minor fault type
* @return	minor fault counter address
********************************************************************************************************/
static uint16_t DataLogging_GetMinorFaultCounterAddr(dl_minor_fault_t MinorFault)
{
	uint16_t addr = (uint16_t)(DATALOGGING_STATIC_LOCATION_ADDR_OFFSET + (uint16_t)(offsetof(dl_static_location_data_t, MinorFaultCounter)));
	switch (MinorFault)
	{
      case FaultDegradedSmokeChamber:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultDegradedSmokeChamber));
        break;
      }

      case FaultObstacleDetected:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultObstacleDetected));
        break;
      }

      case FaultSoilDetected:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultSoilDetected));
        break;
      }

      case FaultCoverageDetected:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultCoverageDetected));
        break;
      }

      case FaultTemperatureOutOfBound:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultTemperatureOutOfBound));
        break;
      }

      case FaultHumidityOutOfBound:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultHumidityOutOfBound));
        break;
      }

      case FaultTestButton:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultTestButton));
        break;
      }

      case FaultDemountedTooLong:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultDemountedTooLong));
        break;
      }

      case FaultCalibrationDataCorrupt:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultCalibrationDataCorrupt));
        break;
      }

      case FaultObstacleBISTOverdue:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultObstacleBISTOverdue));
        break;
      }

      case FaultObstacleDetectionOverdue:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultObstacleDetectionOverdue));
        break;
      }

      case FaultBuzzerCheckOverdue:
      {
        addr += (uint16_t)(offsetof(dl_minor_fault_counters_t, FaultBuzzerCheckOverdue));
        break;
      }

      default:
      {
        break;
      }
	}
	return addr;
}

/****************************************************************************************************//**
*                                         DataLogging_SetMinorFault()
*
* @brief	Increment Minor Fault counter
*
* @param	MinorFault	Minor Fault type
* @return	None
********************************************************************************************************/
void DataLogging_SetMinorFault(dl_minor_fault_t MinorFault, const bool status)
{
	uint8_t buff[2u] = {0};
	uint16_t count;

	if(status == true)
	{
	    uint16_t addr = DataLogging_GetMinorFaultCounterAddr(MinorFault);
	      /*Read current counter*/
	    DataLogging_ReadData(buff, addr, 2u);
	    count = CommonUtils_Uint8ToUint16(buff);

	    if(count == 0xFFFFu)
	    {
	       count = 0xFFFEu;
	    }

	    count++; /*Increment reset reason counter*/

	    CommonUtils_Uint16ToUint8(count, buff);
	    DataLogging_WriteData(buff, addr, sizeof(buff)); /*Write back*/
  }

}

/****************************************************************************************************//**
*                                         DataLogging_GetMinorFaultCount()
*
* @brief	Get Minor Fault counter
*
* @param	RstReason	reset Reason
* @return	minor fault counter
********************************************************************************************************/
uint16_t DataLogging_GetMinorFaultCount(dl_minor_fault_t MinorFault)
{
	uint8_t buff[2u] = {0};
	uint16_t count;
	uint16_t addr = DataLogging_GetMinorFaultCounterAddr(MinorFault);
	/*Read current counter*/
	DataLogging_ReadData(buff, addr, 2u);
	count = CommonUtils_Uint8ToUint16(buff);
	return count;
}

/****************************************************************************************************//**
*                                         dl_get_production_complete()
*
* @brief  Get the production complete flag
*
* @param  n/a
* @return return production complete flag BB or AA
********************************************************************************************************/
uint32_t dl_get_production_complete( void )
{
	uint8_t buf[sizeof(uint32_t)] = {0};                                          /* Buffer with value in correct byte order */

	const uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET         /* Offset in EEPROM where value is located */
	                    + (uint16_t)(offsetof(dl_cfg_calib_data_type, production_complete)));

	DataLogging_ReadData(buf, addr, sizeof(buf));                                 /* Read value from the EEPROM */
	const uint32_t magic_number = CommonUtils_Uint8ToUint32(buf);                 /* Convert to real value*/
  return(magic_number);                                                         /* and return to caller */
}

/****************************************************************************************************//**
*                                         dl_set_production_complete()
*
* @brief  Set the production complete flag
*
* @param  production flag value
* @return true/false for success/failure
********************************************************************************************************/
bool dl_set_production_complete( const uint32_t magic_number )
{
  bool retVal = false;
	uint8_t buf[sizeof(uint32_t)] = {0};                                          /* Buffer with value in correct byte order */

	const uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET         /* Offset in EEPROM where value is located */
	                    + (uint16_t)(offsetof(dl_cfg_calib_data_type, production_complete)));

	CommonUtils_Uint32ToUint8(magic_number, buf);                                 /* Convert value and place in buffer */
	retVal = DataLogging_WriteData(buf, addr, sizeof(buf));                       /* Write value to the EEPROM */
	return retVal;
}

/****************************************************************************************************//**
*                                         dl_get_production_timer()
*
* @brief  Get the production Timer value
*
* @param  n/a
* @return return production timer values in hrs
********************************************************************************************************/
uint32_t dl_get_production_timer(void)
{
    uint8_t buf[sizeof(uint32_t)] = {0};                                        /* Buffer with value in correct byte order */

    const uint16_t addr = ( uint16_t )(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET     /* Offset in EEPROM where value is located */
                        + ( uint16_t )(offsetof( dl_cfg_calib_data_type, production_timer)));

    DataLogging_ReadData(buf, addr, sizeof(buf));                               /* Read value from the EEPROM */
    const uint32_t magic_number = CommonUtils_Uint8ToUint32(buf);               /* Convert to real value*/
    return(magic_number);                                                       /* and return to caller */
}

/****************************************************************************************************//**
*                                         dl_set_production_timer()
*
* @brief  Set the production timer value in hrs
*
* @param  value in hrs
* @return true/false for success/failure
********************************************************************************************************/
bool dl_set_production_timer(const uint32_t magic_number)
{
  bool retVal = false;
  uint8_t buf[sizeof(uint32_t)] = {0};                                          /* Buffer with value in correct byte order */

  const uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET         /* Offset in EEPROM where value is located */
                      + (uint16_t)(offsetof(dl_cfg_calib_data_type, production_timer)));

  CommonUtils_Uint32ToUint8(magic_number, buf);                                 /* Convert value and place in buffer */
  retVal = DataLogging_WriteData(buf, addr, sizeof(buf));                       /* Write value to the EEPROM */
  return retVal;
}

/****************************************************************************************************//**
*                                         dl_get_prod_commence_time()
*
* @brief  Get the production commence time
*
* @param  n/a
* @return return production commence time
********************************************************************************************************/
uint32_t dl_get_prod_commence_time(void)
{
    uint8_t buf[sizeof(uint32_t)] = {0};                                        /* Buffer with value in correct byte order */

    const uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET        /* Offset in EEPROM where value is located */
                        + (uint16_t)(offsetof( dl_cfg_calib_data_type, prod_commence_time)));

    DataLogging_ReadData(buf, addr, sizeof(buf));                               /* Read value from the EEPROM */
    const uint32_t magic_number = CommonUtils_Uint8ToUint32( buf );             /* Convert to real value*/
    return(magic_number);                                                       /* and return to caller */
}

/****************************************************************************************************//**
*                                        dl_set_prod_commence_time()
*
* @brief  Set the production commence time
*
* @param  unix time stamp
* @return true/false for success/failure
********************************************************************************************************/
bool dl_set_prod_commence_time( const uint32_t magic_number )
{
  bool retVal = false;
  uint8_t buf[sizeof(uint32_t)] = {0};                                          /* Buffer with value in correct byte order */

  const uint16_t addr = (uint16_t)(DATALOGGING_CALIB_CONFIG_ADDR_OFFSET         /* Offset in EEPROM where value is located */
                      + (uint16_t)(offsetof(dl_cfg_calib_data_type, prod_commence_time)));

  CommonUtils_Uint32ToUint8(magic_number, buf);                                 /* Convert value and place in buffer */
  retVal = DataLogging_WriteData(buf, addr, sizeof( buf));                      /* Write value to the EEPROM */
  return retVal;
}



/****************************************************************************************************//**
*                                         DataLoggingErase()
*
* @brief	Depends on start address either erases full eeprom or production data
*
* @return	trie if successful, otherwise false
********************************************************************************************************/
bool DataLoggingErase(uint16_t startAddress)
{
	I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* Acquire I2C Bus */
	const bool ok = EEPROM_Blank(startAddress);
	I2C_BusRelease(); /* Release I2C Bus */

	return( ok );
}

/****************************************************************************************************//**
*                                         		DataLogging_WriteData()
*
* @brief	Write data into the EEPROM
*
* @param	pvdata	pointer to data buffer to write
* @param	address	address of EEPROM location to write
* @param	len		length of data to write
********************************************************************************************************/
static bool DataLogging_WriteData(const void *pvdata, uint16_t address, uint16_t len)
{
  bool retVal = false;
	const uint8_t *pu8data = (const uint8_t *)(pvdata);
	uint16_t idx = 0;
	I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* enable the power to eeprom */

	while (len)
	{
		/*Compute Remaining bytes in current page*/
		uint16_t rem = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
		uint16_t bytes_to_write;
		if (len <= rem)
		{
			bytes_to_write = len;
		}
		else
		{
			bytes_to_write = rem;
		}
		retVal = EEPROM_WriteandVerify(&pu8data[idx], address, bytes_to_write);
		len -= bytes_to_write;
		idx += bytes_to_write;
		address += bytes_to_write;
	}
	I2C_BusRelease(); /* Release I2C Bus */
	return retVal;
}

/****************************************************************************************************//**
*                                         		DataLogging_ReadData()
*
* @brief	Read data from the EEPROM
*
* @param	data	pointer to data buffer
* @param	address	address of EEPROM location to write
* @param	len		length of data to read
********************************************************************************************************/
static void DataLogging_ReadData(void* pvdata, uint16_t address, uint16_t len)
{
	uint8_t *pu8data = (uint8_t *)(pvdata);
	I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* Acquire I2C Bus */
	EEPROM_Read(pu8data, address, len);
	I2C_BusRelease(); /* Release I2C Bus */
}


/****************************************************************************************************//**
*                                             Reset_EEROM_Production()
*
* @brief  Resets EEPROM after production lockout period
*
* @param  n/a
* @return n/a
********************************************************************************************************/
void Reset_EEPROM_Production(void)
{
     uint8_t buff[2u] = {0};
      /* Get eeprom integrity? */
     const bool eeprom_ok = data_logging_is_eeprom_ok( );
     /* Can eeprom be trusted? */
     if( eeprom_ok )
     {
       DEBUG_APP("\nProduction Erase ", false, 0u);
       const uint8_t dev_config = DataLogging_GetDeviceConfig( );

       if( dev_config == DEVICE_CONFIG_SMOKE )
       {
           setIsDeviceSmokeEnable(true);
       }
       else
       {
           setIsDeviceSmokeEnable(false);
       }
       const uint32_t firstActivation = DataLogging_GetFirstActivation();
       const uint32_t timeStamp = DataLogging_GetLatestTimestamp();
       const uint32_t opState = DataLogging_GetOperatingState();
       DataLoggingErase(0x100u);

       DataLogging_SetFWBuild();

       DataLogging_SetFirstActivation(firstActivation);
       DataLogging_SetLatestTimestamp(timeStamp);
       DataLogging_SetOperatingState(opState);

       /* initialize minor fault counter to 0x0000*/
       for( uint8_t i = 0; i < MINOR_FAULT_COUNTER; i++)
       {
           uint16_t addr = DataLogging_GetMinorFaultCounterAddr(i);
           DataLogging_WriteData(buff, addr, sizeof(buff)); /*Write back*/
       }

       DataLogging_ResetAllIndexes();
     }
}

void Set_Reset_Cause_MCU(bool status)
{
  resetCauseMCU  = status;
}

bool Get_Reset_Cause_MCU(void)
{
   return resetCauseMCU;
}


/****************************************************************************************************//**
*                                             DataLogging_SetLogMsgToMCU()
*
* @brief  This module sets whether log messages send to MCU2
*
* @param  bool status.  True = send, false = not to send
* @return n/a
********************************************************************************************************/
void DataLogging_SetLogMsgToMCU(bool status)
{
  logMsgtoMCU = status;
}
/****************************************************************************************************//**
*                                             Reset_EEROM_Production()
*
* @brief  this module returns the satus to send log messages to MCU2
*
* @param  n/a
* @return true = send,  false = not to send
********************************************************************************************************/
bool DataLogging_GetLogMsgToMCU(void)
{
  return logMsgtoMCU;
}
