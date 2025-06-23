#ifndef P0200_FTM_H
#define P0200_FTM_H

#include "debug.h"
#include "stdint.h"
#include "nextgen_protocol.h"
#include "os.h"
#include "app.h"
#include "events.h"
#include "system_events.h"
#include "hal_BURTCTimer.h"
#include "timeHandler.h"
#include "string.h"
#include "hal_AFE.h"
#include "led_buzzer.h"
#include "fault_handler.h"
#include "telegram.h"
#include "spi_comms.h"
#include "data_logging.h"
#include "temp_humid.h"
#include "hal_LETimer.h"
#include "hal_switches.h"
#include "acquisition_Co.h"
#include "switches_handler.h"
#include "acquisition_Smoke.h"
#include "telegram.h"
#include "assistance_light.h"
#include "diagnostics.h"

#ifdef DEBUG_BUILD
#define P0200_FTM_COMMAND_COUNT 95U
#else
#define P0200_FTM_COMMAND_COUNT 98U
#endif

extern const nextGenCommsCommand_t P0200_FTMCommandTable[P0200_FTM_COMMAND_COUNT];
#define P0200_FTM_COMMAND_TABLE_SIZE ((size_t )(sizeof(P0200_FTMCommandTable) / sizeof(P0200_FTMCommandTable[0u])))
#define SH_TEST_DATA_BYTES 19U
#define CO_TEST_DATA_BYTES 17U

extern nextGenCommsDriverInterface_t ngCommsDriver;
void exitFTMProtocol(void);
void setXModemEnable(bool enable);
bool getXModemEnable(void);
void setIncresedTimerRate(bool enable);
bool getIncresedTimerRate();
void resetProtocolTimoutTimer(void);
bool FTM_Is_NG_Protocol_Active();
void setFaultFlagStatus(uint32_t fault_val);
uint32_t getFaultFlagStatus(void);
void faultFlagsRestore(void);
uint8_t get_prod_comp_timer(void);
behaviour_state_enum_System_modes get_previous_mode(void);
void restore_uart_tx_pin_mode(void);
void setDefaultUART(bool enable);
bool getDefaultUART(void);

nextGenCommsAckNackReason_t FTM_Mode_Enter(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Mode_Exit(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_EnterBootloader(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_ReadSerialNumber(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_ReadFirmwareNum(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_ReadFirmwareVer(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Reset_Timeout(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
#ifdef DEBUG_BUILD
nextGenCommsAckNackReason_t CLI_Mode_Enter(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
#endif
nextGenCommsAckNackReason_t FTM_Read_OpMode(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Comp_Prod(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Previous_OpMode(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);


nextGenCommsAckNackReason_t FTM_Timers_SetSystemTime(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Timers_ResetRate(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Timers_IncreaseRate(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Timers_GetSystemTime(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Demount_ChangeStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Demount_SimulateState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_CO_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_SetMeasurementPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_SetMuteStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_SimulatedCOLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_RunSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_ReadSensitivityTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_CO_StopSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Smoke_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_SetMeasurementPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_SetChamberBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_SimulatedLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_SetMuteStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_StartSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_ReadSensitivityTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Smoke_StopSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Heat_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_SetDetPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_SetMuteState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_SimulateLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_StartSensiTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_ReadSensiTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Heat_StopSensiTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Battery_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_SimVolLvl(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_SimImpLvl(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_MeasureVol(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Battery_MeasureImp(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Buzzer_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Buzzer_StartTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Buzzer_GetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Buzzer_RunBist(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Buzzer_SoundOutputTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Humid_SetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Humid_SimulateLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Temp_SetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Temp_SimulateTempVal(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_LED_GetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_LED_StartTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Obs_SelectSensor(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_SimObDis(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_SetObDetPeriod(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_SetObDetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_SetCoDetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_GetObsDetResults(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_StopTests(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_SensorResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_RunBist(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Obs_BistResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_AssistanceLight_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Reset_Soft(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Reset_I2C(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Read_DBG_Reg(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Write_DBG_Reg(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_EEPROM_StopTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_EEPROM_SimulateCorruption(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_EEPROM_DownloadData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_EEPROM_ClearEEPROM(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_EEPROM_ReadRow(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_AmbientLight_SetStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Switch_SetReadPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Switch_GetStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Switch_GetPeriodicityResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Switch_StopTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);


nextGenCommsAckNackReason_t FTM_SystemConfigFlags(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_DeviceConfigFlags(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Radio_Test(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Uart_FactoryTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

nextGenCommsAckNackReason_t FTM_Fault_Read(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Fault_Clear_All(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);
nextGenCommsAckNackReason_t FTM_Fault_Restore(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

typedef enum {
  Mode_Enter = 0x0001u,
  Mode_Exit = 0x0002u,
  EnterBootloader = 0x0003u,
  ReadSerialNumber = 0x0020u,
  ReadFwNum = 0x0021u,
  ReadFwVer = 0x0022u,
  Reset_Timeout = 0x0027u,
#ifdef DEBUG_BUILD
  Mode_CLI_Enter = 0x0028,
#endif
  Read_OpMode = 0x0029u,
  Comp_prod = 0x0030u,
  Previous_OpMode = 0x0031,


  /*Timers commands*/
  Timers_SetSystemTime = 0x0101u,
  Timers_ResetRate = 0x0102u,
  Timers_IncreaseRate = 0x0103u,
  Timers_GetSystemTime = 0x0104u,
  /*Demount detection commands*/
  Demount_ChangeStatus = 0x0201u,
  Demount_SimulateState = 0x0202u,//FTM changes. added

  /*CO acquisition commands*/
  CO_SetState = 0x0401u,
  CO_SetBISTPeriodicity = 0x0402u,
  CO_SetMeasurementPeriodicity = 0x0403u,
  CO_SetMuteStatus = 0x0404u,
  CO_SimulatedCOLevel = 0x0405u,
  CO_RunBIST = 0x040B,
  CO_RunSensitivityTest = 0x040C,
  CO_ReadSensitivityTestData = 0x040D,
  CO_StopSensitivityTest = 0x040E,
  /*Smoke detection commands*/
  Smoke_SetState = 0x0501u,
  Smoke_SetMeasurementPeriodicity = 0x0502u,
  Smoke_SetBISTPeriodicity = 0x0503u,
  Smoke_SetChamberBISTPeriodicity = 0x0504u,
  Smoke_SimulatedLevel = 0x0505u,
  Smoke_SetMuteStatus = 0x0506u,
  Smoke_RunBIST = 0x050CU,
  Smoke_StartSensitivityTest = 0x050DU,
  Smoke_StopSensitivityTest = 0x050EU,
  Smoke_ReadSensitivityTestData = 0x050FU,
  /*Heat detection commands*/
  Heat_SetState = 0x0601u,
  Heat_SetDetPeriodicity = 0x0602u,
  Heat_SetBISTPeriodicity = 0x0603,
  Heat_SetMuteState = 0x0604u,
  Heat_SimulateLevel = 0x0605u,
  Heat_RunBIST = 0x0607U,
  Heat_StartSensitivityTest = 0x0608U,
  Heat_StopSensitivityTest = 0x0609U,
  Heat_ReadSensitivityTestData = 0x060AU,
  /*Battery monitoring commands*/
  Battery_SetState = 0x0701u,
  Battery_SetBISTPeriodicity = 0x0702u,
  Battery_SimVolLvl = 0x0703u,
  Battery_SimImpLvl = 0x0704u,
  Battery_RunBIST = 0x0705u,
  Battery_MeasureVol = 0x0706u,
  Battery_MeasureImp = 0x0707u,
  /*Buzzer commands*/
  Buzzer_SetState = 0x0801u,
  Buzzer_StartTest = 0x0802u,
  Buzzer_GetState = 0x0803u,
  Buzzer_RunBist = 0x0804u,
  Buzzer_SoundOutputTest = 0x0805u,
  /*Humidity commands*/
  Humidity_SetBISTState = 0x0901u,
  Humidity_SimulateLevel = 0x0902u,
  /*Temperature commands*/
  Temperature_SetBISTState = 0x0A01u,
  Temperature_SimulateTemperatureValue = 0x0A02u,
  /*LED commands*/
  LED_GetState = 0x0B01u,
  LED_StartTest = 0x0B02u,
  /*Obstacle Coverage commands*/
  Obs_SelectSensor = 0x0C01u,
  Obs_SimObDis = 0x0C02u,
  Obs_SetObDetPeriod = 0x0C03u,
  Obs_SetObDetBISTState = 0x0C04u,
  Obs_SetCoDetBISTState = 0x0C05u,
  Obs_GetDetResult = 0x0C0Fu,
  Obs_StopTests = 0x0C10u,
  Obs_SensorResult = 0x0C11u,
  Obs_RunBist = 0x0C12u,
  Obs_BistResult = 0xC13u,
  /*Assistance light commands*/
  AssistanceLight_SetState = 0x0D01u,
  /*CPU reset command*/
  Reset_Soft = 0x0E01u,
  Reset_I2C = 0x0E02u,

  /*EEPROM commands*/
  EEPROM_StopTest = 0x0F01U,
  EEPROM_SimulateCorruption = 0x0F02U,
  EEPROM_DownloadData = 0x0F03U,
  EEPROM_ClearEEPROM = 0x0F04U,
  EEPROM_ReadRow = 0x0F08U,
  /*Ambient light commands*/
  AmbientLight_SetStatus = 0x1001U,
  /*Switch commands*/
  Switch_SetReadPeriodicity = 0x1101U,
  Switch_GetStatus = 0x1102U,
  Switch_GetPeriodicityResult = 0x1103,
  Switch_StopTest = 0x1104,
  /*Configuration commands*/
  SystemConfigFlags = 0x1201U,
  DeviceConfigFlags = 0x1202U,
  /*Radio commands*/
  Radio_Test = 0x1301U,
  /* factory specific test */
  UART_Factory_Test = 0x1501U,
  /*Debug Port - JTAG*/
  Read_DBG = 0x1701u,
  Write_DBG = 0x1702u,
  /*Fault Status*/
  Fault_Read = 0xF001U,
  Fault_Clear = 0xF002U,
  Fault_Restore = 0xF003U,
} P0200_FTMCommand_t;



#endif /*P0200_FTM_H*/
