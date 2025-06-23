#include "P0200_FTM.h"
#include "acquisition_Heat.h"
#ifdef BTL_EN
#include "btl_interface.h"
#endif
#include "production.h"
#include "em_eusart.h"
#include "uartCLI.h"
#include "em_i2c.h"
#include "sl_i2cspm_instances.h"
#include "em_gpio.h"

#ifdef FTM_BUILD


const nextGenCommsCommand_t P0200_FTMCommandTable[P0200_FTM_COMMAND_COUNT] = {
  {.command = Mode_Enter, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Mode_Enter},
  {.command = Mode_Exit, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Mode_Exit},
  {.command = EnterBootloader, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_EnterBootloader},
  {.command = ReadSerialNumber, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_ReadSerialNumber},
  {.command = ReadFwNum, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_ReadFirmwareNum},
  {.command = ReadFwVer, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_ReadFirmwareVer},
  {.command = Reset_Timeout, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Reset_Timeout},
#ifdef DEBUG_BUILD
  {.command = Mode_CLI_Enter, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = CLI_Mode_Enter},
#endif
  {.command = Read_OpMode, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Read_OpMode},
  {.command = Comp_prod, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Comp_Prod},
  {.command = Previous_OpMode, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Previous_OpMode},

  {.command = Timers_SetSystemTime, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = FTM_Timers_SetSystemTime},
  {.command = Timers_ResetRate, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Timers_ResetRate},
  {.command = Timers_IncreaseRate, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Timers_IncreaseRate},
  {.command = Timers_GetSystemTime, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Timers_GetSystemTime},

  {.command = Demount_ChangeStatus, .rxSizeMin = 1u,  .rxSizeMax = 1u,.handler = FTM_Demount_ChangeStatus},
  {.command = Demount_SimulateState, .rxSizeMin = 1u,  .rxSizeMax = 1u,.handler = FTM_Demount_SimulateState},

  {.command = CO_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_CO_SetState},
  {.command = CO_SetBISTPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_CO_SetBISTPeriodicity},
  {.command = CO_SetMeasurementPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_CO_SetMeasurementPeriodicity},
  {.command = CO_SetMuteStatus, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_CO_SetMuteStatus},
  {.command = CO_SimulatedCOLevel, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_CO_SimulatedCOLevel},
  {.command = CO_RunBIST, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_CO_RunBIST},
  {.command = CO_RunSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_CO_RunSensitivityTest},
  {.command = CO_ReadSensitivityTestData, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_CO_ReadSensitivityTestData},
  {.command = CO_StopSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_CO_StopSensitivityTest},

  {.command = Smoke_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Smoke_SetState},
  {.command = Smoke_SetMeasurementPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Smoke_SetMeasurementPeriodicity},
  {.command = Smoke_SetBISTPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Smoke_SetBISTPeriodicity},
  {.command = Smoke_SetChamberBISTPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Smoke_SetChamberBISTPeriodicity},
  {.command = Smoke_SimulatedLevel, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = FTM_Smoke_SimulatedLevel},
  {.command = Smoke_SetMuteStatus, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Smoke_SetMuteStatus},
  {.command = Smoke_RunBIST, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Smoke_RunBIST},
  {.command = Smoke_StartSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Smoke_StartSensitivityTest},
  {.command = Smoke_StopSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Smoke_StopSensitivityTest},
  {.command = Smoke_ReadSensitivityTestData, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Smoke_ReadSensitivityTestData},

  {.command = Heat_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Heat_SetState},
  {.command = Heat_SetDetPeriodicity, .rxSizeMin = 2u,  .rxSizeMax = 2u,.handler = FTM_Heat_SetDetPeriodicity},
  {.command = Heat_SetBISTPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Heat_SetBISTPeriodicity},
  {.command = Heat_SetMuteState, .rxSizeMin = 1u,  .rxSizeMax = 1u,.handler = FTM_Heat_SetMuteState},
  {.command = Heat_SimulateLevel, .rxSizeMin = 1u,  .rxSizeMax = 1u,.handler = FTM_Heat_SimulateLevel},
  {.command = Heat_RunBIST, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Heat_RunBIST},
  {.command = Heat_StartSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Heat_StartSensiTest},
  {.command = Heat_StopSensitivityTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Heat_StopSensiTest},
  {.command = Heat_ReadSensitivityTestData, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Heat_ReadSensiTestData},

  {.command = Battery_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Battery_SetState},
  {.command = Battery_SetBISTPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Battery_SetBISTPeriodicity},
  {.command = Battery_SimVolLvl, .rxSizeMin = 4u, .rxSizeMax = 4u, .handler = FTM_Battery_SimVolLvl},
  {.command = Battery_SimImpLvl, .rxSizeMin = 4u,  .rxSizeMax = 4u,.handler = FTM_Battery_SimImpLvl},
  {.command = Battery_RunBIST, .rxSizeMin = 0u,  .rxSizeMax = 0u,.handler = FTM_Battery_RunBIST},
  {.command = Battery_MeasureVol, .rxSizeMin = 0u,  .rxSizeMax = 0u,.handler = FTM_Battery_MeasureVol},
  {.command = Battery_MeasureImp, .rxSizeMin = 0u,  .rxSizeMax = 0u,.handler = FTM_Battery_MeasureImp},

  {.command = Buzzer_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Buzzer_SetState},
  {.command = Buzzer_StartTest, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Buzzer_StartTest},
  {.command = Buzzer_GetState, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Buzzer_GetState},
  {.command = Buzzer_RunBist, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Buzzer_RunBist},
  {.command = Buzzer_SoundOutputTest, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Buzzer_SoundOutputTest},

  {.command = Humidity_SetBISTState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Humid_SetBISTState},
  {.command = Humidity_SimulateLevel , .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Humid_SimulateLevel},
  {.command = Temperature_SetBISTState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Temp_SetBISTState},
  {.command = Temperature_SimulateTemperatureValue, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Temp_SimulateTempVal},

  {.command = LED_GetState, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_LED_GetState},
  {.command = LED_StartTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_LED_StartTest},

  {.command = Obs_SelectSensor, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Obs_SelectSensor},
  {.command = Obs_SimObDis, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Obs_SimObDis},
  {.command = Obs_SetObDetPeriod, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Obs_SetObDetPeriod},
  {.command = Obs_SetObDetBISTState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Obs_SetObDetBISTState},
  {.command = Obs_SetCoDetBISTState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Obs_SetCoDetBISTState},
  {.command = Obs_GetDetResult, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Obs_GetObsDetResults},
  {.command = Obs_StopTests, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Obs_StopTests},
  {.command = Obs_SensorResult, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Obs_SensorResult},
  {.command = Obs_RunBist, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Obs_RunBist},
  {.command = Obs_BistResult, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Obs_BistResult},

  {.command = AssistanceLight_SetState, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_AssistanceLight_SetState},

  {.command = Reset_Soft, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Reset_Soft},
  {.command = Reset_I2C, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Reset_I2C},

  {.command = EEPROM_StopTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_EEPROM_StopTest},
  {.command = EEPROM_SimulateCorruption, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_EEPROM_SimulateCorruption},
  {.command = EEPROM_DownloadData, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_EEPROM_DownloadData},
  {.command = EEPROM_ClearEEPROM, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_EEPROM_ClearEEPROM},
  {.command = EEPROM_ReadRow, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_EEPROM_ReadRow},

  {.command = AmbientLight_SetStatus, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_AmbientLight_SetStatus},

  {.command = Switch_SetReadPeriodicity, .rxSizeMin = 2u, .rxSizeMax = 2u, .handler = FTM_Switch_SetReadPeriodicity},
  {.command = Switch_GetStatus, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Switch_GetStatus},
  {.command = Switch_GetPeriodicityResult, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Switch_GetPeriodicityResult},
  {.command = Switch_StopTest, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Switch_StopTest},

  {.command = SystemConfigFlags, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_SystemConfigFlags},
  {.command = DeviceConfigFlags, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_DeviceConfigFlags},

  {.command = Radio_Test, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Radio_Test},
  
  {.command = UART_Factory_Test, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Uart_FactoryTest},
  
  {.command = Read_DBG, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Read_DBG_Reg},
  {.command = Write_DBG, .rxSizeMin = 1u, .rxSizeMax = 1u, .handler = FTM_Write_DBG_Reg},

  {.command = Fault_Read, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Fault_Read},
  {.command = Fault_Clear, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Fault_Clear_All},
  {.command = Fault_Restore, .rxSizeMin = 0u, .rxSizeMax = 0u, .handler = FTM_Fault_Restore},

};

static behaviour_state_enum_System_modes previous_system_mode = Standby_Mode;
static uint32_t fault_flag_status = 0U;
static uint8_t endOfProductionTimer = 120U;

static void set_prod_comp_timer(uint8_t period);

/*******************************************************************************
 * @brief Start_Battery_Timer
 * @details Starts the timer
 * @param None
 * @return None
 ******************************************************************************/
void Start_Battery_Timer(void)
{
  BURTCTimer_Start(TMR_Battery_Measurement_BIST_event_0, periodical,
                   BATTERY_MEAS_BIST_PERIOD); /* Default battery periodicity is 24hours */
}

/*******************************************************************************
 * @brief restoreSystemDefaults
 * @details Puts the system defaults to previous mode
 * @param None
 * @return None
 ******************************************************************************/
void RestoreFTMDefaults(void)
{
  set_ftm_btn_pattern(false, false, false, false);
  set_ftm_btn_event(false);
  set_ftm_sub_command(0U);
  SetSimulatedHeatMode(false);
  if(getXModemEnable())
  {
    xModemModeExit();
  }
  setXModemEnable(false);
  if(getIncresedTimerRate())
  {
      BURTC_reinit(10U);
  }
  setIncresedTimerRate(false);
  smoke_set_ftm_chamber_test(false);
  if(GetAssistanceLightStatus())
  {
      GPIO_TurnAssistanceLEDoff();
  }
  SetAssistanceLightStatus(false);
  LEDBuzz_SetFTMPulseTone(false);
  set_obs_det_ftm_timeover(false);
  set_obs_det_ftm_period_timeover(false);
  (void)BURTCTimer_Stop(FTM_Test_Btn_event_1);
  setBehavioural_Operational_State(state_Idle);
}

/*******************************************************************************
 * @brief exitProtocol
 * @details Puts the system into exit FTM mode
 * @param None
 * @return None
 ******************************************************************************/
void exitFTMProtocol(void)
{
  if(ngCommsDriver.isProtocolActive)
  {
      RTOS_ERR err;
      restore_uart_tx_pin_mode();
#ifndef DEBUG_BUILD
      /* Get production complete status */
      const uint32_t prod_comp = prod_get_value( );

      /* enable the dma wakeup in FTM mode, only during the production lockout period */
      if(prod_comp != PROD_COMP_BB)
      {
      	commsAppInit_FTMMode(false);
      }
#endif
      //restore_uart_tx_pin_mode();
      DEBUG_FTM("\nExit FTM Mode Entered", false, 0U);
      (void)BURTCTimer_Stop(FTM_Timeout_event_1);
      ngCommsDriver.isProtocolActive = false;
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      faultFlagsRestore();
      RestoreFTMDefaults();
      setBehavioural_System_Modes(previous_system_mode);
      OSTimeDly(2, OS_OPT_TIME_DLY, &err);
      Start_Diagnostic_BIST();
      Start_Battery_Timer();
      DEBUG_FTM("\nExit FTM Mode Leaving", false, 0U);
  }
}

/*******************************************************************************
 * @brief resetProtocolTimoutTimer
 * @details This is timer reset function
 * @param None
 * @return None
 ******************************************************************************/
void resetProtocolTimoutTimer(void)
{
  (void)BURTCTimer_Stop(FTM_Timeout_event_1);
  BURTCTimer_Start(FTM_Timeout_event_1, one_shot, FTM_MODE_TIMEOUT);
}

/*******************************************************************************
 * @brief nextGenComms_FTM_Mode_Enter
 * @details Puts the system in function test mode FTM
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Mode_Enter(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  behaviour_state_enum_System_modes current_system_mode = getBehavioural_System_Modes(false);

  if(current_system_mode != Functional_Test_Mode)
  {
      RTOS_ERR err;

#ifndef DEBUG_BUILD
      /* Get production complete status */
      const uint32_t prod_comp = prod_get_value( );

      /* enable the dma wakeup in FTM mode, only during the production lockout period */
      if(prod_comp != PROD_COMP_BB)
      {
		 commsAppInit_FTMMode(true);   
	  }
#endif

      setBehavioural_System_Modes(Functional_Test_Mode);
      OSTimeDly(2, OS_OPT_TIME_DLY, &err);
      previous_system_mode = current_system_mode;
      ngCommsDriver.isProtocolActive = true;
      Stop_All_timers_except_timestamp();
      fault_flag_status = FaultHandler_GetFaultFlags();
      setFaultFlagStatus(fault_flag_status);
      RestoreFTMDefaults();
      BURTCTimer_Start(FTM_Timeout_event_1, one_shot, FTM_MODE_TIMEOUT);
      DEBUG_FTM("\nEntered FTM Mode", false, 0U);
  }
  else
  {
      retVal = NG_NACK_REASON_InvalidDeviceState;
      DEBUG_FTM("\nError Already in FTM Mode", false, 0U);
  }

  resultData->bufferLength = 0U;
  return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_Mode_Exit
 * @details Exits function test mode FTM back to latest system mode
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Mode_Exit(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  exitFTMProtocol();
  resultData->bufferLength = 0U;
  return NG_NACK_REASON_OK;

}
/*******************************************************************************
 * @brief nextGenComms_FTM_EnterBootloader
 * @details Enter bootloader
 * @param message   received message
 * @param messageSize size of the received message
 * @param resultData  pointer to tx data structure
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EnterBootloader(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;
  nextGenCommsAckNackReason_t Result = NG_NACK_REASON_OK;
#ifdef BTL_EN

  int32_t status = bootloader_init();

  if(status == BOOTLOADER_OK)
    {
      bootloader_rebootAndInstall();
    }
  else
    {
      Result = NG_NACK_REASON_SDKError;
   }

#endif

  return Result;
}

#ifdef DEBUG_BUILD
extern nextGenCommsAckNackReason_t CLI_Mode_Enter(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
    (void) message;
    (void) messageSize;
    (void) resultData;

    exitFTMProtocol();
    EUSART_Enable(EUSART0, false);
    EUSART_IntClear(EUSART0, 0xFFFFFFFFu);
    EUSART0->STARTFRAMECFG = 0;
    EUSART0->SIGFRAMECFG = 0;
    EUSART_Enable(EUSART0, eusartEnable);
    UARTCLI_SetCliMode(true); //Enter CLI Mode
    resultData->bufferLength = 0U;
    return NG_NACK_REASON_OK;
}
#endif

/*******************************************************************************
 * @brief FTM_ReadSerialNumber
 * @details Read product serial number
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_ReadSerialNumber(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

  uint32_t serialNum = 0U;
  serialNum = DataLogging_GetDeviceID();
  resultData->buffer[0U] = ((serialNum >> 24U) & 0xFFU);
  resultData->buffer[1U] = ((serialNum >> 16U) & 0xFFU);
  resultData->buffer[2U] = ((serialNum >> 8U) & 0xFFU);
  resultData->buffer[3U] = (serialNum & 0xFFU);
  resultData->bufferLength = 4U;
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_ReadFirmwareNum
 * @details Read product firmware number (SAxxxx)
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_ReadFirmwareNum(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;


  resultData->buffer[0U] = (uint8_t)FW_SA_NUM_1;
  resultData->buffer[1U] = (uint8_t)FW_SA_NUM_2;
  resultData->bufferLength = 2U;
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_ReadFirmwareVer
 * @details Read product firmware version
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_ReadFirmwareVer(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  resultData->buffer[0U] = (uint8_t)FW_MAJOR_REV;
  resultData->buffer[1U] = (uint8_t)FW_MINOR_REV;
  resultData->buffer[2U] = (uint8_t)FW_BUILD_REV;
  resultData->bufferLength = 3U;
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Reset_Timeout
 * @details Resets FTM timeout period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Reset_Timeout(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  (void)BURTCTimer_Stop(FTM_Timeout_event_1);
   BURTCTimer_Start(FTM_Timeout_event_1, one_shot, FTM_MODE_TIMEOUT);

  DEBUG_FTM("\nFTM Reset Timeout", false, 0U);
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Read_OpMode
 * @details Read out system operating mode
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Read_OpMode(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  resultData->buffer[0U] = (uint8_t)getBehavioural_System_Modes(false);
  resultData->bufferLength = 1U;

  DEBUG_FTM("\nFTM Read out OP mode", false, 0U);
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Comp_Prod
 * @details This command sends terminate production to MCU2
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Comp_Prod(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  RTOS_ERR err;
  bool eCheckSuccess = false;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  resultData->bufferLength = 0U;

  set_prod_comp_timer(message[0]);
  eCheckSuccess = dl_set_production_timer((uint32_t)message[0]);                 /*Save amount of hrs*/

  if(eCheckSuccess == true)
  {
      eCheckSuccess = dl_set_prod_commence_time(get_currentTime());             /*Save current time*/

      if(eCheckSuccess == true)
      {

          if((hal_get_ads_state() == Ads_offBase) && (message[0] == 0U))
          {
              eCheckSuccess = prod_set_value(PROD_COMP_BB);
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              Reset_EEPROM_Production();
          }
          else if((hal_get_ads_state() == Ads_offBase) && (message[0] >= 1U))
          {
              eCheckSuccess = prod_set_value(PROD_COMP_AA);
              BURTCTimer_Start(TMR_Production_complete_1, one_shot, ((message[0]*3600U)/BURTC_PERIOD));
          }
          else
          {
              eCheckSuccess = prod_set_value(PROD_COMP_AA);
          }

          if(eCheckSuccess == true)
          {
              SPIComms_Send_Data_to_MCU2(SPI_CMD_Termination_production);
              DEBUG_FTM("\nFTM Complete Production success", true, message[0]);
          }
          else
          {
              DEBUG_FTM("\nFTM Complete Production failure", false, 0u);
              retVal = NG_NACK_REASON_TestFailed;
          }
      }
      else
      {
          DEBUG_FTM("\nFTM Complete Production failure", false, 0u);
          retVal = NG_NACK_REASON_TestFailed;
      }
  }
  else
  {
      DEBUG_FTM("\nFTM Complete Production failure", false, 0u);
      retVal = NG_NACK_REASON_TestFailed;
  }

  return retVal;
}

/*******************************************************************************
 * @brief FTM_Previous_OpMode
 * @details Read out previous system operating mode before entering FTM
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Previous_OpMode(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  resultData->buffer[0U] = (uint8_t)previous_system_mode;
  resultData->bufferLength = 1U;
  DEBUG_FTM("\nFTM Read out previous OP mode", false, 0U);
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Reset_Soft
 * @details Resets CPU
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Reset_Soft(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  NVIC_SystemReset();

  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Reset_I2C
 * @details Resets I2C
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Reset_I2C(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
    (void) message;
    (void) messageSize;
    (void) resultData;

    I2C_Reset(I2C0);
    sl_i2cspm_init_instances();

    return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Read_DBG_Reg
 * @details Reads DBGROUTEPEN Register status
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Read_DBG_Reg(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  uint32_t regRead = 0U;
  uint8_t regResult = 0U;

  for(uint8_t i = 0U; i < 4U; i++)
  {
      regRead = GetDBGRegPinStatus(i);
      if((regRead & 0x1) == 1U)
      {
              regResult |= (1U << i);
      }
  }

  resultData->buffer[0U] = regResult;
  resultData->bufferLength = 1U;


  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Write_DBG_Reg
 * @details Write to DBGROUTEPEN Register
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Write_DBG_Reg(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  for(uint8_t i = 0U; i < 4U; i++)
  {
      SetDBGRegPinStatus(i, (message[0] & (1U << i)));
  }

  return NG_NACK_REASON_OK;
}

bool FTM_Is_NG_Protocol_Active()
{
    uint32_t stfrm = 0;
    uint32_t etfrm = 0;
    stfrm = EUSART0->STARTFRAMECFG;
    etfrm = EUSART0->SIGFRAMECFG;
    return ((stfrm == 0x02) && (etfrm == 0x03));
}

behaviour_state_enum_System_modes get_previous_mode(void)
{
  return previous_system_mode;
}

static void set_prod_comp_timer(uint8_t period)
{
  endOfProductionTimer = period;
}

uint8_t get_prod_comp_timer(void)
{
  return endOfProductionTimer;
}

#endif
