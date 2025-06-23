#include  <kernel/include/os.h>
#include "P0200_FTM.h"
#include "battery_measurement.h"

/*******************************************************************************
 * @brief FTM_Battery_SetState
 * @details Sets the battery monitoring state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Battery_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;
  const uint8_t newState = message[0U];

  switch(newState)
  {
     case 1:
       DataLogging_SetEventLogbookRecord(DEF_LBE_BATTERY_ERR_START, NULL);
       FaultHandler_FaultClearAll();
       LEDBuzz_Post(PatternStopAll);
       FaultHandler_FaultSet(BatteryFault);
       DEBUG_FTM("\nFTM Batt Set State 01", false, 0U);
       break;
     case 2:
       FaultHandler_FaultClear(BatteryFault);
       LEDBuzz_Post(PatternStopMajorFault);
       DEBUG_FTM("\nFTM Batt Set State 02", false, 0U);
       break;
     default:
       paramValid = false;
       DEBUG_FTM("\nFTM Batt Invalid State", false, 0U);
       retVal = NG_NACK_REASON_InvalidData;
       break;
  }

  if(paramValid)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Battery_SetBISTPeriodicity
 * @details Sets the battery BIST period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Battery_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

   uint16_t new_Period = 0U;
   uint16_t modulo_check = 0U;
   nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

   new_Period = message[0U];
   new_Period = new_Period << 8U;
   new_Period = new_Period | message[1U];
   modulo_check = new_Period % BURTC_PERIOD;  /* check new period is multiples of 10 */

   if(modulo_check != 0U)
   {
       retVal = NG_NACK_REASON_InvalidData;
       DEBUG_FTM("\nERROR FTM Batt Set Periodicity Invalid Data", false, 0U);
   }
   else
   {
       new_Period = new_Period / BURTC_PERIOD;
       BURTCTimer_Start(TMR_Battery_Measurement_BIST_event_0, periodical, new_Period);
       DEBUG_FTM("\nFTM Batt Set Periodicity", false, 0U);

   }

   return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_BatteryMonitoring_SimulateVoltageLevel
 * @details Sets the battery simulated voltage level
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Battery_SimVolLvl(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) messageSize;
  (void) resultData;

  //set_battery_A_Voltage(message[0U], message[1U]);
  //set_battery_B_Voltage(message[2U], message[3U]);

  uint32_t batt_vol_A = 0U;
  uint32_t batt_vol_B = 0U;

  batt_vol_A = message[0U];
  batt_vol_A = batt_vol_A << 8U;
  batt_vol_A = batt_vol_A | message[1U];

  batt_vol_B = message[2U];
  batt_vol_B = batt_vol_B << 8U;
  batt_vol_B = batt_vol_B | message[3U];

  if(((batt_vol_A < BATT_LOW_THRESHOLD) || (batt_vol_A > BATT_MAX_VOLTAGE)) || ((batt_vol_B < BATT_LOW_THRESHOLD) || (batt_vol_B > BATT_MAX_VOLTAGE)))
  {

       if((batt_vol_A < BATT_DEAD_THRESHOLD) || (batt_vol_B < BATT_DEAD_THRESHOLD))
       {
            RTOS_ERR err;
            DataLogging_SetEventLogbookRecord(DEF_LBE_BATTERY_SHUTDOWN_START, NULL);
             /* post the shutdown event */
             OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
             EVENT_SHUTDOWN_0,                    /*   event bit-mask.                     */
             OS_OPT_POST_FLAG_SET,                /*   Set the flag.                       */
             &err);
             /*   Check error code */
             APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
       }
       else
       {
           /* Log the status in eeprom */
           DataLogging_SetEventLogbookRecord(DEF_LBE_BATTERY_ERR_START, NULL);
           /* Indicate the LED and buzzer status using major fault pattern */
           FaultHandler_FaultClearAll();
           LEDBuzz_Post(PatternStopAll);
           FaultHandler_FaultSet(BatteryFault);
       }
  }
  else
  {
      FaultHandler_FaultClear(BatteryFault);
      LEDBuzz_Post(PatternStopMajorFault);
  }

  set_ftm_Batt_A_Sim_Vol(batt_vol_A);
  set_ftm_Batt_B_Sim_Vol(batt_vol_B);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);

  return NG_NACK_REASON_OK;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_BatteryMonitoring_SimulateImpedanceLevel
 * @details Sets the battery simulated impedance level
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Battery_SimImpLvl(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  uint32_t batt_imp_A = 0U;
  uint32_t batt_imp_B = 0U;

  batt_imp_A = message[0U];
  batt_imp_A = batt_imp_A << 8U;
  batt_imp_A = batt_imp_A | message[1U];

  batt_imp_B = message[2U];
  batt_imp_B = batt_imp_B << 8U;
  batt_imp_B = batt_imp_B | message[3U];

  if((batt_imp_A >= BATT_HIGH_IMPEDANCE_THRESHOLD) || (batt_imp_B >= BATT_HIGH_IMPEDANCE_THRESHOLD))
  {
      /* Log the status in eeprom */
        DataLogging_SetEventLogbookRecord(DEF_LBE_BATTERY_ERR_START, NULL);
        /* Indicate the LED and buzzer status using major fault pattern */
        FaultHandler_FaultClearAll();
        LEDBuzz_Post(PatternStopAll);
        FaultHandler_FaultSet(BatteryFault);
  }
  else
  {
      FaultHandler_FaultClear(BatteryFault);
      LEDBuzz_Post(PatternStopMajorFault);
  }

  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);

  return NG_NACK_REASON_OK;
}

nextGenCommsAckNackReason_t FTM_Battery_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  RTOS_ERR err;
  OS_MSG_SIZE size;

  bool result = false;
  hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
  AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
  result = batt_circuit_bist();

  if(result == false)
  {
      resultData->buffer[0U] = 0x00;
  }
  else
  {
      resultData->buffer[0U] = 0x01;
  }

  resultData->bufferLength = 1U;

  DEBUG_FTM("\nFTM Batt Bist", true, resultData->buffer[0U]);

  return NG_NACK_REASON_OK;

}

nextGenCommsAckNackReason_t FTM_Battery_MeasureVol(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  RTOS_ERR err;
  OS_MSG_SIZE size;
  uint16_t batt_A;
  uint16_t batt_B;

  hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
  AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  batt_A = (uint16_t)get_battery_A_Voltage();
  batt_B = (uint16_t)get_battery_B_Voltage();

  resultData->buffer[0U] = (uint8_t)(batt_A >> 8U);
  resultData->buffer[1U] = (uint8_t)(batt_A & 0x00FF);
  resultData->buffer[2U] = (uint8_t)(batt_B >> 8U);
  resultData->buffer[3U] = (uint8_t)(batt_B & 0x00FF);

  resultData->bufferLength = 4U;

  return NG_NACK_REASON_OK;

}


nextGenCommsAckNackReason_t FTM_Battery_MeasureImp(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  RTOS_ERR err;
  OS_MSG_SIZE size;
  uint16_t batt_A;
  uint16_t batt_B;

  hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
  AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  batt_A = (uint16_t)get_battery_A_Impedance();
  batt_B = (uint16_t)get_battery_B_Impedance();

  resultData->buffer[0U] = (uint8_t)(batt_A >> 8U);
  resultData->buffer[1U] = (uint8_t)(batt_A & 0x00FF);
  resultData->buffer[2U] = (uint8_t)(batt_B >> 8U);
  resultData->buffer[3U] = (uint8_t)(batt_B & 0x00FF);

  resultData->bufferLength = 4U;

  return NG_NACK_REASON_OK;

}

