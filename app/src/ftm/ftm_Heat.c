
#include "P0200_FTM.h"
#include "acquisition_Heat.h"
#include "diagnostics.h"

static uint8_t HeatSensitivityTestData[SH_TEST_DATA_BYTES];
static bool b_isHeatEventSet = false;

/*******************************************************************************
 * @brief FTM_Heat_SetState
 * @details Sets the heat detection state
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

    (void) messageSize;
    (void) resultData;

    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
    bool paramValid = true;
    const uint8_t newState = message[0U];

    switch(newState)
    {
      case 1:
        setBehavioural_Operational_State(state_Heat_Alarm);
        LEDBuzz_Post(PatternAlarmHeat);

        SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);

         /* Increment the counter smoke alarm */
        DataLogging_SetHeatEvent((uint8_t)EVENT_TYPE_LOCAL);
        DataLogging_SetEventLogbookRecord(DEF_LBE_HEAT_DET_START, NULL);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Countersdates);
        b_isHeatEventSet = true;
        DEBUG_FTM("\nFTM Heat Set State 01", false, 0U);
        break;
      case 2:
        setBehavioural_Operational_State(state_Idle);
        LEDBuzz_Post(PatternAlarmHeatStop);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
        DataLogging_SetEventLogbookRecord(DEF_LBE_HEAT_DET_END, NULL);
        b_isHeatEventSet = false;
        DEBUG_FTM("\nFTM Heat Set State 02", false, 0U);
        break;
      case 3:
        FaultHandler_FaultClearAll();
        LEDBuzz_Post(PatternStopAll);
        FaultHandler_FaultSet(HeatSensorHwFault);
        DataLogging_SetEventLogbookRecord(DEF_LBE_HEAT_DET_HW_ERR_START, NULL);
        DEBUG_FTM("\nFTM Heat Set State 03", false, 0U);
        break;
      case 4:
        FaultHandler_FaultClear(HeatSensorHwFault);
        LEDBuzz_Post(PatternStopMajorFault);
        DataLogging_SetEventLogbookRecord(DEF_LBE_HEAT_DET_HW_ERR_END, NULL);
        DEBUG_FTM("\nFTM Heat Set State 04", false, 0U);
        break;
      default:
        retVal = NG_NACK_REASON_InvalidData;
        paramValid = false;
        DEBUG_FTM("\nERROR FTM Heat Set State Invalid Data", false, 0U);
        break;
    }

    if(paramValid)
    {
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
    }

    return retVal;
  return NG_NACK_REASON_OK;
}


/*******************************************************************************
 * @brief FTM_HeatSetDetPeriodicity
 * @details Sets the heat detection period
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_SetDetPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

  uint16_t new_Period = 0U;
  uint16_t modulo_check = 0U;
  int32_t temp = 0;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  new_Period = message[0U];
  new_Period = new_Period << 8U;
  new_Period = new_Period | message[1U];
  modulo_check = new_Period % BURTC_PERIOD;  /* check new period is multiples of 10 */

  if(modulo_check != 0U)
  {
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM Heat Mesurement Periodicity Invalid Data", false, 0U);
  }
  else
  {
      new_Period = new_Period / BURTC_PERIOD;
      BURTCTimer_Start(TMR_Heat_measure_BIST_event_0, periodical, new_Period);

      if (heat_measurement(false) == true)
      {
          resultData->buffer[0U] = (uint8_t)(getThermistorADC() >> 8U) & 0xFFU;
          resultData->buffer[1U] = (uint8_t) getThermistorADC() & 0xFFU;
          temp = getHeatAfterCompensation();
          if(temp >= 0)
          {
              resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
              resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
              resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
              resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
          }
          else
          {
               temp = ~temp;      /*2's complement for neg values*/
               temp = temp + 1;

               resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
               resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
               resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
               resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
          }


          resultData->buffer[6U] = (uint8_t)getHeatState();
          resultData->bufferLength = 7U;

      }
      else
      {
          retVal = NG_NACK_REASON_TestFailed;
          (void)BURTCTimer_Stop(TMR_Heat_measure_BIST_event_0);
      }

  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Heat_SetBISTPeriodicity
 * @details Sets the heat BIST period
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

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
        DEBUG_FTM("\nERROR FTM Heat BIST Periodicity Invalid Data", false, 0U);
    }
    else
    {
        new_Period = new_Period / BURTC_PERIOD;
        BURTCTimer_Start(TMR_Heat_measure_BIST_event_0, periodical, new_Period);

        resultData->buffer[0U] = (uint8_t)heat_measurement(true);

        if(resultData->buffer[0U] == 1U)
        {
            resultData->buffer[0U] = 0U;  // BIST Passed
        }
        else
        {
            resultData->buffer[0U] = 1U;  // BIST Failed
        }

        resultData->bufferLength = 1U;
    }

    return retVal;
}


/*******************************************************************************
 * @brief FTM_Heat_SetMuteState
 * @details Sets the heat detection mute state
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_SetMuteState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

    (void) messageSize;
    (void) resultData;

    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
    const uint8_t newState = message[0U];
    bool paramValid = true;

    switch(newState)
    {
      case 1:
        setBehavioural_Operational_State(state_Heat_Alarm_Silence);
        LEDBuzz_Post(PatternAlarmSilence);
        DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_START, NULL);
        DEBUG_FTM("\nFTM Heat Mute 01", false, 0U);
        break;
      case 2:
        if(b_isHeatEventSet || ((uint8_t)getHeatState() == 1U) || ((uint8_t)getHeatState() == 2U))
        {
            LEDBuzz_Post(PatternAlarmHeatStop);
            setBehavioural_Operational_State(state_Heat_Alarm);
            LEDBuzz_Post(PatternAlarmHeat);
        }
        else
        {
            setBehavioural_Operational_State(state_Heat_Alarm_Silence);
            LEDBuzz_Post(PatternAlarmHeatStop);
        }
        DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_END, NULL);
        DEBUG_FTM("\nFTM Heat Mute 02", false, 0U);
        break;
      default:
        paramValid = false;
        retVal = NG_NACK_REASON_InvalidData;
        DEBUG_FTM("\nERROR FTM Heat Mute Invalid Data", false, 0U);
        break;

    }

    if(paramValid)
    {
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
    }

    return retVal;
}


/*******************************************************************************
 * @brief FTM_Heat_SimulateHeatLevel
 * @details Sets the simulated heat level
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_SimulateLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) messageSize;
  (void) resultData;

  int32_t temp = 0;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  SetSimulatedHeatMode(true);
  InjectCurrentHeatValue(message[0]*10U);
  if(heat_measurement(false) == true)
  {
      resultData->buffer[0U] = (uint8_t)(getThermistorADC() >> 8U) & 0xFFU;
      resultData->buffer[1U] = (uint8_t) getThermistorADC() & 0xFFU;
      temp = getHeatAfterCompensation();
      if(temp >= 0)
      {
          resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
          resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
          resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
          resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
      }
      else
      {
           temp = ~temp;      /*2's complement for neg values*/
           temp = temp + 1;

           resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
           resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
           resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
           resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
      }
      resultData->buffer[6U] = (uint8_t)getHeatState();
      resultData->bufferLength = 7U;
  }
  else
  {
      retVal = NG_NACK_REASON_TestFailed;
  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Heat_RunBIST
 * @details Runs the heat BIST
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
   (void) message;
   (void) messageSize;

   resultData->buffer[0U] = (uint8_t)heat_measurement(true);

    if(resultData->buffer[0U] == 1U)
    {
        resultData->buffer[0U] = 0U;  // BIST Passed
    }
    else
    {
        resultData->buffer[0U] = 1U;  // BIST Failed
    }

    resultData->bufferLength = 1U;

   return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Heat_StartSensiTest
 * @details Performs the heat detection at set periodicity time
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_StartSensiTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
   (void) message;
   (void) messageSize;

   nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
   int32_t temp = 0;
   bool bistResult = false;

   if(temp_humid_measure_bist(&bistResult) != false)
   {
         retVal = NG_NACK_REASON_TestFailed;
         DEBUG_FTM("\nERROR Heat Smoke StartSensitivityTest", false, 0U);
   }
   else
   {

      BURTCTimer_Start(TMR_Heat_measure_BIST_event_0, periodical, HEAT_MEASURMENT_BIST_PERIOD);

      if(heat_measurement(false) == true)
      {
          resultData->buffer[0U] = (uint8_t)(getThermistorADC() >> 8U) & 0xFFU;
          resultData->buffer[1U] = (uint8_t) getThermistorADC() & 0xFFU;

          temp = getHeatAfterCompensation();
          if(temp >= 0)
          {
              resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
              resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
              resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
              resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
          }
          else
          {
               temp = ~temp;      /*2's complement for neg values*/
               temp = temp + 1;

               resultData->buffer[2U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
               resultData->buffer[3U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
               resultData->buffer[4U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
               resultData->buffer[5U] = (uint8_t)( temp & 0x000000FFu);
          }

          set_ftm_sub_command(0U);
          temp = get_TempVal();

          if(temp >= 0 )
          {
              resultData->buffer[6U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
              resultData->buffer[7U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
              resultData->buffer[8U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
              resultData->buffer[9U] = (uint8_t)( temp & 0x000000FFu);
          }
          else
          {
              temp = ~temp;      /*2's complement for neg values*/
              temp = temp + 1;

              resultData->buffer[6U] = (uint8_t)((temp & 0xFF000000u) >> 24u);
              resultData->buffer[7U] = (uint8_t)((temp & 0x00FF0000u) >> 16u);
              resultData->buffer[8U] = (uint8_t)((temp & 0x0000FF00u) >> 8u);
              resultData->buffer[9U] = (uint8_t)( temp & 0x000000FFu);

          }

          resultData->buffer[10U] = (uint8_t)((get_HumidityVal() & 0xFF000000u) >> 24u);
          resultData->buffer[11U] = (uint8_t)((get_HumidityVal() & 0x00FF0000u) >> 16u);
          resultData->buffer[12U] = (uint8_t)((get_HumidityVal() & 0x0000FF00u) >> 8u);
          resultData->buffer[13U] = (uint8_t)( get_HumidityVal() & 0x000000FFu);

          resultData->buffer[14U] = (uint8_t)getHeatState();

          resultData->buffer[15U] = (uint8_t)((get_currentTime() & 0xFF000000u) >> 24u);
          resultData->buffer[16U] = (uint8_t)((get_currentTime() & 0x00FF0000u) >> 16u);
          resultData->buffer[17U] = (uint8_t)((get_currentTime() & 0x0000FF00u) >> 8u);
          resultData->buffer[18U] = (uint8_t)( get_currentTime() & 0x000000FFu);

          resultData->bufferLength = SH_TEST_DATA_BYTES;

          /* Store result for test ReadSmokeSensitivityTestData */
          for(uint16_t i = 0U; i < resultData->bufferLength; i++)
          {
                   HeatSensitivityTestData[i] = resultData->buffer[i];
          }

          DEBUG_FTM("\nFTM Heat StartSensitivityTest", false, 0U);

      }
      else
      {
          retVal = NG_NACK_REASON_TestFailed;
          (void)BURTCTimer_Stop(TMR_Heat_measure_BIST_event_0);
          DEBUG_FTM("\nERROR Heat Smoke StartSensitivityTest", false, 0U);
      }
   }


   return retVal;
}

/*******************************************************************************
 * @brief FTM_Heat_ReadSensiTestData
 * @details Reads the Heat Dectection data
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_ReadSensiTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
   (void) message;
   (void) messageSize;

   /* Restore result for test RunSmokeSensitivityTest */
   for(uint8_t i = 0U; i < SH_TEST_DATA_BYTES; i++)
   {
        resultData->buffer[i] = HeatSensitivityTestData[i];
   }

   resultData->bufferLength = SH_TEST_DATA_BYTES;

   DEBUG_FTM("\nFTM Heat ReadSensitivityTestData", false, 0U);
   return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Heat_StopSensiTest
 * @details Stops Heat Detection periodicity and returns to FTM
 * @param const uint8_t message[], const uint8_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Heat_StopSensiTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
    (void) message;
    (void) messageSize;
    (void) resultData;

    b_isHeatEventSet = false;
    SetSimulatedHeatMode(false);
    (void)BURTCTimer_Stop(TMR_Heat_measure_BIST_event_0);
    setBehavioural_Operational_State(state_Idle);
    FaultHandler_FaultClearAll();
    LEDBuzz_Post(PatternStopAll);

    DEBUG_FTM("\nFTM Heat StopSensitivityTest", false, 0U);
    return NG_NACK_REASON_OK;

}
