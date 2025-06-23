#include "P0200_FTM.h"


static uint8_t SmokeSensitivityTestData[SH_TEST_DATA_BYTES];
static bool b_isSmokeEventSet = false;

/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SetState
 * @details Sets the smoke detection state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;
  const uint8_t newState = message[0U];

  switch(newState)
  {
    case 1:
      setBehavioural_Operational_State(state_Smoke_Alarm);
      /* This will Transfer MCU-2 Alarm information */
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      LEDBuzz_Post(PatternAlarmSmoke);


       /* Increment the counter smoke alarm */
      DataLogging_SetSmokeEvent((uint8_t)EVENT_TYPE_LOCAL);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_DET_START, NULL);

      /* This will Transfer MCU-2 Counters and date information */
      /*@note Due to MCU-1 firmware is not completely merge : Plz check Counter & dates use as hardcoded values*/
      /*@note Plz check or enable the actual read values*/
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Countersdates);
      b_isSmokeEventSet = true;
      DEBUG_FTM("\nFTM Smoke Set State 01", false, 0U);
      break;
    case 2:
      setBehavioural_Operational_State(state_Idle);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      LEDBuzz_Post(PatternAlarmSmokeStop);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_DET_END, NULL);
      b_isSmokeEventSet = false;
      DEBUG_FTM("\nFTM Smoke Set State 02", false, 0U);
      break;
    case 3:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(DegradedSmokeChamberFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_CHAM_CONT_START, NULL);
      DEBUG_FTM("\nFTM Smoke Set State 03", false, 0U);
      break;
    case 4:
      FaultHandler_FaultClear(DegradedSmokeChamberFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_CHAM_CONT_END, NULL);
      DEBUG_FTM("\nFTM Smoke Set State 04", false, 0U);
      break;
    case 5:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(SmokeChamberDiodeHwFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_CHAM_HW_ERR_START, NULL);
      DEBUG_FTM("\nFTM Smoke Set State 05", false, 0U);
      break;
    case 6:
      FaultHandler_FaultClear(SmokeChamberDiodeHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_CHAM_HW_ERR_END, NULL);
      DEBUG_FTM("\nFTM Smoke Set State 06", false, 0U);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      DEBUG_FTM("\nERROR FTM Smoke Set State Invalid Data", false, 0U);
      break;
  }

  if(paramValid == true)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SetMeasurementPeriodicity
 * @details Sets the smoke measurement period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SetMeasurementPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  uint16_t new_Period = 0U;
  uint16_t modulo_check = 0U;
  bool testParamsValid = true;

   new_Period = message[0U];
   new_Period = new_Period << 8U;
   new_Period = new_Period | message[1U];
   modulo_check = new_Period % BURTC_PERIOD;  /* check new period is multiples of 10 */

   if(new_Period < 2U)  /* minimum time required to carry out test */
   {
        retVal = NG_NACK_REASON_InvalidData;
        DEBUG_FTM("\nERROR FTM Smoke Measurement Periodicity", false, 0U);
   }
   else
   {
       new_Period /= BURTC_PERIOD;

       if(new_Period < 1U)   /*if time is below 10sec then use LEtimer*/
       {
           LETimer_start(LETIMER_FMT_Smoke, message[1U]*1000U);
       }
       else
       {
           if(modulo_check != 0U)
           {
                 testParamsValid = false;
                 retVal = NG_NACK_REASON_InvalidData;
                 DEBUG_FTM("\nERROR FTM Smoke Measurement Periodicity", false, 0U);
           }
           else
           {
               if(getIsDeviceSmokeEnable() == true)
               {
                   BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, (uint32_t)new_Period);
               }
               else
               {
                   testParamsValid = false;
                   retVal = NG_NACK_REASON_CommandNotImplemented;
                   DEBUG_FTM("\nERROR FTM Device not configured for smoke", false, 0U);
               }
           }
       }


       if(testParamsValid == true)
       {
         smoke_set_ftm_chamber_test(false);
         smoke_periodic_measurement();

         resultData->buffer[0U] = (uint8_t) (smoke_get_dark_measurement() >> 8U) & 0xFFU;
         resultData->buffer[1U] = (uint8_t) smoke_get_dark_measurement() & 0xFFU;
         resultData->buffer[2U] = (uint8_t)(smoke_get_bright_measurement() >> 8U) & 0xFFU;
         resultData->buffer[3U] = (uint8_t) smoke_get_bright_measurement() & 0xFFU;
         resultData->buffer[4U] = (uint8_t)(smoke_get_value() >> 8U ) & 0xFFU;
         resultData->buffer[5U] = (uint8_t) smoke_get_value() & 0xFFU;
         resultData->buffer[6U] = (uint8_t) getFTMSmokeState();

         resultData->bufferLength = 7U;

         DEBUG_FTM("\nFTM Smoke Measurement Periodicity", true, new_Period);
      }

   }



   return retVal;

}


/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SetBISTPeriodicity
 * @details Sets the smoke detection BIST period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  uint16_t new_Period = 0U;
  uint16_t modulo_check = 0U;
  bool testParamsValid = true;
  bool bistHWfaultResult = false;

   new_Period = message[0U];
   new_Period = new_Period << 8U;
   new_Period = new_Period | message[1U];
   modulo_check = new_Period % BURTC_PERIOD;   /* check new period is multiples of 10 */

   if(new_Period < 2U)  /* minimum time required to carry out test */
   {
        retVal = NG_NACK_REASON_InvalidData;
        DEBUG_FTM("\nERROR FTM Smoke BIST Periodicity", false, 0U);
   }
   else
   {
       new_Period /= BURTC_PERIOD;

       if(new_Period < 1U)   /*if time is below 10sec then use LEtimer*/
       {
           LETimer_start(LETIMER_FMT_BIST_Smoke, message[1U]*1000U);
       }
       else
       {
           if(modulo_check != 0U)
           {
               testParamsValid = false;
               retVal = NG_NACK_REASON_InvalidData;
               DEBUG_FTM("\nERROR FTM Smoke BIST Periodicity", false, 0U);
           }
           else
           {
               BURTCTimer_Start(TMR_Smoke_BIST_event_0, periodical, (uint32_t)new_Period);
           }
       }

       if(testParamsValid == true)
       {
           bistHWfaultResult =  smoke_circuit_check();

           resultData->buffer[0U] = (uint8_t)bistHWfaultResult;
           resultData->bufferLength = 1U;
           DEBUG_FTM("\nFTM Smoke BIST Periodicity", true, new_Period);
       }
   }


   return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SetSmokeChamberBISTPeriodicity
 * @details Sets the smoke chamber BIST period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SetChamberBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
    uint16_t new_Period = 0U;
    bool bistChamberResult = false;
    uint16_t modulo_check = 0U;
    bool testParamsValid = true;

    new_Period = message[0U];
    new_Period = new_Period << 8U;
    new_Period = new_Period | message[1U];
    modulo_check = new_Period % BURTC_PERIOD; /* check new period is multiples of 10 */

    if(new_Period < 2U)  /* minimum time required to carry out test */
    {
          retVal = NG_NACK_REASON_InvalidData;
          DEBUG_FTM("\nERROR FTM Smoke Chamber BIST Periodicity", false, 0U);
    }
    else
    {
         new_Period /= BURTC_PERIOD;

         if(new_Period < 1U)   /*if time is below 10sec then use LEtimer*/
         {
             LETimer_start(LETIMER_FMT_BIST_Smoke, message[1U]*1000U);
         }
         else
         {
             if(modulo_check != 0U)
             {
                 testParamsValid = false;
                 retVal = NG_NACK_REASON_InvalidData;
                 DEBUG_FTM("\nERROR FTM Smoke Chamber BIST Periodicity", false, 0U);
             }
             else
             {
                if(getIsDeviceSmokeEnable() == true)
                {
                    BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, (uint32_t)new_Period);
                }
                else
                {
                    testParamsValid = false;
                    retVal = NG_NACK_REASON_CommandNotImplemented;
                    DEBUG_FTM("\nERROR FTM Device not configured for smoke", false, 0U);
                }
             }
         }

         if(testParamsValid == true)
         {
           smoke_set_ftm_chamber_test(true);
           smoke_periodic_measurement();
           bistChamberResult = smoke_get_ftm_chamber_status();
           resultData->buffer[0U] = (uint8_t)bistChamberResult;
           resultData->bufferLength = 1U;
           DEBUG_FTM("\nFTM Smoke Chamber BIST Periodicity", true, new_Period);
         }

     }


    return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SimulatedSmokeLevel
 * @details Sets the simulated smoke level
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SimulatedLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

  smoke_set_dark_measurement(message[0U], message[1U]); /* set the smoke simulated dark values*/
  smoke_set_bright_measurement(message[2U], message[3U]); /* set the smoke simulated bright values*/

  smoke_set_ftm_active(true);
  smoke_set_ftm_chamber_test(false);
  smoke_periodic_measurement(); /*performs smoke detect */
  smoke_set_ftm_active(false);

  resultData->buffer[0U] = (uint8_t)(smoke_get_dark_measurement() >> 8U) & 0xFFU;
  resultData->buffer[1U] = (uint8_t) smoke_get_dark_measurement() & 0xFFU;
  resultData->buffer[2U] = (uint8_t)(smoke_get_bright_measurement() >> 8U) & 0xFFU;
  resultData->buffer[3U] = (uint8_t) smoke_get_bright_measurement() & 0xFFU;
  resultData->buffer[4U] = (uint8_t)(smoke_get_value() >> 8U ) & 0xFFU;
  resultData->buffer[5U] =  (uint8_t)smoke_get_value() & 0xFFU;
  resultData->buffer[6U] =  (uint8_t)getFTMSmokeState();

  resultData->bufferLength = 7U;

  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);

  DEBUG_FTM("\nFTM Smoke Simulated Level", false, 0U);
  return NG_NACK_REASON_OK;

}


/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_SetMuteStatus
 * @details Sets the smoke mute status
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_SetMuteStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  const uint8_t newState = message[0U];
  bool paramValid = true;

  switch(newState)
  {
    case 1:
      setBehavioural_Operational_State(state_Smoke_Alarm_Silence);
      LEDBuzz_Post(PatternAlarmSilence);
      DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_START, NULL);
      DEBUG_FTM("\nFTM Smoke Mute 01", false, 0U);
      break;
    case 2:
      if(b_isSmokeEventSet || super_smoke_detected() || smoke_detected())
      {
          LEDBuzz_Post(PatternAlarmSmokeStop);
          setBehavioural_Operational_State(state_Smoke_Alarm);
          LEDBuzz_Post(PatternAlarmSmoke);
      }
      else
      {
          setBehavioural_Operational_State(state_Smoke_Alarm_Silence);
          LEDBuzz_Post(PatternAlarmSmokeStop);
      }
      DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_END, NULL);
      DEBUG_FTM("\nFTM Smoke Mute 02", false, 0U);
      break;
    default:
      paramValid = false;
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM Smoke Mute Invalid Data", false, 0U);
      break;

  }

  if(paramValid == true)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;
}

/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_RunSmokeBIST
 * @details Performs smoke BIST
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

  bool bist_result = false;
  bist_result = smoke_measurement_and_bist();

  resultData->buffer[0] = (uint8_t) bist_result;
  resultData->bufferLength = 1U;

  DEBUG_FTM("\nFTM Smoke RunBIST", true, resultData->buffer[0]);

  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_StartSomkeSensitivityTest
 * @details Performs smoke sensitivity test at normal specified period level
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_StartSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

  int32_t tempVal = 0U;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool bistResult = false;

  if(temp_humid_measure_bist(&bistResult) != false)
  {
      retVal = NG_NACK_REASON_TestFailed;
      DEBUG_FTM("\nERROR FTM Smoke StartSensitivityTest", false, 0U);
  }
  else
  {
      if(getIsDeviceSmokeEnable() == true)
      {
          BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);
          smoke_set_ftm_chamber_test(false);
          smoke_periodic_measurement();

          set_ftm_sub_command(0U);
          tempVal = get_TempVal();

          resultData->buffer[0U] = (uint8_t)(smoke_get_dark_measurement() >> 8U) & 0xFFU;
          resultData->buffer[1U] = (uint8_t) smoke_get_dark_measurement() & 0xFFU;
          resultData->buffer[2U] = (uint8_t)(smoke_get_bright_measurement() >> 8U) & 0xFFU;
          resultData->buffer[3U] = (uint8_t) smoke_get_bright_measurement() & 0xFFU;
          resultData->buffer[4U] = (uint8_t)(smoke_get_value() >> 8U ) & 0xFFU;
          resultData->buffer[5U] = (uint8_t)smoke_get_value() & 0xFFU;

          if(tempVal >= 0)
          {
            resultData->buffer[6U] = (uint8_t)((tempVal & 0xFF000000u) >> 24u);
            resultData->buffer[7U] = (uint8_t)((tempVal & 0x00FF0000u) >> 16u);
            resultData->buffer[8U] = (uint8_t)((tempVal & 0x0000FF00u) >> 8u);
            resultData->buffer[9U] = (uint8_t)( tempVal & 0x000000FFu);
          }
          else
          {
              tempVal = ~tempVal;        /* 2's complement for negative val */
              tempVal = tempVal + 1;

              resultData->buffer[6U] = (uint8_t)((tempVal & 0xFF000000u) >> 24u);
              resultData->buffer[7U] = (uint8_t)((tempVal & 0x00FF0000u) >> 16u);
              resultData->buffer[8U] = (uint8_t)((tempVal & 0x0000FF00u) >> 8u);
              resultData->buffer[9U] = (uint8_t)( tempVal & 0x000000FFu);

          }

          resultData->buffer[10U] = (uint8_t)((get_HumidityVal() & 0xFF000000u) >> 24u);
          resultData->buffer[11U] = (uint8_t)((get_HumidityVal() & 0x00FF0000u) >> 16u);
          resultData->buffer[12U] = (uint8_t)((get_HumidityVal() & 0x0000FF00u) >> 8u);
          resultData->buffer[13U] = (uint8_t)( get_HumidityVal() & 0x000000FFu);
          resultData->buffer[14U] = (uint8_t) getBehavioural_Operational_State();
          resultData->buffer[15U] = (uint8_t)((get_currentTime() & 0xFF000000u) >> 24u);
          resultData->buffer[16U] = (uint8_t)((get_currentTime() & 0x00FF0000u) >> 16u);
          resultData->buffer[17U] = (uint8_t)((get_currentTime() & 0x0000FF00u) >> 8u);
          resultData->buffer[18U] = (uint8_t)( get_currentTime() & 0x000000FFu);

          resultData->bufferLength = SH_TEST_DATA_BYTES;

          /* Store result for test ReadSmokeSensitivityTestData */
          for(uint16_t i = 0U; i < resultData->bufferLength; i++)
          {
              SmokeSensitivityTestData[i] = resultData->buffer[i];
          }

          DEBUG_FTM("\nFTM Smoke StartSensitivityTest", false, 0U);
      }
      else
      {
          retVal = NG_NACK_REASON_CommandNotImplemented;
          DEBUG_FTM("\nERROR FTM Device not configured for smoke", false, 0U);
      }
  }


  return retVal;
}

/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_ReadSmokeSensitivityTestData
 * @details Performs read back of smoke sensitivity test data
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_ReadSensitivityTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

   /* Restore result for test RunSmokeSensitivityTest */
   for(uint8_t i = 0U; i < SH_TEST_DATA_BYTES; i++)
   {
       resultData->buffer[i] = SmokeSensitivityTestData[i];
   }

   resultData->bufferLength = SH_TEST_DATA_BYTES;

   DEBUG_FTM("\nFTM Smoke ReadSensitivityTestData", false, 0U);
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief nextGenComms_FTM_SmokeDetection_StopSmokeSensitivityTest
 * @details Stop smoke test
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Smoke_StopSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  b_isSmokeEventSet = false;
  smoke_set_ftm_chamber_test(false);
  (void)BURTCTimer_Stop(TMR_Smoke_measure_event_0);
  (void)BURTCTimer_Stop(TMR_Smoke_BIST_event_0);
  (void)LETimer_stop(LETIMER_FMT_Smoke);
  (void)LETimer_stop(LETIMER_FMT_BIST_Smoke);
  setBehavioural_Operational_State(state_Idle);
  FaultHandler_FaultClearAll();
  LEDBuzz_Post(PatternStopAll);

  /* Store result for test ReadSmokeSensitivityTestData */
  for(uint16_t i = 0U; i < SH_TEST_DATA_BYTES; i++)
  {
            SmokeSensitivityTestData[i] = 0U;
  }

  DEBUG_FTM("\nFTM Smoke StopSensitivityTest", false, 0U);
  return NG_NACK_REASON_OK;
}
