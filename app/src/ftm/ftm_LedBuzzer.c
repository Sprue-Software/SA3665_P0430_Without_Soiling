
#include "P0200_FTM.h"
#include "diagnostics.h"
#include "hal_Timer0.h"

/*******************************************************************************
 * @brief FTM_Humidity_SetBISTState
 * @details Sets the Humid BIST state state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Buzzer_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;

  switch(message[0])
  {
    case 3U:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(BuzzerHwFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_BUZZER_CHECK_HW_ERR_START, NULL);
      DEBUG_FTM("\nFTM LEDBUZZ Set State 03", false, 0U);
      break;
    case 4U:
      FaultHandler_FaultClear(BuzzerHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_BUZZER_CHECK_HW_ERR_END, NULL);
      DEBUG_FTM("\nFTM LEDBUZZ Set State 04", false, 0U);
      break;
    default:
      paramValid = false;
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM LEDBUZZ Set State Invalid Data", false, 0U);
      break;
  }

  if(paramValid == true)
   {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
   }

  return retVal;

}
nextGenCommsAckNackReason_t FTM_Buzzer_StartTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  switch(message[0])
  {
      case 1U:
        PWM_Timer0_Stop();
        LEDBuzz_BuzzerTurnOnHighFreq();
        break;
      case 2U:
        LEDBuzz_BuzzerTurnOffHighFreq();
        break;
      case 3U:
        LEDBuzz_BuzzerTurnOnLowFreq();
        PWM_Timer0_Start(3000U, 0.5);
        break;
      case 4U:
        LEDBuzz_BuzzerTurnOffLowFreq();
        break;
      default:
        retVal = NG_NACK_REASON_InvalidData;
        break;
    }


  return retVal;
}
nextGenCommsAckNackReason_t FTM_Buzzer_GetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

  return NG_NACK_REASON_OK;

}

nextGenCommsAckNackReason_t FTM_Buzzer_RunBist(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  uint8_t f_error = 0U;

  f_error = runBuzzerBist();
  if(f_error == 0U)
  {
        resultData->buffer[0U] = 0U;
  }
  else
  {
      resultData->buffer[0U] = 1U;
  }

  resultData->bufferLength = 1U;
  return NG_NACK_REASON_OK;

}

nextGenCommsAckNackReason_t FTM_Buzzer_SoundOutputTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
    (void) messageSize;
    (void) resultData;

    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

    switch(message[0])
    {
        case 1U:
          PWM_Timer0_Stop();
          LEDBuzz_BuzzerTurnOnHighFreq();
          break;
        case 2U:
          LEDBuzz_SetFTMPulseTone(true);
          LEDBuzz_Post(PatternAlarmSmoke);
          break;
        case 3U:
          LEDBuzz_BuzzerTurnOnLowFreq();
          PWM_Timer0_Start(3000U, 0.5);
          break;
        case 4U:
          LEDBuzz_BuzzerTurnOffHighFreq();
          LEDBuzz_BuzzerTurnOffLowFreq();
          LEDBuzz_Post(PatternStopAll);
          LEDBuzz_SetFTMPulseTone(false);
          break;
        default:
          retVal = NG_NACK_REASON_InvalidData;
          break;
      }


    return retVal;
}


nextGenCommsAckNackReason_t FTM_LED_GetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  return retVal;

}
nextGenCommsAckNackReason_t FTM_LED_StartTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
    (void) message;
    (void) messageSize;
    (void) resultData;
    LEDBuzz_Post(PatternExtendedUserTestPass);
    return  NG_NACK_REASON_OK;

}
