#include "P0200_FTM.h"


/*******************************************************************************
 * @brief FTM_Switch_SetReadPeriodicity
 * @details Sets the switch pattern read status periodicity
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Switch_SetReadPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  uint16_t new_Period = 0U;
  uint16_t modulo_check = 0U;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  new_Period = message[0U];
  new_Period = new_Period << 8U;
  new_Period = new_Period | message[1U];
  modulo_check = new_Period % BURTC_PERIOD;  /* check new period is multiples of 10 */

  set_ftm_btn_event(false);
  set_ftm_btn_pattern(false, false, false, false);


  if(modulo_check != 0U)
  {
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM Switches Invalid Data", false, 0U);
  }
  else
  {
      new_Period = new_Period / BURTC_PERIOD;
      BURTCTimer_Start(FTM_Test_Btn_event_1, periodical, new_Period);
      DEBUG_FTM("\nFTM Switches SetReadPeriodicity", false, 0U);
  }

  return retVal;
}

/*******************************************************************************
 * @brief FTM_Switch_GetStatus
 * @details Gets the status of ADS and Test button status
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Switch_GetStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  uint8_t status = 0U;
  uint8_t status_Test = (uint8_t)hal_switches_get_state(); /* bit 0 */
  uint8_t status_Ads = (uint8_t)hal_get_ads_state();  /* bit 1 */
  status = status_Ads;
  status = status << 1U;
  status = status | status_Test;
  resultData->buffer[0U] = (uint8_t)status;
  resultData->bufferLength = 1U;
  DEBUG_FTM("\nFTM Switches GetStatus", true, status);

  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Switch_GetPeriodicityResult
 * @details Gets the status of pattern switch status
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Switch_GetPeriodicityResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if(get_ftm_btn_event() == false)
  {
       retVal = NG_NACK_REASON_TestFailed;
       DEBUG_FTM("\nERROR FTM Switches GetPeriodicityResult", false, 0U);
  }
  else
  {
      set_ftm_btn_event(false);
      if(get_ftm_shortP() == true)
      {
          resultData->buffer[0U] = 0x01;
      }
      else if (get_ftm_longP() == true)
      {
          resultData->buffer[0U] = 0x02;
      }
      else if (get_ftm_longHold() == true)
      {
          resultData->buffer[0U] = 0x03;
      }
      else if (get_ftm_5shortPress() == true)
      {
          resultData->buffer[0U] = 0x04;
      }
      else
      {
          resultData->buffer[0U] = 0x00;
      }

      resultData->bufferLength = 1U;
      DEBUG_FTM("\nFTM Switches GetPeriodicityResult", true, resultData->buffer[0U]);
  }

  set_ftm_btn_pattern(false, false, false, false);

  return retVal;
}

/*******************************************************************************
 * @brief FTM_Switch_StopTest
 * @details Stops switch test
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Switch_StopTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

  (void)BURTCTimer_Stop(FTM_Test_Btn_event_1);
  set_ftm_btn_event(false);
  set_ftm_btn_pattern(false, false, false, false);
  DEBUG_FTM("\nFTM Switch Stop Test", false, 0U);

  return NG_NACK_REASON_OK;
}
