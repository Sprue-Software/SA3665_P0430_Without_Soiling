#include "P0200_FTM.h"

static bool incTimerRate = false;

/*******************************************************************************
 * @brief FTM_Timers_SetSystemTime
 * @details Sets the system time
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Timers_SetSystemTime(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) messageSize;
  (void) resultData;

  uint32_t currTime = 0U;
  currTime = (message[0] << 24) | (message[1] << 16) | (message[2] << 8) | message[3];
  time_setCurrentTime(currTime);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Date_Time);
  DEBUG_FTM("\nFTM Timers Set System Time", true, currTime);
  return NG_NACK_REASON_OK;
}

/*******************************************************************************
 * @brief FTM_Timers_SetSystemTime
 * @details Sets the system time
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Timers_GetSystemTime(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;

  uint32_t currTime = get_currentTime();
  resultData->buffer[0U] = (uint8_t)((currTime >> 24U) & 0XFFU);
  resultData->buffer[1U] = (uint8_t)((currTime >> 16U) & 0XFFU);
  resultData->buffer[2U] = (uint8_t)((currTime >> 8U) & 0XFFU);
  resultData->buffer[3U] = (uint8_t)(currTime & 0XFFU);
  resultData->bufferLength = 4U;
  DEBUG_FTM("\nFTM Timers Get System Time", true, currTime);
  return NG_NACK_REASON_OK;
}


/*******************************************************************************
 * @brief FTM_Timers_ResetSystemTime
 * @details Resets the BURTC to default rate
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Timers_ResetRate(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;
  BURTC_reinit(10U);
  return NG_NACK_REASON_OK;
}


/*******************************************************************************
 * @brief FTM_Timers_IncreaseRate
 * @details Set the BURTC system timer rate
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Timers_IncreaseRate(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  setIncresedTimerRate(true);

  switch (message[0U])
  {
    case 10U:
    case 8U:
    case 5U:
    case 4U:
    case 2U:
    case 1U:
      BURTC_reinit(message[0U]);
      DEBUG_FTM("\nFTM Timers Increase Rate", true, message[0U]);
      break;
    default:
      setIncresedTimerRate(false);
      DEBUG_FTM("\nERROR FTM Timers Increase Rate Invalid Data", false, 0U);
      retVal = NG_NACK_REASON_InvalidData;
      break;

  }

  return retVal;
}

void setIncresedTimerRate(bool enable)
{
  incTimerRate = enable;
}
bool getIncresedTimerRate()
{
  return incTimerRate;
}

