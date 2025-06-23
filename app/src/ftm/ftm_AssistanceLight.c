#include "P0200_FTM.h"


/*******************************************************************************
 * @brief FTM_AssistanceLight_SetState
 * @details Sets the assistance light state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_AssistanceLight_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if(message[0U] == 0U)
  {
      GPIO_TurnAssistanceLEDoff();


  }
  else if (message[0U] == 1U)
  {
      GPIO_TurnAssistanceLEDon();
  }
  else
  {
     retVal = NG_NACK_REASON_InvalidData;
  }

  return retVal;
}


