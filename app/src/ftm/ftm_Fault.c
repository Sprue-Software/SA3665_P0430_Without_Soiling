

#include "P0200_FTM.h"

static uint32_t fault_status = 0U;

bool isFaultHandlerBitSet(uint32_t fault_val, uint8_t bit_pos)
{
  bool retVal = false;

     if (fault_val & (1U << bit_pos))
     {
         retVal = true;
     }

     return retVal;
}

void faultFlagsRestore(void)
{
  RTOS_ERR err;
  for(uint8_t fault = 0U; fault < 32U; fault++)
    {
       if(isFaultHandlerBitSet(getFaultFlagStatus(), fault))
       {
           FaultHandler_FaultSet(fault);
          OSTimeDly(2, OS_OPT_TIME_DLY, &err);
       }
    }
}

void setFaultFlagStatus(uint32_t fault_val)
{
  fault_status = fault_val;
}

uint32_t getFaultFlagStatus(void)
{
   return fault_status;
}

/*******************************************************************************
 * @brief Reads faults code
 * @details reads faults code from fault handler
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Fault_Read(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

  uint32_t fault_status = FaultHandler_GetFaultFlags();
  resultData->buffer[0U] = (uint8_t) ((fault_status >> 24U) & 0xFFU);
  resultData->buffer[1U] = (uint8_t) ((fault_status >> 16U) & 0xFFU);
  resultData->buffer[2U] = (uint8_t) ((fault_status >> 8U) & 0xFFU);
  resultData->buffer[3U] = (uint8_t) (fault_status & 0xFFU);
  resultData->bufferLength = 4U;
  DEBUG_FTM("\nFault Status", true, fault_status);

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  return retVal;
}


nextGenCommsAckNackReason_t FTM_Fault_Clear_All(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

  FaultHandler_FaultClearAll();
  LEDBuzz_Post(PatternStopAll);

  return NG_NACK_REASON_OK;
}


nextGenCommsAckNackReason_t FTM_Fault_Restore(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

  faultFlagsRestore();

  return NG_NACK_REASON_OK;
}
