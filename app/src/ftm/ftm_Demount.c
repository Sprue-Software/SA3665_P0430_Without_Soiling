#include "P0200_FTM.h"

/*******************************************************************************
 * @brief nextGenComms_FTM_DemountDetection_ChangeStatus
 * @details Changing the demount state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Demount_ChangeStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  const uint8_t newState = message[0U];
  bool paramValid = true;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

   switch(newState)
   {
     case 1U:
       DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_START, NULL);
       DEBUG_FTM("\nFTM Demount Set State 01", false, 0U);
       break;
     case 2U:
       FaultHandler_FaultClearAll();
       LEDBuzz_Post(PatternStopAll);
       FaultHandler_FaultSet(DemountedTooLongFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_TOO_LONG_START, NULL);
       DEBUG_FTM("\nFTM Demount Set State 02", false, 0U);
       break;
     case 3U:
       DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_END, NULL);
       DEBUG_FTM("\nFTM Demount Set State 03", false, 0U);
       break;
     case 4U:
       FaultHandler_FaultClearAll();
       LEDBuzz_Post(PatternStopAll);
       FaultHandler_FaultSet(DemountingDetectionHwFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_DET_HW_ERR_START, NULL);
       DEBUG_FTM("\nFTM Demount Set State 04", false, 0U);
       break;
     case 5U:
       FaultHandler_FaultClear(DemountingDetectionHwFault);
       LEDBuzz_Post(PatternStopMajorFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_DET_HW_ERR_END, NULL);
       DEBUG_FTM("\nFTM Demount Set State 05", false, 0U);
       break;
     default:
       retVal = NG_NACK_REASON_InvalidData;
       paramValid = false;
       DEBUG_FTM("\nERROR FTM Demount Invalid Data", false, 0U);
       break;
   }

   if(paramValid == true)
   {
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
   }

   return retVal;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_DemountDetection_SimulateState
 * @details Simulating the demount state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Demount_SimulateState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if((message[0U] == 0U) || (message[0U] == 1U))
  {
      hal_set_ads_state(message[0U]);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }
  else
  {
      retVal = NG_NACK_REASON_InvalidData;
  }
  return retVal;
}

