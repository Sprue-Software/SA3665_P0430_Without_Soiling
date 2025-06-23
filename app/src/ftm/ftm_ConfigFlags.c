#include "P0200_FTM.h"


/*******************************************************************************
 * @brief FTM_SystemConfigFlags
 * @details Sets system configuration flags to new values
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_SystemConfigFlags(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;

  behaviour_state_enum_System_modes current_system_mode;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;

  switch(message[0])
  {
    case 0U:
      setBehavioural_System_Modes(Standby_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Standby_Mode)
      {
          retVal = NG_NACK_REASON_TestFailed;
          resultData->buffer[0U] = 0xFF;
      }
      else
      {
          resultData->buffer[0U] = (uint8_t)Standby_Mode;
          DEBUG_FTM("\nFTM SM", false, 0U);
      }
      break;
    case 1U:
      setBehavioural_System_Modes(Commisioning_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Commisioning_Mode)
      {
          retVal = NG_NACK_REASON_TestFailed;
          resultData->buffer[0U] = 0xFF;
      }
      else
      {
          resultData->buffer[0U] = (uint8_t)Commisioning_Mode;
          DEBUG_FTM("\nFTM CM", false, 0U);
      }
      break;
    case 2U:
      setBehavioural_System_Modes(Operational_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Operational_Mode)
      {
          retVal = NG_NACK_REASON_TestFailed;
          resultData->buffer[0U] = 0xFF;
      }
      else
      {
          resultData->buffer[0U] = (uint8_t)Operational_Mode;
          DEBUG_FTM("\nFTM OM", false, 0U);
      }
      break;
    case 3U:
      setBehavioural_System_Modes(Functional_Test_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Functional_Test_Mode)
      {
          retVal = NG_NACK_REASON_TestFailed;
          resultData->buffer[0U] = 0xFF;
      }
      else
      {
          resultData->buffer[0U] = (uint8_t)Functional_Test_Mode;
          DEBUG_FTM("\nFTM ftm", false, 0U);
      }
      break;
    case 4U:
      setBehavioural_System_Modes(Transport_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Transport_Mode)
      {
          retVal = NG_NACK_REASON_TestFailed;
          resultData->buffer[0U] = 0xFF;
      }
      else
      {
          resultData->buffer[0U] = (uint8_t)Transport_Mode;
          DEBUG_FTM("\nFTM TM", false, 0U);
      }
      break;
    case 5U:
      setBehavioural_System_Modes(Shutdown_Mode);
      current_system_mode = getBehavioural_System_Modes(false);
      if(current_system_mode != Shutdown_Mode)
      {
           retVal = NG_NACK_REASON_TestFailed;
           resultData->buffer[0U] = 0xFF;
      }
      else
      {
           resultData->buffer[0U] = (uint8_t)Shutdown_Mode;
           DEBUG_FTM("\nFTM ShutM", false, 0U);
      }
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM Invalid Data", false, 0U);
      paramValid = false;
      break;
  }

  if(paramValid)
  {
      resultData->bufferLength = 1U;
  }
  return retVal;
}


/*******************************************************************************
 * @brief FTM_DeviceConfigFlags
 * @details Sets the alarm type selection configuration flags to new values
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_DeviceConfigFlags(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  return NG_NACK_REASON_OK;
}


