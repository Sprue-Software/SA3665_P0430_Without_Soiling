

#include "P0200_FTM.h"
#include "ambient_light.h"

/*******************************************************************************
 * @brief nextGenComms_FTM_EEPROM_SetBISTPeriodicity
 * @details Sets the EEPROM CRC checking period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_AmbientLight_SetStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  uint8_t ambient_Status = message[0U];

  if((ambient_Status == 0U) || (ambient_Status == 1U))
  {
      const bool perform_measurement = true;

      if(ambient_light_get_BIST(perform_measurement) == false)
      {
          retVal = NG_NACK_REASON_TestFailed;
          DEBUG_FTM("\nERROR FTM Ambient Light BIST Failed", false, 0U);
      }
      else
      {
          ambient_light_set_status(ambient_Status);
          SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
          DEBUG_FTM("\nFTM Ambient Light", false, 0U);
      }
  }
  else
  {
      retVal = NG_NACK_REASON_InvalidData;
      DEBUG_FTM("\nERROR FTM Ambient Light Invalid Data", false, 0U);
  }

  return retVal;
}
