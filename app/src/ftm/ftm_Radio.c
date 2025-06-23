
#include "P0200_FTM.h"

/*******************************************************************************
 * @brief FTM_Radio_Test
 * @details Performs radio test
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Radio_Test(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{

  (void) messageSize;
  (void) resultData;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;
  switch(message[0])
  {
    case 1U:
    case 2U:
    case 3U:
    case 4U:
    case 5U:
      set_radio_test(message[0]);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      break;
  }

  if(paramValid)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Radio_test);
  }
  return retVal;
}

