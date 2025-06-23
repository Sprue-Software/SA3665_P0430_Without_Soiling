#include "P0200_FTM.h"
#include "eeprom_handler.h"
#include "hal_i2c.h"
#include "comms_handler.h"
#include <stdlib.h>

#define READ_LEN        (16U)
#define ROW_LEN         (16U) // in bytes
#define ROW_NUMBER_MAX  (256U) // 256 * 16 = 4096 total eeprom length
#define EEPROM_STARTING_OFFSET (0x0000u)
static bool isXmodemEnable = false;


/*******************************************************************************
 * @brief FTM_EEPROM_StopTest
 * @details Exits XMODEM mode
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EEPROM_StopTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;
  setXModemEnable(false);
  xModemModeExit();
  DEBUG_FTM("\nFTM EP Exit XModem ", false, 0U);
  return NG_NACK_REASON_OK;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_EEPROM_SimulateCorruption
 * @details Sets the simulated corrupted EEPROM CRC value
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EEPROM_SimulateCorruption(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;

  return NG_NACK_REASON_OK;
}

static bool xModemCallback_EEPROM(const uint8_t packetId, uint8_t data[128u])
{
  I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* enable the power to eeprom */
  EEPROM_Read(data, (packetId - 1U) * 128U, 128U);
  I2C_BusRelease(); /* Release I2C Bus */
  return packetId < (EEPROM_SIZE / 128U);
}
/*******************************************************************************
 * @brief nextGenComms_FTM_EEPROM_DownloadData
 * @details Downloads the EEPROM content
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EEPROM_DownloadData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void) message;
  (void) messageSize;
  (void) resultData;
  setXModemEnable(true);
  DEBUG_FTM("\nFTM EP DownLoadData", false, 0U);
  return xModemModeEnter(xModemCallback_EEPROM) ? NG_NACK_REASON_OK : NG_NACK_REASON_InternalError;
}


/*******************************************************************************
 * @brief nextGenComms_FTM_EEPROM_ClearEEPROm
 * @details Clears a specific EEPROM area
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EEPROM_ClearEEPROM(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) message;
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  const bool ok = DataLoggingErase(0u);

  if( ok )
  {

  }
  else
  {
    retVal = NG_NACK_REASON_TestFailed;
  }

  return retVal;
}

/*******************************************************************************
 * @brief nextGenComms_FTM_EEPROM_ReadRow
 * @details Reads one full row from eeprom. row number is an input from user
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_EEPROM_ReadRow(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) message;
  (void) messageSize;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  uint8_t eepromDataBuffer[READ_LEN]={0u, };
  bool read_status = false;

  uint16_t rowNumber = (((uint16_t)message[0] << 8u) | message[1]);
  const uint32_t address = (EEPROM_STARTING_OFFSET + (rowNumber-1)*ROW_LEN);

  /* static area size is 256 bytes, each row is 16 bytes.
   * Hence number of rows = 256 /16 = 16 rows
   */
  if ((0u < rowNumber) && (rowNumber <= ROW_NUMBER_MAX))
  {
      I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* enable the power to eeprom */
      /* read 1 row of data from static area of eeprom */
      read_status = EEPROM_Read(eepromDataBuffer, address, READ_LEN);
      I2C_BusRelease(); /* Release I2C Bus */

      if (read_status != true)
      {
          retVal = NG_NACK_REASON_TestFailed;
      }
      else
      {
          for(uint8_t i = 0U; i < READ_LEN; i++)
          {
              resultData->buffer[i] = eepromDataBuffer[i];
          }
          resultData->bufferLength = READ_LEN;
      }
  }
  else
  {
      retVal = NG_NACK_REASON_InvalidData;
  }


  return retVal;
}

void setXModemEnable(bool enable)
{
  isXmodemEnable = enable;
}

bool getXModemEnable(void)
{
  return isXmodemEnable;
}
