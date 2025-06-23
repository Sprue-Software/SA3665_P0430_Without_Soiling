
#include "P0200_FTM.h"

/*******************************************************************************
 * @brief FTM_Humidity_SetBISTState
 * @details Sets the Humid BIST state state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Humid_SetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  const uint8_t newState = message[0U];
  bool paramValid = true;

  switch(newState)
  {
    case 1U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_HUMID_OOR_START, NULL);
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(HumiditySensorOutOfBoundsFault);
      set_ftm_sub_command(newState);
      DEBUG_FTM("\nFTM Humid SetBIST State 01 ", false, 0U);
      break;
    case 2U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_HUMID_OOR_END, NULL);
      FaultHandler_FaultClear(HumiditySensorOutOfBoundsFault);
      set_ftm_sub_command(newState);
      DEBUG_FTM("\nFTM Humid SetBIST State 02 ", false, 0U);
      break;
    case 3U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_START, NULL);
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(HumiditySensorHwFault);
      DEBUG_FTM("\nFTM Humid SetBIST State 03 ", false, 0U);
      break;
    case 4U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_END, NULL);
      FaultHandler_FaultClear(HumiditySensorHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DEBUG_FTM("\nFTM Humid SetBIST State 04 ", false, 0U);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      DEBUG_FTM("\nFTM Humid SetBIST State Invalid Data ", false, 0U);
      break;
  }

  if(paramValid == true)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;

}

/*******************************************************************************
 * @brief FTM_Humidity_SimulateLevel
 * @details Sets the Humidity simulated level
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Humid_SimulateLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  if(message[0] > 100U)
  {
      DataLogging_SetEventLogbookRecord(DEF_LBE_HUMID_OOR_START, NULL);
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(HumiditySensorOutOfBoundsFault);
  }
  else
  {
      DataLogging_SetEventLogbookRecord(DEF_LBE_HUMID_OOR_END, NULL);
      FaultHandler_FaultClear(HumiditySensorOutOfBoundsFault);
  }
  set_ftm_simulate_humid(message[0U]);
  set_ftm_sub_command(0U);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  DEBUG_FTM("\nFTM Humid SimulateLevel", false, 0U);

  return NG_NACK_REASON_OK;

}

/*******************************************************************************
 * @brief FTM_Temperature_SetBISTState
 * @details Sets the Temp BIST state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Temp_SetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  const uint8_t newState = message[0U];
  bool paramValid = true;

  switch(newState)
  {
    case 1U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_START, NULL);
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(TempSensorOutOfBoundsFault);
      set_ftm_sub_command(newState);
      DEBUG_FTM("\nFTM Temp SetBIST State 01", false, 0U);
      break;
    case 2U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_END, NULL);
      FaultHandler_FaultClear(TempSensorOutOfBoundsFault);
      set_ftm_sub_command(newState);
      DEBUG_FTM("\nFTM Temp SetBIST State 02", false, 0U);
      break;
    case 3U:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_START, NULL);
      FaultHandler_FaultSet(TemperatureSensorHwFault);
      DEBUG_FTM("\nFTM Temp SetBIST State 03", false, 0U);
      break;
    case 4U:
      DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_END, NULL);
      FaultHandler_FaultClear(TemperatureSensorHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DEBUG_FTM("\nFTM Temp SetBIST State 04", false, 0U);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      DEBUG_FTM("\nERROR FTM Temp SetBIST State Invalid Data", false, 0U);
      break;
  }

  if(paramValid == true)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;


}

/*******************************************************************************
 * @brief FTM_Temperature_SimulateTemperatureValue
 * @details Sets the Temp simulate value
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Temp_SimulateTempVal(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{

  (void) messageSize;
  (void) resultData;

  if(message[0U] & 0x80)    /* check for negative numbers 2's complement*/
  {
     if(message[0U] < 0xEC)  /* check for -20deg*/
     {
         DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_START, NULL);
         FaultHandler_FaultClearAll();
         LEDBuzz_Post(PatternStopAll);
         FaultHandler_FaultSet(TempSensorOutOfBoundsFault);
     }
     else
     {
         DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_END, NULL);
         FaultHandler_FaultClear(TempSensorOutOfBoundsFault);
     }
  }
  else
  {
      if(message[0U] > 70U)  /*check for > 70deg*/
      {
          DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_START, NULL);
          FaultHandler_FaultClearAll();
          LEDBuzz_Post(PatternStopAll);
          FaultHandler_FaultSet(TempSensorOutOfBoundsFault);
      }
      else
      {
          DataLogging_SetEventLogbookRecord(DEF_LBE_TEMP_OOR_END, NULL);
          FaultHandler_FaultClear(TempSensorOutOfBoundsFault);
      }
  }
  set_ftm_sub_command(0U);
  set_ftm_simulate_temp(message[0]);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);

  return NG_NACK_REASON_OK;
}

