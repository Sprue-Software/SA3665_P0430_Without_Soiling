
#include "P0200_FTM.h"

/*******************************************************************************
 * @brief FTM_Obs_SelectSensor
 * @details Selects laser sensor (sensor1, sensor2, or sensor3) for the test
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_SelectSensor(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;

  DEBUG_FTM("\nFTM Obstacle Sensor", false, 0U);

  switch(message[0])
  {
    case 1U:
      Set_OC_Parameter(OC_Distance, 0u, 0u, 0u, 1U);
      break;
    case 2U:
      Set_OC_Parameter(OC_Distance, 0u, 0u, 0u, 2U);
      break;
    case 3U:
      Set_OC_Parameter(OC_Distance, 0u, 0u, 0u, 3U);
      break;
    default:
      paramValid = false;
      retVal = NG_NACK_REASON_InvalidData;
      break;
  }

  if(paramValid)
  {
      set_obs_det_ftm_timeover(false);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Trig_Detection);
  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Obs_SimObDis
 * @details Sets the simulated obstacle object distance
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_SimObDis(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) resultData;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if(messageSize == 2U)
  {
      uint16_t distance = 0U;
      distance = message[0];
      distance = distance << 8U;
      distance = distance | message[1U];
      set_laser_distance(distance);
  }
  else
  {
      retVal = NG_NACK_REASON_InvalidMessageLength;
  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Obs_SetObDetPeriod
 * @details Sets the obstacle/coverage detection period
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_SetObDetPeriod(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

    (void) messageSize;
    (void) resultData;


    uint16_t new_Period = 0U;
    uint16_t modulo_check = 0U;
    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

    new_Period = message[0U];
    new_Period = new_Period << 8U;
    new_Period = new_Period | message[1U];

    modulo_check = new_Period % BURTC_PERIOD;  /* check new period is multiples of 10 */
    if(modulo_check != 0U)
    {
        retVal = NG_NACK_REASON_InvalidData;
        DEBUG_FTM("\nERROR FTM Obstacle Set Periodicity Invalid Data", false, 0U);
    }
    else
    {
        new_Period = new_Period / BURTC_PERIOD;
        BURTCTimer_Start(TMR_Obstacle_Coverage_BIST_event_0, periodical, new_Period);
        DEBUG_FTM("\nFTM Obstacle Set Periodicity", true, new_Period);
    }

   return retVal;
}


/*******************************************************************************
 * @brief FTM_Obs_SetObDetBISTState
 * @details Sets the obstacle detection BIST state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_SetObDetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;

  switch(message[0])
  {
    case 1U:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(ObstacleDetectedFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_OBSTACLE_DET_START, NULL);
      DEBUG_FTM("\nFTM Obstacle Set State 01", false, 0U);
      break;
    case 2U:
      FaultHandler_FaultClear(ObstacleDetectedFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_OBSTACLE_DET_END, NULL);
      DEBUG_FTM("\nFTM Obstacle Set State 02", false, 0U);
      break;
    case 3U:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(ObstacleDetectionHwFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_OBSTACLE_DET_HW_ERR_START, NULL);
      DEBUG_FTM("\nFTM Obstacle Set State 03", false, 0U);
      break;
    case 4U:
      FaultHandler_FaultClear(ObstacleDetectionHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_OBSTACLE_DET_HW_ERR_END, NULL);
      DEBUG_FTM("\nFTM Obstacle Set State 04", false, 0U);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      DEBUG_FTM("\nERROR FTM Obstacle Set State Invalid Data", false, 0U);
      break;
  }

  if(paramValid == true)
  {
       SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;
}


/*******************************************************************************
 * @brief FTM_Obs_SetCoDetBISTState
 * @details Sets the coverage detection BIST state
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_SetCoDetBISTState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){

   (void) messageSize;
   (void) resultData;

   nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
   bool paramValid = true;

   switch(message[0])
   {
     case 1U:
       FaultHandler_FaultClearAll();
       LEDBuzz_Post(PatternStopAll);
       FaultHandler_FaultSet(CoverageDetectedFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_COVERAGE_DET_START, NULL);
       DEBUG_FTM("\nFTM Coverage Set State 01", false, 0U);
       break;
     case 2U:
       FaultHandler_FaultClear(CoverageDetectedFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_COVERAGE_DET_END, NULL);
       DEBUG_FTM("\nFTM Coverage Set State 02", false, 0U);
       break;
     case 3U:
       FaultHandler_FaultClearAll();
       LEDBuzz_Post(PatternStopAll);
       FaultHandler_FaultSet(ObstacleDetectionHwFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_COVERAGE_HW_ERR_START, NULL);
       DEBUG_FTM("\nFTM Coverage Set State 03", false, 0U);
       break;
     case 4U:
       FaultHandler_FaultClear(ObstacleDetectionHwFault);
       LEDBuzz_Post(PatternStopMajorFault);
       DataLogging_SetEventLogbookRecord(DEF_LBE_COVERAGE_HW_ERR_END, NULL);
       DEBUG_FTM("\nFTM Coverage Set State 04", false, 0U);
       break;
     default:
       retVal = NG_NACK_REASON_InvalidData;
       DEBUG_FTM("\nERROR FTM Coverage Set State Invalid Data", false, 0U);
       break;
   }

   if(paramValid == true)
   {
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
   }

   return retVal;
}

nextGenCommsAckNackReason_t FTM_Obs_GetObsDetResults(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
   (void) (message);
   (void) messageSize;

   LASER_TEST laser_ftm_result;
   nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

   if(get_obs_det_ftm_period_timeover() == false)
   {
       retVal = NG_NACK_REASON_TestNotFinished;
       DEBUG_FTM("\FTM obs GetPeriodicityResult not yet finished", false, 0U);
   }
   else
   {
       laser_ftm_result = get_laser_status();
       resultData->buffer[0U] = 0x00U;
       if(laser_ftm_result == 0x04)
       {
           laser_ftm_result = 0xFFU;
       }
       resultData->buffer[1U] = (uint8_t)laser_ftm_result;
       resultData->bufferLength = 2U;
       DEBUG_FTM("\nFTM obs GetPeriodicityResult", true, laser_ftm_result);
   }

   return retVal;
}

/*******************************************************************************
 * @brief FTM_Obs_StopTests
 * @details Stops the laser tests
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Obs_StopTests(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData){
  (void)message;
  (void)messageSize;
  (void)resultData;

  DEBUG_FTM("\nFTM Obstacle StopTests", false, 0U);
  (void)BURTCTimer_Stop(TMR_Obstacle_Coverage_BIST_event_0);
  set_obs_det_ftm_timeover(false);
  set_obs_det_ftm_period_timeover(false);
  return NG_NACK_REASON_OK;
}

nextGenCommsAckNackReason_t FTM_Obs_SensorResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void)message;
  (void)messageSize;

  uint16_t distance = 0U;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if(get_obs_det_ftm_timeover() == false)
  {
        retVal = NG_NACK_REASON_TestNotFinished;
        DEBUG_FTM("\nFTM obs sensor distance test not yet finished", false, 0U);
  }
  else
  {
      set_obs_det_ftm_timeover(false);
      distance = get_laser_distance();
      resultData->buffer[0U] = 0x00U;
      resultData->buffer[1U] = (uint8_t) (distance >> 8) & 0xFFU;  //MSB
      resultData->buffer[2U] = (uint8_t) distance & 0xFFU;         //LSB
      resultData->bufferLength = 3U;
      DEBUG_FTM("\nFTM obs sensor result", true, distance);
  }

  return retVal;
}
nextGenCommsAckNackReason_t FTM_Obs_RunBist(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void)message;
  (void)messageSize;
  (void)resultData;
  set_obs_det_ftm_timeover(false);
  Set_OC_Parameter(OC_BIST, 0u, 0u, 0u, 0u);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Trig_Detection);
  DEBUG_FTM("\nFTM Obs Run Bist", false, 0u);

  return NG_NACK_REASON_OK;
}

nextGenCommsAckNackReason_t FTM_Obs_BistResult(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void)message;
  (void)messageSize;
  (void)resultData;

  LASER_TEST laser_result;
  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;

  if(get_obs_det_ftm_timeover() == false)
  {
       retVal = NG_NACK_REASON_TestNotFinished;
       DEBUG_FTM("\nFTM Bist Result test not yet finished", false, 0U);
  }
  else
  {
       set_obs_det_ftm_timeover(false);
       laser_result = get_laser_status();
       resultData->buffer[0U] = 0x00U;
       if(laser_result != LASER_SENSOR_FAILUARE)
       {
           laser_result = 0x00U;
       }
       else
       {
           laser_result = 0xFFU;
       }
       resultData->buffer[1U] = (uint8_t)laser_result;
       resultData->bufferLength = 2U;

       DEBUG_FTM("\nFTM Bist Result", true, laser_result);
  }

  return retVal;
}
