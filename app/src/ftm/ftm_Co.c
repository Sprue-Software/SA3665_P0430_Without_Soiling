#include "P0200_FTM.h"

static uint8_t CoSensitivityTestData[CO_TEST_DATA_BYTES];
static bool b_isCoEventSet = false;

nextGenCommsAckNackReason_t FTM_CO_SetState(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
  bool paramValid = true;

  switch(message[0U])
  {
    case 1:
      setBehavioural_Operational_State(state_CO_Alarm);
      LEDBuzz_Post(PatternAlarmCO);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      DataLogging_SetCOEvent((uint8_t) EVENT_TYPE_LOCAL);
      DataLogging_SetEventLogbookRecord(DEF_LBE_CO_DET_START, NULL);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Countersdates);
      b_isCoEventSet = true;
      DEBUG_FTM("\nFTM Co Set State 01", false, 0U);
      break;
    case 2:
      setBehavioural_Operational_State(state_Idle);
      LEDBuzz_Post(PatternAlarmCOStop);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      DataLogging_SetEventLogbookRecord(DEF_LBE_CO_DET_END, NULL);
      b_isCoEventSet = false;
      DEBUG_FTM("\nFTM Co Set State 02", false, 0U);

      break;
    case 3:
      FaultHandler_FaultClearAll();
      LEDBuzz_Post(PatternStopAll);
      FaultHandler_FaultSet(COSenserHwFault);
      DataLogging_SetEventLogbookRecord(DEF_LBE_CO_DET_HW_ERR_START, NULL);
      DEBUG_FTM("\nFTM Co Set State 03", false, 0U);
      break;
    case 4:
      FaultHandler_FaultClear(COSenserHwFault);
      LEDBuzz_Post(PatternStopMajorFault);
      DEBUG_FTM("\nFTM Co Set State 04", false, 0U);
      break;
    default:
      retVal = NG_NACK_REASON_InvalidData;
      paramValid = false;
      DEBUG_FTM("\nERROR FTM Co Set State Invalid Data", false, 0U);
      break;

  }

  if(paramValid)
  {
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  }

  return retVal;
}

nextGenCommsAckNackReason_t FTM_CO_SetBISTPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
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
       DEBUG_FTM("\nERROR FTM CO Bist Periodicity Invalid Data", false, 0U);
   }
   else
   {
       new_Period = new_Period / BURTC_PERIOD;
       BURTCTimer_Start(TMR_CO_BIST_event_0, periodical, new_Period);

       uint8_t co_hw_status = 0U;
       set_ftm_strike_count();
       acquisitionCO(true);
       restore_co_bist_strike_count();
       co_hw_status = (uint8_t) getFTM_CO_HW_Fault();
       if(co_hw_status == 0U)
       {
           resultData->buffer[0U] = 0x00;
       }
       else
       {
           resultData->buffer[0U] = 0x01;
       }

       resultData->bufferLength = 1U;
       DEBUG_FTM("\nFTM CO BIST Result = ", true, resultData->buffer[0U]);

   }

   return retVal;
}

nextGenCommsAckNackReason_t FTM_CO_SetMeasurementPeriodicity(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{

  (void) messageSize;

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
        DEBUG_FTM("\nERROR FTM CO Mesurement Periodicity Invalid Data", false, 0U);
    }
    else
    {
        new_Period = new_Period / BURTC_PERIOD;
        BURTCTimer_Start(TMR_CO_measure_event_0, periodical, new_Period);

        acquisitionCO(false);
        uint16_t co_data = 0U;
        co_data = getFTM_CO_RawData();
        resultData->buffer[0U] = (uint8_t) ((co_data >> 8U) & 0xFFU);
        resultData->buffer[1U] = (uint8_t) (co_data & 0xFFU);

        co_data = getFTM_CO_AfterCompData();
        resultData->buffer[2U] = (uint8_t) ((co_data >> 8U) & 0xFFU);
        resultData->buffer[3U] = (uint8_t) (co_data & 0xFFU);

        co_data = (uint8_t) get_CO_Level();

        if(co_data == 0U)
        {
            resultData->buffer[4U] = 0x00;
        }
        else if(co_data == 2U)
        {
            resultData->buffer[4U] = 0x01;
        }
        else
        {
            resultData->buffer[4U] = 0x02;
        }

        resultData->bufferLength = 5U;

        DEBUG_FTM("\nFTM CO Measurement Periodicity",false, 0U);

    }

    return retVal;
}

nextGenCommsAckNackReason_t FTM_CO_SetMuteStatus(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{

  (void) messageSize;
  (void) resultData;

    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
    const uint8_t newState = message[0U];
    bool paramValid = true;

    switch(newState)
    {
      case 1:
        setBehavioural_Operational_State(state_CO_Alarm_Silence);
        LEDBuzz_Post(PatternAlarmSilence);
        DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_START, NULL);
        DEBUG_FTM("\nFTM CO Mute 01", false, 0U);
        break;
      case 2:
        if(b_isCoEventSet || (0U != (uint8_t) get_CO_Level()))
        {
            LEDBuzz_Post(PatternAlarmCOStop);
            setBehavioural_Operational_State(state_CO_Alarm);
            LEDBuzz_Post(PatternAlarmCO);
        }
        else
        {
            setBehavioural_Operational_State(state_Smoke_Alarm_Silence);
            LEDBuzz_Post(PatternAlarmCOStop);
        }
        DataLogging_SetEventLogbookRecord(DEF_LBE_ALARM_MUTED_END, NULL);
        DEBUG_FTM("\nFTM Co Mute 02", false, 0U);
        break;
      default:
        paramValid = false;
        retVal = NG_NACK_REASON_InvalidData;
        DEBUG_FTM("\nERROR FTM Co Mute Invalid Data", false, 0U);
        break;

    }

    if(paramValid)
    {
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
    }

    return retVal;
}


nextGenCommsAckNackReason_t FTM_CO_SimulatedCOLevel(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) messageSize;
  (void) resultData;

  /* Get simulated PPM value */
  const uint16_t simulated_raw_ppm_reading = ( ( uint16_t )message[ 0U ] << 8 ) | ( uint16_t )message[ 1U ];
  /* Set the ppm value to be used by co aquisition */
  acqco_simulated_ppm_co_reading( simulated_raw_ppm_reading );
  /* Perform a reading with no co diags */
  acquisitionCO( false );
  /* Return co aquisition to obtain the calculated */
  acqco_simulate_raw_co_reading( false );
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
  DEBUG_FTM("\nFTM Smoke Simulated Level", false, 0U);
  /* No data returned */
  resultData->bufferLength = 0U;
  return NG_NACK_REASON_OK;
}


nextGenCommsAckNackReason_t FTM_CO_RunBIST(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
    (void) message;
    (void) messageSize;

    set_ftm_strike_count();
    uint8_t co_hw_status = 0U;
    acquisitionCO(true);
    restore_co_bist_strike_count();
    co_hw_status = (uint8_t) getFTM_CO_HW_Fault();
    if(co_hw_status == 0U)
    {
             resultData->buffer[0U] = 0x00;
    }
    else
    {
             resultData->buffer[0U] = 0x01;
    }

    resultData->bufferLength = 1U;
    DEBUG_FTM("\nFTM CO RUN BIST Result = ", true, resultData->buffer[0U]);


    return NG_NACK_REASON_OK;
}


nextGenCommsAckNackReason_t FTM_CO_RunSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

    int32_t tempVal = 0U;
    nextGenCommsAckNackReason_t retVal = NG_NACK_REASON_OK;
    bool bistResult = false;

    if(temp_humid_measure_bist(&bistResult) != false)
    {
        retVal = NG_NACK_REASON_TestFailed;
        DEBUG_FTM("\nERROR FTM Co StartSensitivityTest", false, 0U);
    }
    else
    {

        BURTCTimer_Start(TMR_CO_measure_event_0, periodical, CO_MEASUREMENT_PERIOD);
        acquisitionCO(false);

        set_ftm_sub_command(0U);
        tempVal = get_TempVal();

        uint16_t co_data = 0U;
        co_data = getFTM_CO_RawData();
        resultData->buffer[0U] = (uint8_t) ((co_data >> 8U) & 0xFFU);
        resultData->buffer[1U] = (uint8_t) (co_data & 0xFFU);

        co_data = getFTM_CO_AfterCompData();
        resultData->buffer[2U] = (uint8_t) ((co_data >> 8U) & 0xFFU);
        resultData->buffer[3U] = (uint8_t) (co_data & 0xFFU);

        if(tempVal >= 0)
        {
            resultData->buffer[4U] = (uint8_t)((tempVal & 0xFF000000u) >> 24u);
            resultData->buffer[5U] = (uint8_t)((tempVal & 0x00FF0000u) >> 16u);
            resultData->buffer[6U] = (uint8_t)((tempVal & 0x0000FF00u) >> 8u);
            resultData->buffer[7U] = (uint8_t)( tempVal & 0x000000FFu);
        }
        else
        {
            tempVal = ~tempVal;
            tempVal = tempVal + 1;  /* 2's complement for negative val */

            resultData->buffer[4U] = (uint8_t)((tempVal & 0xFF000000u) >> 24u);
            resultData->buffer[5U] = (uint8_t)((tempVal & 0x00FF0000u) >> 16u);
            resultData->buffer[6U] = (uint8_t)((tempVal & 0x0000FF00u) >> 8u);
            resultData->buffer[7U] = (uint8_t)( tempVal & 0x000000FFu);
        }

        resultData->buffer[8U] = (uint8_t)((get_HumidityVal() & 0xFF000000u) >> 24u);
        resultData->buffer[9U] = (uint8_t)((get_HumidityVal() & 0x00FF0000u) >> 16u);
        resultData->buffer[10U] = (uint8_t)((get_HumidityVal() & 0x0000FF00u) >> 8u);
        resultData->buffer[11U] = (uint8_t)( get_HumidityVal() & 0x000000FFu);
        resultData->buffer[12U] = (uint8_t) getBehavioural_Operational_State();
        resultData->buffer[13U] = (uint8_t)((get_currentTime() & 0xFF000000u) >> 24u);
        resultData->buffer[14U] = (uint8_t)((get_currentTime() & 0x00FF0000u) >> 16u);
        resultData->buffer[15U] = (uint8_t)((get_currentTime() & 0x0000FF00u) >> 8u);
        resultData->buffer[16U] = (uint8_t)( get_currentTime() & 0x000000FFu);

        resultData->bufferLength = CO_TEST_DATA_BYTES;

        /* Store result for test ReadSmokeSensitivityTestData */
        for(uint16_t i = 0U; i < resultData->bufferLength; i++)
        {
            CoSensitivityTestData[i] = resultData->buffer[i];
        }

        DEBUG_FTM("\nFTM Co StartSensitivityTest", false, 0U);
    }


    return retVal;
}



nextGenCommsAckNackReason_t FTM_CO_ReadSensitivityTestData(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;

    /* Restore result for test RunSmokeSensitivityTest */
    for(uint8_t i = 0U; i < CO_TEST_DATA_BYTES; i++)
    {
        resultData->buffer[i] = CoSensitivityTestData[i];
    }

    resultData->bufferLength = CO_TEST_DATA_BYTES;

    DEBUG_FTM("\nFTM Co ReadSensitivityTestData", false, 0U);
    return NG_NACK_REASON_OK;
}

nextGenCommsAckNackReason_t FTM_CO_StopSensitivityTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;

    b_isCoEventSet = false;
    (void)BURTCTimer_Stop(TMR_CO_BIST_event_0);
    (void)BURTCTimer_Stop(TMR_CO_measure_event_0);
    setBehavioural_Operational_State(state_Idle);
    FaultHandler_FaultClearAll();
    LEDBuzz_Post(PatternStopAll);

    DEBUG_FTM("\nFTM Co StopSensitivityTest", false, 0U);

    return NG_NACK_REASON_OK;
}


