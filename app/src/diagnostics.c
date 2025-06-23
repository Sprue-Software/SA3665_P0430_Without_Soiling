/*
 * diagnostics.c
 *
 *  Created on: 31 Mar 2022
 *      Author: ndiwathe
 *      sample code
 */
#include "acquisition_Smoke.h"
#include "acquisition_Co.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/source/os_priv.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include <SHT41.h>
#include "em_core.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "hal_BURTCTimer.h"
#include "events.h"
#include "app.h"
#include "system_events.h"
#include "diagnostics.h"
#include "hal_i2c.h"
#include "eeprom_handler.h"
#include "temp_humid.h"
#include "hal_switches.h"
#include "hal_AFE.h"
#include "battery_measurement.h"
#include "timeHandler.h"
#include "acquisition_Heat.h"
#include "ambient_light.h"
#include "telegram.h"
#include "spi_comms.h"
#include "hal_LETimer.h"
#include "led_buzzer.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "production.h"

#define MAX_RETRY             3U

static OS_FLAGS DelayedFlags = 0;
static bool bBistResult = false;
static bool isDeviceSmokeEnable = true;

/**
 * @brief setIsDeviceSmokeEnable
 * @details This module set the flag for device enable for smoke measurement
 * @Param bool
 * @return n/a
 */
void setIsDeviceSmokeEnable(bool val)
{
  isDeviceSmokeEnable = val;
}

/**
 * @brief getIsDeviceSmokeEnable
 * @details This module return flag to check for device enable for smoke measurement
 * @Param n/a
 * @return true = smoke measurement enable: false = smoke measurement disable
 */
bool getIsDeviceSmokeEnable(void)
{
  return isDeviceSmokeEnable;
}


/**
 * @brief SetBistResult
 * @details This module is being called from runALLbist to set bist result
 * @Param bool
 * @return n/a
 */
void setBistResult(bool val)
{
  bBistResult = val;
}

/**
 * @brief getBistResult
 * @details This module is being called from checkBISTResults to return bist result
 * @Param n/a
 * @return true = success and false = failure
 */
bool getBistResult()
{
  return bBistResult;
}
/**
 * @brief runs buzzer bist
 * @details This module is being called from diagnostics to check buzzer hw fault
 * @Param n/a
 * @return 0 = success and 1 = failure
 */
uint8_t runBuzzerBist()
{

   uint8_t retVal = hal_AFE_HornFaultTest();
   get_the_buzzer_fault();
   return retVal;
}

/**
 * @brief runs All bist
 * @details This module is being called from diagnostics to check
 * Smoke, heat, co, buzzer, obstacle battery temp humid hw fault
 * @Param n/a
 * @return true = success and false = failure
 */

bool runAllBist()
{
  bool retVal           = true;
  bool bist_result      = true;
  uint8_t counter       = 0U;
  bool temp_humid_bist  = false;
  bool smoke_bist       = false;
  bool heat_bist        = false;
  bool batt_bist        = false;
  bool buzz_bist        = false;
  bool eeprom_bist      = false;

  /*As per PTR-1254, all BISTs should be run at the first time the device is installed on base (commissioning mode)*/
  /* Trigger temp & humidity BIST */
  while(counter < MAX_RETRY)
  {
      temp_humid_measure_bist(&bist_result );
      if( bist_result )
      {
          DEBUG_DIAG("\n BIST failed(Temp Humid)", false, 0u);
          temp_humid_bist = true;
      }
      else
      {
          DEBUG_DIAG("\n BIST Passed(Temp Humid)", false, 0u);
          temp_humid_bist = false;
          break;
      }
      counter++;
  }


  bist_result = batt_circuit_bist();

    if( bist_result )
    {
      DEBUG_DIAG("\n BIST failed(Batt)", false, 0u);
      batt_bist = true;
    }
    else
    {
      batt_bist = false;
      DEBUG_DIAG("\n BIST Passed(Batt)", false, 0u);
    }

  if(getIsDeviceSmokeEnable())
  {
    /* Smoke BIST */
    counter = 0U;
    while(counter < MAX_RETRY)
    {
        bist_result = smoke_measurement_and_bist();
        if( bist_result || get_cham_minor_fault())
        {
            DEBUG_DIAG("\n BIST failed(Smoke)", false, 0u);
            smoke_bist = true;

        }
        else
        {
            DEBUG_DIAG("\n BIST Passed(Smoke)", false, 0u);
            smoke_bist = false;
            break;
        }
        counter++;
    }
  }

  counter = 0U;
  while(counter < MAX_RETRY)
  {
      bist_result = heat_measurement(true);
      if(bist_result)
      {
          DEBUG_DIAG("\n BIST Passed(Heat)", false, 0u);
          heat_bist = false;
          break;
      }
      else
      {
          DEBUG_DIAG("\n BIST failed(Heat)", false, 0u);
          heat_bist = true;
      }
      counter++;
  }

  uint8_t buzzbist = 0U;
  counter = 0U;
  while(counter < MAX_RETRY)
  {
      buzzbist = runBuzzerBist();
      if( buzzbist == 0U )
      {
          DEBUG_DIAG("\n BIST Passed(BUzzer)", false, 0u );
          buzz_bist = false;
          break;
      }
      else
      {
          DEBUG_DIAG("\n BIST Failed(BUzzer)", false, 0u );
          buzz_bist = true;
      }
      counter++;
  }

  const bool eeprom_ok = data_logging_is_eeprom_ok();
  if(eeprom_ok == false)
  {
    DEBUG_DIAG("\nBIST Failed(EEPROM CRC)", false, 0ul );
    FaultHandler_FaultSet( EEPROMCalDataCorruptionFault );
    DataLogging_SetMinorFault(FaultCalibrationDataCorrupt, true);
    eeprom_bist = true;
  }
  else
  {
    DEBUG_DIAG("\nBIST Passed(EEPROM CRC)", false, 0ul );
  }

  if(temp_humid_bist | smoke_bist | heat_bist | batt_bist | buzz_bist | eeprom_bist)
  {
    setBistResult(false);
    retVal = false;
  }
  else
  {
    setBistResult(true);
  }

  return retVal;
}

/**
 * @brief Performs heat measurement
 * @details This module is being called from diagnostics and for FTM heat
 * @Param bool bist_test
 * @return true/false = success/failure
 */
bool heat_measurement(bool bist_test)
{
   int32_t temperature = 0;
   bool retVal = false;
   RTOS_ERR err;
   OS_MSG_SIZE size;

   DEBUG_DIAG("\n Heat BIST + Measurement ", false, 0u);
   hal_AFE_Post(setup_thermistor, NULL, NULL, false);
   AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
   APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
   smoke_thermistek_meas  = afeResponse->afe_adc_data;
   /*Battery voltage compensation*/

   //No compensation needed as the reference voltage is internal reference
   DEBUG_DIAG("\n :****************:", false, 0u);
   DEBUG_DIAG("\n ADC count:", true, smoke_thermistek_meas);
   if (calculateTemperature((uint32_t)smoke_thermistek_meas, &temperature, &retVal) == true)
   {
     DEBUG_DIAG("\n Therm temp:", true, temperature);
     if(!bist_test)
     {
       if(temperature != DEFAULT_TEMPERATURE)
       {

         detectHeat(temperature);
       }
       else
       {
        //Check if this is needed. Add new fault condition ( calculation fault) in FAULTHANDLER_FAULT. As it is almost caused by fault ADC values which is covered.
       }
     }
   }
   return retVal;
}

/*  sample code  for testing */
//#define EXAMPLE_CODE
/**
 * @brief Diagnostics routine, run acquisition routines and check for faults.
 * @details
 * This is just for reference to test the logic , Need to implement as per the requirement
 * @req PTR-1401, PTR-1085, PTR-1086, PTR-1124, PTR-1074
 */
bool diagnostics(behaviour_state_enum_System_modes modes, Ads_state_t ADS_status, OS_FLAGS flags) {
  /*This need protection semaphore as this function called by different places*/

  bool diagPerformed = true;
  OS_MSG_SIZE size;
  RTOS_ERR err;
  AFERspMessage_t *afeResponse;
  bool bistResult = false;

  /* This will check the Gap of Led Buzzer */
   if ((LEDBuzz_checkForGap() == false) && 
      ((flags & (uint32_t) DIAGNOSTIC_EVENTS()) || (flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0))))
   {
       /*LED/Buzzer is active so delay the diagnostics until complete*/
       diagPerformed = false;
       LETimer_start(LETIMER_DIAG_DELAY, DIAG_DELAY_PERIOD);
       /*Add new delayed flags*/
       DelayedFlags |= (flags & (uint32_t) DIAGNOSTIC_EVENTS());
       DelayedFlags |= (flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0));
   }
   else 
   {
       /* MUA TODO: Potential Racing condition*/
       /*Reset delayed flags*/
       DelayedFlags = 0u;
       /* All modules need to disable BIST in Alarm */
       /* This will Periodic when device- Operational mode + ADS On +event are set by BURTC timers */
       if ((((modes == Operational_Mode) || (modes == Functional_Test_Mode)) && (ADS_status == Ads_onBase)) && (flags != 0))
       {
           DEBUG_DIAG("\n ", false, 0u);
           if ((flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0)) != 0u)
           {

               DEBUG_DIAG("\n Battery Measurement", false, 0u);
               hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
               afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
               APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
               battery_bist(); /* battery BIST */
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_Smoke_BIST_event_0)) != 0u)
           {
               if(getIsDeviceSmokeEnable() == true)
               {
                   /* Smoke chamber BIST flag is set before the smoke measurement */
                   DEBUG_DIAG("\n Smoke BIST", false, 0u);
                   if(modes == Functional_Test_Mode)
                   {
                       smoke_measurement_and_bist();
                   }
                   else
                   {
                       smoke_chamber_test = true;
                   }
               }
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_Smoke_measure_event_0)) != 0u)
           {
               /* based on the flag status, smoke chamber BIST will be done
                * after the smoke measurement */
               if(getIsDeviceSmokeEnable() == true)
               {
                   smoke_periodic_measurement();
                   DEBUG_DIAG("\n Smoke Measurement: ", true, smoke_get_value());
               }
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_Heat_measure_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\nTime Stamp ", true, get_currentTime());
               time_handleTimestamp();
               (void)heat_measurement(false);
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_heartbeat_event_0)) != 0u)
           {
               if(getHeartBeatFlag() == true)
               {
                   setHeartBeatFlag(false);
                   LEDBuzz_HeartbeatRequestProcess();
               }
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_CO_measure_event_0)) != 0u)
           {

               DEBUG_DIAG("\n Co Measurement", false, 0u);
               acquisitionCO(false);
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_CO_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Co BIST ", false, 0u);
               acquisitionCO(true);
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_TempHum_measure_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Operation Temp-Humid", false, 0u);
               temp_humid_measure_bist(&bistResult); /* measure the temperature and humidity sensor values */
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_BUZZER_BIST_event_0)) != 0u)
           {
               runBuzzerBist();
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_Obstacle_Coverage_BIST_event_0)) != 0u)
           {
               Set_OC_Parameter(OC_detection ,0u,0u,0u,0u);
               SPIComms_Send_Data_to_MCU2(SPI_CMD_Trig_Detection);
               set_obs_det_ftm_period_timeover(false);
               DEBUG_DIAG("\n Obstacle + Coverage BIST  ", false, 0u);
           }
       }
       /* This will happen only when device- Standby mode + ADS off only Battery +Temp+SPI will be active */
       if ((modes == Standby_Mode) && (ADS_status == Ads_offBase))
       {

           if ((flags & FLAGS_BIT_INDEX(TMR_TempHum_measure_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Standby Temp-Humid", false, 0u);
               temp_humid_measure_bist(&bistResult); /* measure the temperature and humidity sensor values */
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0)) != 0u)
           {

               //DEBUG_DIAG("\n Standby Battery Measurement", false, 0u);
               hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
               afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
               APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
               battery_bist(); /* battery BIST */
           }

       }
       /* This will happen only when device- Transport mode */
       if ((modes == Transport_Mode) || (modes == Commisioning_Mode))
       {
           if ((flags & FLAGS_BIT_INDEX(TMR_TempHum_measure_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Transport_Mode Temp-Humid", false, 0u);
               temp_humid_measure_bist(&bistResult); /* measure the temperature and humidity sensor values */
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0)) != 0u)
           {

               DEBUG_DIAG("\n Transport_Mode Battery Measurement", false, 0u);
               hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
               afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
               APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
               battery_bist(); /* battery BIST */
           }

       }
       /* This will happen only when device- Standby mode + ADS enabled first time */
       if ((modes == Standby_Mode) && (ADS_status == Ads_onBase) && (flags == 0))
       {  /* UH> flags == 0 required?? */
            /* start the BIST running pattern */
             DEBUG_DIAG("\n StandBy Run all BIST", false, 0u);
             diagPerformed = runAllBist();
       }

       /* This will happen only when device- Operational mode + ADS enabled first time */
       if (((modes == Operational_Mode) && (ADS_status == Ads_onBase)) && (flags == 0))
       {

           DEBUG_DIAG("\n Operational BIST ALL", false, 0u);
           diagPerformed = runAllBist();
    
       }
       /* This will happen only when device- Operational mode + ADS offbase */
       //@usha: Need to revisit this
       if (((modes == Operational_Mode) && (ADS_status == Ads_offBase)) && (flags != 0))
       {

           if ((flags & FLAGS_BIT_INDEX(TMR_heartbeat_event_0)) != 0u)
           {
              if(getHeartBeatFlag() == true)
              {
                  setHeartBeatFlag(false);
                  LEDBuzz_HeartbeatRequestProcess();
              }
           }

           if ((flags & FLAGS_BIT_INDEX(TMR_TempHum_measure_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Operationl Mode Off Base Temp-Humid", false, 0u);
               temp_humid_measure_bist(&bistResult); /* measure the temperature and humidity sensor values */
           }
           if ((flags & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0)) != 0u)
           {
               DEBUG_DIAG("\n Operationl Mode Off Base Battery Measurement", false, 0u);
               hal_AFE_Post(setup_batteryVoltage, NULL, NULL, false);
               afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
               APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
               battery_bist(); /* battery BIST */
           }

           /*Check Button Press status if pattern is required if for Domestic or Extended */
       }

       DEBUG_DIAG("\n************", false, 0u);
   }
  return diagPerformed;
}

/**
 * @brief getThermistorADC
 * @details This module returns thermistor ADC data
 * @Param N/A
 * @return smoke_thermistek_meas
 */
uint16_t getThermistorADC(void)
{
  return smoke_thermistek_meas;
}

OS_FLAGS diagnostics_GetDelayedFlags(void)
{
  return DelayedFlags;
}
