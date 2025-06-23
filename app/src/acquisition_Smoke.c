/*
 * Acquisition_Smoke.c
 *
 *  Created on: 1 Apr 2022
 *      Author:
 *
 */

#include "acquisition_Smoke.h"
#include "acquisition_smoke_dust.h"
#include "hal_LETimer.h"
#include "hal_AFE.h"
#include "events.h"
#include "hal_BURTCTimer.h"
#include "led_buzzer.h"
#include "system_events.h"
#include "fault_handler.h"
#include "data_logging.h"
#include "acquisition_Heat.h"

// TODO:  review and change these threshold values as required
#define SMOKE_CIRCUIT_FAULT_THRESHOLD     (542u) /* 20mV or less clean air measurement indicates fault in HW */
#define SMOKE_MAX_CHAMBER_FAULT_STRIKE_COUNT  (3u) /* smoke circuit hardware fault strike count */
#define SMOKE_THERMAL_STABILISATION_COUNT (3u) /* number of readings to wait after power on abefore executing the fast flame mode */
#define SMOKE_FAST_FLAMEMODE_THRESHOLD    (8u) /* 5deg/min (50 per min as scaled) or 50/6 (~8) per 10sec */
#define SMOKE_FAST_FLAME_TIMEOUT_TIME_MS  (180000u) /* 3 minutes fast flame mode timeout time */
#define SMOKE_CORR_HISTORY_MAX_SIZE        (255u) 
#define SMOKE_CORR_HISTORY_DEFAULT_SIZE    (168u) /* 1 week history of the dark smoke measurement 24*7 = 168*/
//#define SMOKE_ALARM_THRESHOLD             (0x1850) /* smoke alarm activation threshold ADC count */
#define SUPERSMOKE_FACTOR                 (6u) /* super smoke condition threshold (x6 times smoke threshold) */
#define SMOKE_ALARM_STRIKECOUNT_MAX       (3u) /* readings taken before setting the alarm condition */
#define NO_SMOKE_STRIKECOUNT              (2u) /* number of measurements taken before clearing the alarm condition */
#define SMOKE_CLEAN_AIR_VALUE             (4250u) /* calibration value for clean air */
#define SMOKE_CALIBRATION_VALUE           (7150u) /* calibration value for aerosol */

static SmokeResult smoke_result;
static bool smoke_bist_fault;
static uint8_t smoke_temp_history_index;
static uint16_t smoke_thermistek_history[2u];   //Last two temperature readings
static uint8_t smoke_thermistek_count;
static bool thermal_stabilisation_done;
static uint16_t current_Smoke_value;
static uint8_t smoke_alarm_strikecount; /* counter to identify the high smoke level */
static uint8_t super_smoke_strikecount; /* counter to identify the super smoke level */
static bool smoke_increased_periodicity; /* faster sample rate enabled ?? */
static uint8_t smoke_clear_strikecount; /* no smoke confirmation strike count */

static int16_t smoke_thermistek_latest;
static int16_t smoke_thermistek_previous;
static int16_t smoke_dust_correction;

static Smoke_FTM_state_enum SmokeFTMState = Smoke_ftm_none;
static uint16_t smoke_ftm_darkresult;   /* FTM simulated dark values */
static uint16_t smoke_ftm_brightresult; /* FTM simulated bright values */
static bool smoke_ftm_simulation_active; /* checks for ftm simulated status */
static bool ftm_chamber_fault = false;

uint16_t smoke_thermistek_meas; /* temperature ADC value read from thermistor */
bool smoke_fastflame_required; /* do we need to execute the fast flame mode?? */
uint16_t smoke_alarm_threshold; /* threshold limit for the smoke alarm threshold */
uint16_t supersmoke_threshold;
bool smoke_fast_flame_mode_st; /* fast flame mode running ?? */
bool smoke_chamber_test; /* is it time to do the smoke chamber test? */
bool super_smoke_st; /* super smoke status */
bool smoke_alarm_st; /* smoke alarm condition */

static uint16_t EEPROM_CAL_CLEANAIR_MEAS        = SMOKE_CLEAN_AIR_VALUE;
static uint16_t EEPROM_CAL_SMOKE_MEAS           = SMOKE_CALIBRATION_VALUE;
static uint32_t SmokeCircuitFaultThreshold      = SMOKE_CIRCUIT_FAULT_THRESHOLD;
static uint16_t SmokeFastFlameThreshold         = SMOKE_FAST_FLAMEMODE_THRESHOLD;
static uint8_t SmokeMaxChamberFaultStrikeCount  = SMOKE_MAX_CHAMBER_FAULT_STRIKE_COUNT;

#ifdef DEBUG_BUILD
static bool simulatedSmokeValues = false;
static uint16_t currentSmokeValue = 0;
// static uint16_t currentSmokeThreshold = 0;
#endif

static uint16_t smoke_level; /* ADC difference between bright and dark measurement */

/**
 * @brief smoke module initialisation function
 */
void smoke_init(void) {
  /* initialisation of smoke module */
  smoke_result.brightresult = 0u;
  smoke_result.darkresult = 0u;
  smoke_bist_fault = false;

  smoke_thermistek_history[0] = 0u;
  smoke_thermistek_history[1] = 0u;
  smoke_temp_history_index = 0u;
  smoke_thermistek_meas = 0u;
  smoke_thermistek_count = 0u;

  thermal_stabilisation_done = false; /* thermal stabilisation period status */
  smoke_fastflame_required = true; /* by default fast flame mode required */
  smoke_alarm_threshold = 0u; /* initialise the smoke alarm threshold value */
  supersmoke_threshold = 0u;
  smoke_fast_flame_mode_st = false; /* fastflame mode not running by default */
  smoke_chamber_test = false; /* it not time to do the smoke chamber test on power on reset */

  super_smoke_st = false; /* no super smoke condition by default */
  smoke_alarm_st = false; /* no smoke detected by default */
  current_Smoke_value = 0u;

  smoke_alarm_strikecount = 0u;
  super_smoke_strikecount = 0u;
  smoke_clear_strikecount = 0u;

  smoke_thermistek_latest = 0;
  smoke_thermistek_previous = 0;
  smoke_dust_correction = 0;

  smoke_increased_periodicity = false; /* normal periodicity enabled by default */

  smoke_ftm_darkresult = 0U;
  smoke_ftm_brightresult = 0U;
  smoke_ftm_simulation_active = false;

#ifdef EEPROM_CALI
  /* Get eeprom integrity? */
  const bool eeprom_ok = data_logging_is_eeprom_ok( );

  /* Can eeprom be trusted? */
  if( eeprom_ok )
  {
    EEPROM_CAL_CLEANAIR_MEAS = DataLogging_GetSmokeCalRefThreshold( );

    /* If EEPROM values not initialised then take defaults */
    if( ( EEPROM_CAL_CLEANAIR_MEAS == 0U ) || ( EEPROM_CAL_CLEANAIR_MEAS == 0xFFFFU ) )
    {
        EEPROM_CAL_CLEANAIR_MEAS = SMOKE_CLEAN_AIR_VALUE;
    }

    /* If EEPROM values not initialised then take defaults */
    EEPROM_CAL_SMOKE_MEAS = DataLogging_GetSmokeCalThreshold( );

    if( ( EEPROM_CAL_SMOKE_MEAS == 0U ) || ( EEPROM_CAL_SMOKE_MEAS == 0xFFFFU ) )
    {
        EEPROM_CAL_SMOKE_MEAS = SMOKE_CALIBRATION_VALUE;
    }

    SmokeMaxChamberFaultStrikeCount = DataLogging_GetSmokeChamberFaultStrikeCount();

    if(SmokeMaxChamberFaultStrikeCount != SMOKE_MAX_CHAMBER_FAULT_STRIKE_COUNT)
    {
        SmokeMaxChamberFaultStrikeCount = SMOKE_MAX_CHAMBER_FAULT_STRIKE_COUNT;
        DataLogging_SetSmokeChamberFaultStrikeCount(SmokeMaxChamberFaultStrikeCount);
    }

    uint16_t smokeBistPeriod  = DataLogging_GetSmokeBISTPeriod();
    if(smokeBistPeriod != SMOKE_BIST_MEASUREMENT)
    {
        smokeBistPeriod = (uint16_t)SMOKE_BIST_MEASUREMENT;
        DataLogging_SetSmokeBISTPeriod(smokeBistPeriod);
    }

    uint8_t smokeAcqPeriod = DataLogging_GetSmokeAcqPeriod();
    if(smokeAcqPeriod != SMOKE_MEASUREMENT)
    {
        smokeAcqPeriod = (uint8_t)SMOKE_MEASUREMENT;
        DataLogging_SetSmokeAcqPeriod(smokeAcqPeriod);
    }

  }
#endif

  DEBUG_SMOKE("\nCAL_CLEANAIR:", true, EEPROM_CAL_CLEANAIR_MEAS);
  DEBUG_SMOKE("\nCAL_SMOKE:", true, EEPROM_CAL_SMOKE_MEAS);
  
  /* Initialises the smoke averaging function used for dust correction */
  smoke_dust_init( EEPROM_CAL_CLEANAIR_MEAS );
}

/**
 * @brief smoke circuit fault condition check
 * @return return the smoke hardware status,
 * true= faulty, false= not faulty
 * @req PTR-1120, PTR-1119, PTR-1212, PTR-1204
 */
bool smoke_circuit_check(void)
{
  bool bist_fail_status = false;
  static bool hw_smoke_log_error = false;
  static uint8_t smoke_chamber_fault_strike_count = 0;
  if (smoke_bist_fault == false)
  {
    /* read the current fault status */
    uint32_t Faults = FaultHandler_GetFaultFlags();
    /* smoke hw fault already not set */
    if ((Faults & DEF_SMOKE_CHAMBER_HW_FAULT) == 0u)
    {
        if ((smoke_result.darkresult > smoke_result.brightresult)
            || ((smoke_result.brightresult - smoke_result.darkresult)
                < SmokeCircuitFaultThreshold))
        {
            smoke_chamber_fault_strike_count++;
            if (smoke_chamber_fault_strike_count >= SmokeMaxChamberFaultStrikeCount)
            {
                smoke_bist_fault = true; /* smoke hardware fault is confirmed */
                DEBUG_SMOKE("\n Smoke circuit fault", false, 0u);
                if(hw_smoke_log_error == false)
                {
                    /* Indicate the LED and buzzer status using major fault pattern */
                    FaultHandler_FaultSet(SmokeChamberDiodeHwFault);
                    DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_CHAM_HW_ERR_START, NULL );
                    hw_smoke_log_error = true;
                }
            }
            bist_fail_status = true; /* set the BIST fail status */
        }
        else
        {
            smoke_chamber_fault_strike_count = 0u; /* reset the strike count, fault not confirmed */
            smoke_bist_fault = false;
            bist_fail_status = false;
            DEBUG_SMOKE("\n No Smoke circuit fault", false, 0u);
            hw_smoke_log_error = false;
        }
    }
    else
    {
        /* do nothing if the major fault is already set */
        bist_fail_status = true; /* set the BIST fail status */
        SmokeFTMState = Smoke_ftm_alarm_faulty;
    }
  }
  else
  {
    /* smoke hardware fault is present. Do nothing */
    bist_fail_status = true; /* set the BIST fail status */
    SmokeFTMState = Smoke_ftm_alarm_faulty;
    DEBUG_SMOKE("\n Smoke latchy fault", false, 0u);
  }
  return bist_fail_status;
}

/**
 * @brief smoke measurement and hardware BIST to confirm the smoke circuit fault.
 * strike count method is used to confirm the fault status.
 * This function can be called either from commissioning mode or operating mode or functional test mode
 * @return returns the fault status of smoke circuit
 * true = faulty, false= non-faulty
 * @req PTR-1268, PTR-1356, PTR-1185, PTR-1028
 */
bool smoke_measurement_and_bist(void)
{
    RTOS_ERR err;
    OS_MSG_SIZE size;
  bool smoke_hardware_fault;

  if (smoke_bist_fault == false)
  {
    if (smoke_ftm_simulation_active == true)
    {
            /* TODO: get the values from functional test mode interface */
            smoke_result.darkresult = smoke_ftm_darkresult;     //0x1592;
            smoke_result.brightresult = smoke_ftm_brightresult; //0x2f09;
    }
    else
    {
        hal_AFE_Post(setup_SmokeTest, NULL, &smoke_result, false); /* measurement from smoke circuit */
        AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    }

    smoke_hardware_fault = smoke_circuit_check(); /* smoke circuit BIST */

    /* strike count running, need to reread the status */
    while ((smoke_bist_fault == false) && (smoke_hardware_fault == true))
    {
      hal_AFE_Post(setup_SmokeTest, NULL, &smoke_result, false); /* smoke measurement */
      AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
      APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
      smoke_hardware_fault = smoke_circuit_check(); /* smoke circuit BIST */
      /* run the loop until fault is confirmed or BIST is successful */
      //DEBUG_SMOKE("\n Smoke strike count...", false, 0u);
    }

    if (smoke_hardware_fault == false)
    {
        //DEBUG_SMOKE("\n Smoke raw measurement ok", false, 0u);
    }
    else
    {
        /* For LDRA */
    }
  }
  else
  {
    smoke_hardware_fault = true; /* no smoke measurement as there is a HW fault*/
    DEBUG_SMOKE("\n Smoke HW fault", false, 0u);
  }

  return smoke_hardware_fault;
}

/**
 * @brief  This is the periodic raw smoke measurement and BIST function
 * below are the actions performed in the function:
 * - Smoke ADC measurement
 * - Smoke circuit BIST
 * - fast flame mode check
 * - call the dust compensation function
 * - smoke chamber BIST based on the dust compensation measurement
 * - Call the function to calculate compensated smoke level
 * @return  none
 * @req PTR-1166, PTR-1173, PTR-1085, PTR-1124, PTR-1179, PTR-1083, PTR-1079, PTR-1188, PTR-1356, PTR-1355
 *      PTR-983, PTR-1126, PTR-1147, PTR-1185, PTR-1044, PTR-1028, PTR-1073, PTR-1117, PTR-1116,
 */
void smoke_periodic_measurement(void)
{
  bool smoke_hwfault_st = false;
  //uint16_t smoke_level; /* ADC difference between bright and dark measurement */
  //!uint16_t corr_smoke_level; /* corrected smoke level */
  uint16_t thermistek_delta; /* difference in temperature measurement */
  static bool firstSmokeFastFlame = true;

  //For LDRA
  int16_t   int16_var;

  RTOS_ERR err;

  DEBUG_SMOKE("\nENTER SMOKE CHECK", false, 0u);

  smoke_hwfault_st = smoke_measurement_and_bist(); /* smoke ADC measurement and hardware BIST */

  if (smoke_hwfault_st == false)
  {
    /* only if smoke hardware is non faulty, proceed further */
      //DEBUG_SMOKE("\n\n^^^^^^^^^^^^^^^", false, 0u);
      //DEBUG_SMOKE("\nStart smoke alarm check", false, 0u);

    smoke_thermistek_latest = getHeatAfterCompensation();

    if (smoke_thermistek_count < SMOKE_THERMAL_STABILISATION_COUNT)
    {
      smoke_thermistek_count++; /* increase the thermal stabilisation count */
      if (smoke_thermistek_count >= (SMOKE_THERMAL_STABILISATION_COUNT))
      {
        //DEBUG_SMOKE("\nThermistor meas.=", true, smoke_thermistek_latest);
        thermal_stabilisation_done = true;
      }
      else
      {
        /* if thermal stabilisation period is not expired,
         * do not execute fast flame mode.
         * do not store the temperature values during the thermal stabilisation time
         */
        //DEBUG_SMOKE("\nThermal stabil running...", false, 0u);
      }
    }
    else
    {
        /* For LDRA */
    }

    if(firstSmokeFastFlame == true)
    {
        smoke_thermistek_previous = smoke_thermistek_latest;
        firstSmokeFastFlame = false;
    }

    if ((thermal_stabilisation_done == true) && (smoke_fastflame_required == true))
    {
      //DEBUG_SMOKE("\nThermal stabil. Done", false, 0u);
      if ( (smoke_thermistek_latest != DEFAULT_TEMPERATURE) &&
           (smoke_thermistek_previous != DEFAULT_TEMPERATURE) &&
           (smoke_thermistek_latest > smoke_thermistek_previous))
      {
          /* if current temperature is more than previous one */
          int16_var = smoke_thermistek_latest - smoke_thermistek_previous;
          //thermistek_delta = (uint16_t)(smoke_thermistek_latest - smoke_thermistek_previous)
          thermistek_delta = (uint16_t)int16_var;
          // DEBUG_SMOKE("\nTemp delta: ", true, thermistek_delta);
          if ((thermistek_delta > SmokeFastFlameThreshold)
            && (smoke_fast_flame_mode_st == false))
          {

              /* if temperature is increasing in rapid rate */
              smoke_alarm_threshold = (EEPROM_CAL_SMOKE_MEAS >> 1u); /* reduce the alarm threshold to 1/2 */
              supersmoke_threshold = SUPERSMOKE_FACTOR * smoke_alarm_threshold;
              //LETimer_start(LETIMER_FAST_FLAME_MODE, SMOKE_FAST_FLAME_TIMEOUT_TIME_MS); /* fast flame mode timer start */
              BURTCTimer_Start(TMR_Smoke_Fast_Flame_Disable_1, one_shot, FAST_FLAME_DISABLE_EVENT );
              smoke_fast_flame_mode_st = true;
              DEBUG_SMOKE("\n***Fast flame mode ON", false, 0u);
          }
          else
          {
              /* For LDRA */
          }
      }
      else
      {
          /* For LDRA */
      }
    }
    else
    {
        /* For LDRA */
    }

    smoke_thermistek_previous = smoke_thermistek_latest;

    if (smoke_fast_flame_mode_st == false)
    {
      smoke_alarm_threshold = EEPROM_CAL_SMOKE_MEAS; /* set the normal alarm threshold */
      supersmoke_threshold = SUPERSMOKE_FACTOR * smoke_alarm_threshold;
      //DEBUG_SMOKE("\nSmoke threshold: ", true, smoke_alarm_threshold)
    }
    else
    {
        /* For LDRA */
    }

    DEBUG_SMOKE("\nSmoke threshold: ", true, smoke_alarm_threshold);

    if (smoke_result.brightresult > smoke_result.darkresult)
    {
      smoke_level = (smoke_result.brightresult - smoke_result.darkresult);
      DEBUG_SMOKE("\nSmoke level raw: ", true, smoke_level);
    }
    else
    {
      smoke_level = 0u;
    }

    /* Values required for dust correction */
    uint32_t inuse_smoke_raw_level;
    uint32_t inuse_cal_reflection_threshold;

    /* Get dust correction in test mode */
    const bool dust_correction_test_mode = smoke_dust_is_test_mode( );

    /* Is dust correction in test mode? */
    if( dust_correction_test_mode )
    {
      inuse_smoke_raw_level                = smoke_dust_get_raw_test_value( );
      inuse_cal_reflection_threshold       = smoke_dust_get_calref_test_value( );
    }
    else
    {
      inuse_smoke_raw_level                = smoke_level;
      inuse_cal_reflection_threshold       = EEPROM_CAL_CLEANAIR_MEAS;
    }

    /* No, correct the smoke level due to dust contamination */
    const uint32_t smoke_level_corr = smoke_dust_perform_correction( inuse_smoke_raw_level );

    /* Handle any dust errors */
    smoke_dust_handle_chamber_errors( smoke_level_corr );

    // TODO: calculate the smoke level based on the calibration values */
    if( smoke_level_corr > inuse_cal_reflection_threshold )
    {
        current_Smoke_value = smoke_level_corr - inuse_cal_reflection_threshold;
    }
    else
    {
        current_Smoke_value = 0u;
    }

#ifdef DEBUG_BUILD
    if(true == simulatedSmokeValues)
    {
        current_Smoke_value = currentSmokeValue;
    }
#endif
    DEBUG_SMOKE("\nCurrent smoke level", true, current_Smoke_value);

    if (current_Smoke_value >= smoke_alarm_threshold)
    {
        if ((current_Smoke_value >= supersmoke_threshold) && (super_smoke_st == false))
        {
            /* Note. Supersmoke_threshold is 6 times the alarm_threshold. so when alarm_threshold */
            /* is halfed, supersmoke_threshold is 6 times the new one, ie also halfed */
            super_smoke_strikecount++;
            if (super_smoke_strikecount >= SMOKE_ALARM_STRIKECOUNT_MAX)
            {
                super_smoke_st = true; /* set the super smoke condition */
                SmokeFTMState = Smoke_ftm_super;  /* set the super smoke condition for FTM*/
                DEBUG_SMOKE("\n***SUPER SMOKE ALARM ***", true, smoke_level_corr);
                /* post the smoke alarm event */
                OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
                           EVENT_SMOKE_HIGH_SUPER_0, /*   event bit-mask.              */
                           OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
                           &err);
                /*   Check error code */
                APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

                /* write event into EEPROM */
                if(smoke_alarm_st == true)
                {
                    DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_DET_END, NULL );
                    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                }

                DataLogging_SetSmokeEvent(EVENT_TYPE_LOCAL);
                OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_SMOKE_START, NULL );

                smoke_alarm_st = false;

                /* reset to default periodicity */
                BURTCTimer_Stop(TMR_Smoke_measure_event_0); /* stop the current periodic timer*/
                BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);
                smoke_increased_periodicity = false; /* normal periodicity 10sec */
            }
            else
            {
                /* strike count running. Increase the periodicity */
                BURTCTimer_Stop(TMR_Smoke_measure_event_0); /* stop the current periodic timer*/
                LETimer_start(LETIMER_SMOKE_FASTRATE, SMOKE_INCREASED_SAMPLE_RATE);
                smoke_increased_periodicity = true; /* increased periodicity 2sec */
            }
        }
        else
        {
          super_smoke_strikecount = 0u;
          //super_smoke_st = false;
          if ((smoke_alarm_st == false) && (super_smoke_st == false))
          {
              smoke_alarm_strikecount++;
              if (smoke_alarm_strikecount >= SMOKE_ALARM_STRIKECOUNT_MAX)
              {
                  smoke_alarm_st = true; /* set the smoke detected condition */
                  SmokeFTMState = Smoke_ftm_dectected;
                  DEBUG_SMOKE("\n***SMOKE ALARM ***", true, smoke_level_corr);
                  /* post the smoke alarm event */
                  OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
                             EVENT_SMOKE_HIGH_SUPER_0, /*   event bit-mask.              */
                             OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
                             &err);
                  /*   Check error code */
                  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
                  /* write event into EEPROM */
                  DataLogging_SetSmokeEvent(EVENT_TYPE_LOCAL);
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_DET_START, NULL );
                  /* reset to default periodicity */
                  BURTCTimer_Stop(TMR_Smoke_measure_event_0); /* stop the current periodic timer*/
                  BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);
                  smoke_increased_periodicity = false; /* normal periodicity 10sec */
              }
              else
              {
                  /* strike count running. Increase the periodicity */
                  BURTCTimer_Stop(TMR_Smoke_measure_event_0); /* stop the current periodic timer*/
                  LETimer_start(LETIMER_SMOKE_FASTRATE, SMOKE_INCREASED_SAMPLE_RATE); /* start the button press pattern monitor timer */
                  smoke_increased_periodicity = true; /* increased periodicity 2sec */
                  DEBUG_SMOKE("\n**Smoke detected**", false, 0u);
              }
          }
          else
          {
            /* Used to solve the problem that the smoke detection cycle is closed and the smoke alarm connot be exited */
            if(BURTCTimer_Get_Event_Enable(TMR_Smoke_measure_event_0) == false)
            {
                BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);
            }
          }
        }
    }
    else
    {
        /* clear the smoke detected conditions */
        if ((super_smoke_st == true) || (smoke_alarm_st == true))
        {
            smoke_clear_strikecount++;
            if (smoke_clear_strikecount > NO_SMOKE_STRIKECOUNT)
            {

                SmokeFTMState = Smoke_ftm_none;
                super_smoke_strikecount = 0u;
                smoke_alarm_strikecount = 0u;
                DEBUG_SMOKE("\n***NO SMOKE ALARM ***", true, smoke_level_corr);

                /* write event into EEPROM */
                if(super_smoke_st == true) /*Mark end of Super smoke*/
                {
                    DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_SMOKE_END, NULL );
                    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                }

                if(smoke_alarm_st == true)
                {
                    DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_DET_END, NULL );
                    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                }

                /* post the smoke alarm clear event */
                 OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
                 EVENT_SMOKE_NONE_0, /*   event bit-mask.              */
                 OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
                 &err);
                 /*   Check error code */
                 APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

                smoke_alarm_st = false;
                super_smoke_st = false;
            }
            else
            {
                /* do nothing when strikecount running */
            }
        }
        else
        {
            smoke_clear_strikecount = 0u;
        }

        if (smoke_increased_periodicity == true)
        {
            /* reset to default periodicity */
            BURTCTimer_Stop(TMR_Smoke_measure_event_0); /* stop the current periodic timer*/
            BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);
            smoke_increased_periodicity = false; /* normal periodicity 10sec */
            /* increased periodicity will have the non-zero strike count */
            super_smoke_strikecount = 0u;
            smoke_alarm_strikecount = 0u;
        }
        else
        {
          /* For LDRA */
        }
    }
  }
  else
  {
    /* TODO: Do nothing, as fault management module will takecare of
     * reported smoke related fault status.
     */
    DEBUG_SMOKE("\n SMOKE HW FAULT- NO MEAS", false, 0u);
  }
}

/**
 * @brief  This Function will return the smoke value.(Which include Compensation )
 * @return  smoke value
 */
uint16_t smoke_get_value(void) {

  return current_Smoke_value;
}

uint16_t smoke_get_percentage(void) {

  uint16_t smoke_percent_value = 0u;

  if(EEPROM_CAL_SMOKE_MEAS != 0u)
    {
      smoke_percent_value = (uint16_t)(((uint32_t)current_Smoke_value * 100u) / (uint32_t)EEPROM_CAL_SMOKE_MEAS);
    }
  else
    {
      smoke_percent_value = 0u;
    }

  return smoke_percent_value;
}

/**
 * @brief returns the super smoke status detected by the device
 * @return true=device in super smoke level, false= no super smoke level
 */
bool super_smoke_detected(void) {
  return super_smoke_st;
}

/**
 * @brief returns the smoke alarm status detected by the device
 * @return true=smoke alarm condition detected, false= no smoke alarm condition detected
 */
bool smoke_detected(void) {
  return smoke_alarm_st;
}

/**
 * @brief This function returns the smoke dark measurement value
 * @return dark results
 */
uint16_t smoke_get_dark_measurement() {
  return  smoke_result.darkresult;
}

/**
 * @brief This function returns the smoke bright measurement value
 * @return bright results
 */
uint16_t smoke_get_bright_measurement() {
  return  smoke_result.brightresult;
}

/**
 * @brief This function set the FTM simulated smoke dark result
 * @param const uint8_t dark_msb, const uint8_t dark_lsb
 * @return  none
 */
void smoke_set_dark_measurement(const uint8_t dark_msb, const uint8_t dark_lsb) {

  smoke_ftm_darkresult = dark_msb;
  smoke_ftm_darkresult = smoke_ftm_darkresult << 8;
  smoke_ftm_darkresult = smoke_ftm_darkresult | dark_lsb;

}

/**
 * @brief This function set the FTM simulated smoke bright result
 * @param const uint8_t dark_msb, const uint8_t dark_lsb
 * @return  none
 */
void smoke_set_bright_measurement(const uint8_t bright_msb, const uint8_t bright_lsb) {
    smoke_ftm_brightresult = bright_msb;
    smoke_ftm_brightresult = smoke_ftm_brightresult << 8U;
    smoke_ftm_brightresult = smoke_ftm_brightresult | bright_lsb;
}

/**
 * @brief This function return FTM smoke state
 * @param none
 * @return  smoke ftm state
 */
Smoke_FTM_state_enum getFTMSmokeState() {
  return SmokeFTMState;
}

/**
 * @brief This function set FTM simulation active state
 * @param bool ftmSimulationState
 * @return  none
 */
void smoke_set_ftm_active(bool ftmSimulationState) {
  smoke_ftm_simulation_active = ftmSimulationState;
}

/**
 * @brief This function set chamber test to perform
 * @param bool enable
 * @return  none
 */
void smoke_set_ftm_chamber_test(bool enable)
{
  smoke_chamber_test = enable;
}

/**
 * @brief This function get chamber BIST result
 * @param none
 * @return  ftm_chamber_fault
 */
bool smoke_get_ftm_chamber_status()
{
  return ftm_chamber_fault;
}

/**
 * @brief returns the smoke level detected by the device
 * @return smoke level
 */
uint16_t get_smoke_level(void)
{
    return (smoke_level);
}

/**
 * @brief This function re-initialise smoke variable for demount.
 * @return smoke level
 */
void smoke_demount_init (void)
{
  super_smoke_st = false; /* no super smoke condition by default */
  smoke_alarm_st = false; /* no smoke detected by default */
  smoke_alarm_strikecount = 0u;
  super_smoke_strikecount = 0u;
  smoke_clear_strikecount = 0u;
}

#ifdef DEBUG_BUILD

uint16_t getSuperSmokeThreshold()
{
  return supersmoke_threshold;
}
uint16_t getSmokeThreshold()
{
  return smoke_alarm_threshold;
}
void SetSimulatedSmokeMode(bool simulated)
{
  simulatedSmokeValues = simulated;
}

void InjectCurrentSmokeValue(uint32_t smokeValue)
{
  if(true == simulatedSmokeValues)
  {
    currentSmokeValue = smokeValue;
  }
}

void InjectCurrentSmokeThreshold(uint32_t smokeThresh)
{
  if(true == simulatedSmokeValues)
  {
    EEPROM_CAL_SMOKE_MEAS = smokeThresh;
  }
}

#endif
