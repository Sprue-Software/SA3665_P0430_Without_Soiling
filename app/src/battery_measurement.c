/*
 * battery_measurement.c
 *
 *  Created on: 7 Jun 2022
 *      Author: uhegde
 */
#include "os.h"
#include "rtos_err.h"
#include "events.h"
#include "hal_AFE.h"
#include "comms_handler.h"
#include "battery_measurement.h"
#include "hal_BURTCTimer.h"
#include "led_buzzer.h"
#include "fault_handler.h"
#include "data_logging.h"


battery_meas_st battery_status_A; /* status structure of battery A */
battery_meas_st battery_status_B; /* status structure of battery B */

bool battery_periodicity_status = true; /* normal periodicity enabled by default */
bool battery_circuit_fault = false; /* no battery fault by default */

static uint8_t battery_high_strike_count = 0u; /* high voltage battery strike count */

uint32_t adc_battery_A = 40622u;//3V adc counts
uint32_t adc_battery_B = 40622u;//3V adc counts
uint32_t batt_impedance_A;
uint32_t batt_impedance_B;
bool buzzer_off;

static uint16_t lowBattThres = 0U;
static uint16_t deadBattThres = 0U;
static uint8_t strikeBattCount = 0U;
static bool timerChangePeriodicityOnce = false;
static bool lowBattStatus = false;

static void battery_handle_deadbatt_state(void);

struct {
	uint16_t battery_A_voltage_30days[BATTERY_30DAYS_DATA];    /* Battery A voltage 30 days history */
	uint16_t battery_A_impedance_30days[BATTERY_30DAYS_DATA];  /* Battery A impedance 30 days history */

	uint16_t battery_A_voltage_1day[BATTERY_1DAY_DATA];		 /* Battery A voltage 1 days history */
	uint16_t battery_A_impedance_1day[BATTERY_1DAY_DATA];      /* Battery A impedance 1 days history */

	uint16_t battery_A_voltage_3hrs[BATTERY_3HRS_DATA]; 		/* Battery A voltage 3 hrs history */
	uint16_t battery_A_impedance_3hrs[BATTERY_3HRS_DATA];		/* Battery A impedance 3 hrs history */

	uint16_t battery_B_voltage_30days[BATTERY_30DAYS_DATA];	/* Battery B voltage 30 days history */
	uint16_t battery_B_impedance_30days[BATTERY_30DAYS_DATA]; /* Battery B impedance 30 days history */

	uint16_t battery_B_voltage_1day[BATTERY_1DAY_DATA];		/* Battery B voltage 1 day history */
	uint16_t battery_B_impedance_1day[BATTERY_1DAY_DATA];		/* Battery B impedance 1 day history */

	uint16_t battery_B_voltage_3hrs[BATTERY_3HRS_DATA];		/* Battery B voltage 3 hrs history */
	uint16_t battery_B_impedance_3hrs[BATTERY_3HRS_DATA];		/* Battery B impedance 3 hrs history */

	uint16_t battery_monthly_count;  /* monthly buffer index */
	uint16_t battery_day_count;      /* daily buffer index */
	uint16_t battery_3hrs_count; 	/* 3 hrs buffer index */
	bool battery_monthly_full;		/* status of the monthly buffer full status */
	bool battery_day_full;			/* status of the daily buffer full status */
	bool battery_3hrs_full;			/* status of the 3hrs buffer full status */
} battery_record;

/**
 * @brief battery status initialisation function
 */
void battery_init(void) {
	/* reset the state of battery A */
	battery_status_A.battery_curr_state = battery_normal_st;
	battery_status_A.battery_dead_strike_count = 0u;
	battery_status_A.battery_high_imp_strike_count = 0u;
	battery_status_A.battery_low_strike_count = 0u;

	/* reset the state of battery B */
	battery_status_B.battery_curr_state = battery_normal_st;
	battery_status_B.battery_dead_strike_count = 0u;
	battery_status_B.battery_high_imp_strike_count = 0u;
	battery_status_B.battery_low_strike_count = 0u;

	battery_record.battery_monthly_count = 0u;
	battery_record.battery_day_count = 0u;
	battery_record.battery_3hrs_count = 0u;
	battery_record.battery_monthly_full = false;
	battery_record.battery_day_full = false;
	battery_record.battery_3hrs_full = false;

	battery_periodicity_status = true; 		/* normal periodicity enabled */

#ifdef EEPROM_CALI
  /* Get eeprom integrity? */
  const bool eeprom_ok = data_logging_is_eeprom_ok( );

  /* Can eeprom be trusted? */
  if( eeprom_ok )
  {
		lowBattThres    = DataLogging_GetLowBatteryThreshold();
		deadBattThres   = DataLogging_GetDeadBatteryThreshold();
		strikeBattCount = DataLogging_GetBatteryBistStrikeCount();

		uint8_t deadBistBattPeriod = DataLogging_GetDeadBatteryBistPeriod();
		if(deadBistBattPeriod != BATTERY_FAULT_BIST_PERIOD)
		{
		    deadBistBattPeriod = BATTERY_FAULT_BIST_PERIOD;
		    DataLogging_SetDeadBatteryBistPeriod(deadBistBattPeriod);
		}
  }
#endif

	if((lowBattThres == 0U) || (lowBattThres == 0xFFFFU))
	{
	     lowBattThres = (uint16_t)BATT_LOW_THRESHOLD;
	}

	if((deadBattThres == 0U) || (deadBattThres == 0xFFFFU))
	{
	     deadBattThres = (uint16_t)BATT_DEAD_THRESHOLD;
	}

	if((strikeBattCount == 0U) || (strikeBattCount == 0xFFU))
	{
	    strikeBattCount = (uint8_t)BATT_MAX_STRIKE_COUNT;
	}


}

/**
 * @brief measure the voltage and impedance of 2 batteries in device
 * measure th edefault state of the battery circuit to identify any faults
 * * @Req PTR-1232, PTR-1281, PTR-1246, DCR0046
 */
void battery_measure(void) {
	    batt_A_Measurement(); /* measure the voltage and impedance of battery A */
	    batt_B_Measurement(); /* Measure the voltage and impedance of battery B */
}

/**
 * @brief handle the dead battery condition 
 * @Req PTR-1232, PTR-1281, PTR-1246, DCR0046
 */ 
static void battery_handle_deadbatt_state(void){
	RTOS_ERR err;
	/* if both the batteries are not in normal working condition, set the fault */
	/* TODO: change the system state to 'Battery Error Start' */

	/* Log the status in eeprom */
	if( !battery_circuit_fault )
	{
	  DataLogging_SetEventLogbookRecord( DEF_LBE_BATTERY_ERR_START, NULL );
	  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
		battery_circuit_fault = true;
	}

  if ((battery_status_A.battery_curr_state == battery_inLowVoltFault_st) && (battery_status_B.battery_curr_state == battery_inLowVoltFault_st))
  {
      set_low_battery_status(true);
      LEDBuzz_Post(PatternLowBatt);
  }
  else
  {
      set_low_battery_status(false);
  }
  
	/* Indicate the LED and buzzer status using major fault pattern */

	FaultHandler_FaultSet(BatteryFault); /* indicate major fault            */

	battery_periodicity_status = false; /* fault periodicity enabled */
	DEBUG_BATT("\nBattery meas periodicity change", false, 0u);

	if ((battery_status_A.battery_curr_state == battery_shutdown_st) && (battery_status_B.battery_curr_state == battery_shutdown_st)) {
		/* shutdown the system to avoid any unintended behaviour */
		DEBUG_BATT("\nDead battery fault", false, 0u);

		/* log the shutdown status to eeprom */
		DataLogging_SetEventLogbookRecord( DEF_LBE_BATTERY_SHUTDOWN_START, NULL );

		/* post the shutdown event */
		OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
		EVENT_SHUTDOWN_0, /*   event bit-mask.              */
		OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
		&err);
		/*   Check error code */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}
}

/**
 * @brief function to do the battery BIST
 * measure the battery voltage level. 
 * Read the volatge as well as impedance level
 * if the battery voltage level is more than 3.2V then there is some issue with battery circuit.
 * If battery circuit fault indicate the major fault condition (strike count == 3)
 * if battery voltage is less than 2.7V then low battery condition is identified
 * if battery voltage is less than 2.4V then dead battery condition is identified
 * if battery impedance is more than 4ohm high impedance condition is identified
 * when both the batteries are identified faulty indicate to the user
 * If both the batteries are less than 2.4V then system is shudown to avoid the unintended 
 * behaviour
 * @return none
 * @Req PTR-1232, PTR-1281, PTR-1283, PTR-1246, PTR-1374, PTR-1448, PTR-1446, PTR-1445, PTR-1444
 * 	PTR-1443, PTR-1426, DCR0046
 */
void battery_bist(void)
{

  uint32_t batt_A_voltage = 0u;
	uint32_t batt_B_voltage = 0u;
	uint32_t batt_A_impedance = 0u;
	uint32_t batt_B_impedance = 0u;
	bool change_periodicity24hrsto1min = false;


	batt_A_voltage = get_battery_A_Voltage();
	batt_B_voltage = get_battery_B_Voltage();

	batt_A_impedance = get_battery_A_Impedance();
	batt_B_impedance = get_battery_B_Impedance();

	DEBUG_BATT("\nBattery A voltage (mv):", true, batt_A_voltage);
	DEBUG_BATT("\nBattery A impd(milliohm) :", true, batt_A_impedance);
	DEBUG_BATT("\nBattery B voltage (mv) :", true, batt_B_voltage);
	DEBUG_BATT("\nBattery B impd(milliohm):", true, batt_B_impedance);

  if ((battery_circuit_fault == false) && (((batt_A_voltage > BATT_MAX_VOLTAGE) || (batt_B_voltage > BATT_MAX_VOLTAGE))))
	{
      /* battery circuit fault identified */
      battery_periodicity_status = false; /* change to fault condition periodicity */
      DEBUG_BATT("\n HIGH Voltage observed", false, 0u);
      battery_high_strike_count = battery_high_strike_count + 1u;

      if (battery_high_strike_count > strikeBattCount)
      {
          battery_circuit_fault = true; /* confirm the battery circuit fault */
          /* TODO: Battery circuit hardware error start */
          /* eeprom status update */
          DataLogging_SetEventLogbookRecord( DEF_LBE_BATTERY_ERR_START, NULL );
          /* Indicate the LED and buzzer status using major fault pattern */
          FaultHandler_FaultSet(BatteryFault);
      }
      else
      {
          battery_circuit_fault = false; /* it was a battery glitch. hence clear the fault condition */
      }
	}

	/* check the status of battery A */
  if (batt_A_voltage < lowBattThres)
  {
      change_periodicity24hrsto1min = true;
      if (battery_status_A.battery_curr_state == battery_normal_st)
      {
          battery_status_A.battery_low_strike_count++;
          if (battery_status_A.battery_low_strike_count >= strikeBattCount)
          {
              battery_status_A.battery_curr_state = battery_inLowVoltFault_st; /* battery A in low battery state*/
              DEBUG_BATT("\nBattery A Low battery fault", false, 0u);
          }

      }

      if ((batt_A_voltage <= deadBattThres)	&& (battery_status_A.battery_curr_state != battery_shutdown_st))
      {
          battery_status_A.battery_dead_strike_count++;
          if (battery_status_A.battery_dead_strike_count >= strikeBattCount)
          {
              battery_status_A.battery_curr_state = battery_shutdown_st; /* battery A near to shutdown state */
              DEBUG_BATT("\nBattery A dead battery fault", false, 0u);
           }

      }
      else
      {
          battery_status_A.battery_dead_strike_count = 0u; /* it was a battery glitch. hence clear the fault condition */
      }
  }
	else
  {
	    battery_status_A.battery_low_strike_count = 0u; /* it was a battery glitch. hence clear the fault condition */
  }

  if ((batt_A_impedance >= BATT_HIGH_IMPEDANCE_THRESHOLD) && (battery_status_A.battery_curr_state == battery_normal_st))
  {
      change_periodicity24hrsto1min = true;
      battery_status_A.battery_high_imp_strike_count++;
      DEBUG_BATT("\nWWWWWWWW strike count WWWWWWWWW", true, strikeBattCount);
      if (battery_status_A.battery_high_imp_strike_count >= strikeBattCount)
      {
          battery_status_A.battery_curr_state = battery_inHighImpdFault_st; /* battery A fault confirmed */
          DEBUG_BATT("\nBattery A high impd fault", false, 0u);
      }

  }
  else
  {
      battery_status_A.battery_high_imp_strike_count = 0u; /* it was a battery glitch. hence clear the fault condition */
	}

  if (batt_B_voltage < lowBattThres)
  {
      change_periodicity24hrsto1min = true;
      if (battery_status_B.battery_curr_state == battery_normal_st)
      {   /* If Battery previously was in normal condition */
          battery_status_B.battery_low_strike_count++; /* increment the strike count */
          if (battery_status_B.battery_low_strike_count >= strikeBattCount)
          {
              battery_status_B.battery_curr_state = battery_inLowVoltFault_st; /* battery B in low battery state*/
              DEBUG_BATT("\nBattery B low battery fault", false, 0u);
           }

      }


		  if ((batt_B_voltage <= deadBattThres)	&& (battery_status_B.battery_curr_state != battery_shutdown_st))
		  { /* previously battery was not near shutdown state */
		      battery_status_B.battery_dead_strike_count++; /* increment the strike count */
          if (battery_status_B.battery_dead_strike_count >= strikeBattCount)
          {
              battery_status_B.battery_curr_state = battery_shutdown_st; /* battery B near to shutdown state */
              DEBUG_BATT("\nBattery B dead battery fault", false, 0u);
          }

      }
      else
      {
          battery_status_B.battery_dead_strike_count = 0u; /* it was a battery glitch. hence clear the fault condition */
      }
  }
  else
  {
      battery_status_B.battery_low_strike_count = 0u; /* it was a battery glitch. hence clear the fault condition */
	}

  if ((batt_B_impedance >= BATT_HIGH_IMPEDANCE_THRESHOLD)	&& (battery_status_B.battery_curr_state == battery_normal_st))
  {
      change_periodicity24hrsto1min = true;
      battery_status_B.battery_high_imp_strike_count++;
      if (battery_status_B.battery_high_imp_strike_count >= strikeBattCount)
      {
          battery_status_B.battery_curr_state = battery_inHighImpdFault_st; /* battery A fault confirmed */
          DEBUG_BATT("\nBattery B high impd fault", false, 0u);
      }

  }
  else
  {
      battery_status_B.battery_high_imp_strike_count = 0u; /* clear the fault condition */
	}

  if ((battery_status_A.battery_curr_state != battery_normal_st) && (battery_status_B.battery_curr_state != battery_normal_st))
  {
		DEBUG_BATT("\nBattery Error Start", false, 0u);
		battery_handle_deadbatt_state(); 	/* call the dead battery state handler */
	}

  if(change_periodicity24hrsto1min)
  {
     if(timerChangePeriodicityOnce == false)
     {
        /* change the periodicity to 1 minute for the faster monitor */
        BURTCTimer_Stop(TMR_Battery_Measurement_BIST_event_0); /* stop the current periodic timer*/
        BURTCTimer_Start(TMR_Battery_Measurement_BIST_event_0, periodical, BATTERY_FAULT_BIST_PERIOD); /* New periodicity is per minute when both the batteries are faulty */
        timerChangePeriodicityOnce = true;
      }
  }
  else
  {
      /* if battery condition recovers then change periodicity to 24hrs*/
      if(timerChangePeriodicityOnce == true)
      {
          BURTCTimer_Stop(TMR_Battery_Measurement_BIST_event_0); /* stop the current periodic timer*/
          BURTCTimer_Start(TMR_Battery_Measurement_BIST_event_0, periodical, BATTERY_MEAS_BIST_PERIOD);
          timerChangePeriodicityOnce = false;
      }
  }
}

static uint32_t get_battery_mV( const uint32_t adc_counts )
{
	uint32_t mV = ( adc_counts * BATT_MAX_ADC_REF_GAIN ) >> BATT_ADC_RESOLUTION;

  mV = ( uint32_t )hal_AFE_abuf_correct( mV );

  if(mV > BATT_MAX_VOLTAGE)
  {
      mV = BATT_MAX_VOLTAGE;
  }

  return( mV );
}

/**
 * @brief Get A battery voltage from ADC count
 * @return battery voltage in mv (precision batt_mv)
 */
uint32_t get_battery_A_Voltage( void )
{
	const uint32_t batt_volt = get_battery_mV( adc_battery_A );

	DEBUG_BATT("\n Batt A in mV:", true, batt_volt);
	
	return( batt_volt );
}

/**
 * @brief Get B battery voltage ADC count
 * @return battery voltage in mv
 */
uint32_t get_battery_B_Voltage( void )
{
	const uint32_t batt_volt = get_battery_mV( adc_battery_B );

	DEBUG_BATT("\n Batt B in mV:", true, batt_volt);
	
	return( batt_volt );
}

/**
 * @brief Set Simulated FTM battery A voltage in millvolt
 */
void set_ftm_Batt_A_Sim_Vol(uint32_t vol)
{
    adc_battery_A = vol;
}

/**
 *  * @brief Set Simulated FTM battery A voltage in millvolt
 */
void set_ftm_Batt_B_Sim_Vol(uint32_t vol)
{
    adc_battery_B = vol;
}

/**
 * @brief Get Simulated FTM battery A voltage in millvolt
 */
uint32_t get_ftm_Batt_A_Sim_Vol(void)
{
   return adc_battery_A;
}

/**
 * @brief Get Simulated FTM battery A voltage in millvolt
 */
uint32_t get_ftm_Batt_B_Sim_Vol(void)
{
  return adc_battery_B;
}

/**
 * @brief get battery A impedance value
 * @return impedance value in milliohms
 */
uint32_t get_battery_A_Impedance(void) {
	DEBUG_BATT("\n Batt A in mOhm:", true, batt_impedance_A);
	return batt_impedance_A;
}

/**
 * @brief get battery B impedance value
 * @return impedance value in milliohms
 */
uint32_t get_battery_B_Impedance(void)
{
  DEBUG_BATT("\n Batt B in mOhm:", true, batt_impedance_B);
  return batt_impedance_B;
}

/**
 * @brief store the periodici battery voltage and impedance values into the history record.
 * requirement is to store the monthly average of battery voltage and impedance values
 * for both the batteries.
 * when periodicity is normal event 24hrs battery measurement is made and history record
 * is updated.
 * when there is a fault increase the periodicity to 1 minute. 
 * with 1 minute periodicity we are calculating the monthly averageing in multiple steps.
 * this is to limit the buffer size.
 * First using the 1 minute batteyr data, calculate the 3 hrs averaging.
 * use the 3 hrs average data to calculate the daily average.
 * daily average is useed to populate the monthly history buffer.
 * monthly average buffer when full, calculate the average and store into eeprom.
 * 
 * @req DCR0046
 */
void battery_store_data(void) {
	uint8_t count = 0u;

	uint32_t battery_A_voltage_sum = 0u;
	uint32_t battery_A_impedance_sum = 0u;
	uint32_t battery_B_voltage_sum = 0u;
	uint32_t battery_B_impedance_sum = 0u;

	uint16_t battery_A_daily_volt = 0u;
	uint16_t battery_A_daily_impedance = 0u;
	uint16_t battery_B_daily_volt = 0u;
	uint16_t battery_B_daily_impedance = 0u;

	uint16_t battery_A_volt_3hrs = 0u;
	uint16_t battery_A_impedance_3hrs = 0u;
	uint16_t battery_B_volt_3hrs = 0u;
	uint16_t battery_B_impedance_3hrs = 0u;

	/* if the periodicity is normal 24hours */
	if (battery_periodicity_status == true) {
		/* update the history array with battery data */
		battery_record.battery_A_voltage_30days[battery_record.battery_monthly_count]   = (uint16_t) get_battery_A_Voltage();
		battery_record.battery_A_impedance_30days[battery_record.battery_monthly_count] = (uint16_t) get_battery_A_Impedance();
		battery_record.battery_B_voltage_30days[battery_record.battery_monthly_count]   = (uint16_t) get_battery_B_Voltage();
		battery_record.battery_B_impedance_30days[battery_record.battery_monthly_count] = (uint16_t) get_battery_B_Impedance();

		battery_record.battery_monthly_count = battery_record.battery_monthly_count + 1u; /* increament the monthly buffer index */
		if (battery_record.battery_monthly_count == BATTERY_30DAYS_DATA) {
			battery_record.battery_monthly_full = true; /* monthly buffer is full */
			DEBUG_BATT("\n Monthly buffer full", false, 0u);
		} else {
			/* do nothing */
		}
	} else {
		/* update the history array with battery data */
		battery_record.battery_A_voltage_3hrs[battery_record.battery_3hrs_count]   = (uint16_t) get_battery_A_Voltage();
		battery_record.battery_A_impedance_3hrs[battery_record.battery_3hrs_count] = (uint16_t) get_battery_A_Impedance();
		battery_record.battery_B_voltage_3hrs[battery_record.battery_3hrs_count]   = (uint16_t) get_battery_B_Voltage();
		battery_record.battery_B_impedance_3hrs[battery_record.battery_3hrs_count] =(uint16_t) get_battery_B_Impedance();

		battery_record.battery_3hrs_count = battery_record.battery_3hrs_count
				+ 1u;     /* update the 3 hrs array with battery data */
		if (battery_record.battery_3hrs_count == BATTERY_3HRS_DATA) {
			battery_record.battery_3hrs_full = true; /* 3hrs buffer is full */
			DEBUG_BATT("\n3hrs history full", false, 0u);

			/* calculate the sum of battery 3 hrs history data */
			for (count = 0u; count < BATTERY_3HRS_DATA; count++) {
				battery_A_voltage_sum = battery_A_voltage_sum + battery_record.battery_A_voltage_3hrs[count];
				battery_A_impedance_sum = battery_A_impedance_sum + battery_record.battery_A_impedance_3hrs[count];
				battery_B_voltage_sum = battery_B_voltage_sum + battery_record.battery_B_voltage_3hrs[count];
				battery_B_impedance_sum = battery_B_impedance_sum + battery_record.battery_B_impedance_3hrs[count];
			}

			battery_record.battery_3hrs_full = false; /* reset the full status */
			battery_record.battery_3hrs_count = 0u; /* reset the buffer index */

			/* calculate the battery data of 3 hours average */
			battery_A_volt_3hrs = (uint16_t) (battery_A_voltage_sum / BATTERY_3HRS_DATA);
			battery_A_impedance_3hrs = (uint16_t) (battery_A_impedance_sum / BATTERY_3HRS_DATA);
			battery_B_volt_3hrs = (uint16_t) (battery_B_voltage_sum / BATTERY_3HRS_DATA);
			battery_B_impedance_3hrs = (uint16_t) (battery_B_impedance_sum / BATTERY_3HRS_DATA);

			/* store the 3 hrs average data into daily buffer */
			battery_record.battery_A_voltage_1day[battery_record.battery_day_count]   = battery_A_volt_3hrs;
			battery_record.battery_A_impedance_1day[battery_record.battery_day_count] = battery_A_impedance_3hrs;
			battery_record.battery_B_voltage_1day[battery_record.battery_day_count]   = battery_B_volt_3hrs;
			battery_record.battery_B_impedance_1day[battery_record.battery_day_count] = battery_B_impedance_3hrs;
			battery_record.battery_day_count = battery_record.battery_day_count + 1u;

			if (battery_record.battery_day_count == BATTERY_1DAY_DATA) {
				battery_record.battery_day_full = true; /* daily average buffer is full */
				DEBUG_BATT("\nDaily history full", false, 0u);

				/* calculate the sum of battery daily history data */
				for (count = 0u; count < BATTERY_1DAY_DATA; count++) {
					battery_A_voltage_sum = battery_A_voltage_sum + battery_record.battery_A_voltage_1day[count];
					battery_A_impedance_sum = battery_A_impedance_sum + battery_record.battery_A_impedance_1day[count];
					battery_B_voltage_sum = battery_B_voltage_sum + battery_record.battery_B_voltage_1day[count];
					battery_B_impedance_sum = battery_B_impedance_sum + battery_record.battery_B_impedance_1day[count];
				}

				/* calculate the battery data of 3 hours average */
				battery_A_daily_volt = (uint16_t) (battery_A_voltage_sum / BATTERY_1DAY_DATA);
				battery_A_daily_impedance = (uint16_t) (battery_A_impedance_sum / BATTERY_1DAY_DATA);
				battery_B_daily_volt = (uint16_t) (battery_B_voltage_sum / BATTERY_1DAY_DATA);
				battery_B_daily_impedance = (uint16_t) (battery_B_impedance_sum / BATTERY_1DAY_DATA);

				battery_record.battery_day_full = false; /* reset the full status */
				battery_record.battery_day_count = 0u; /* reset the buffer status */

				DEBUG_BATT("\battery_A_daily_volt", true, battery_A_daily_volt);
				DEBUG_BATT("\battery_A_daily_impedance", true, battery_A_daily_impedance);
				DEBUG_BATT("\battery_B_daily_volt", true, battery_B_daily_volt);
				DEBUG_BATT("\battery_B_daily_impedance", true, battery_B_daily_impedance);

				/* update the history array with battery data */
				battery_record.battery_A_voltage_30days[battery_record.battery_monthly_count]   = battery_A_daily_volt;
				battery_record.battery_A_impedance_30days[battery_record.battery_monthly_count] = battery_A_daily_impedance;
				battery_record.battery_B_voltage_30days[battery_record.battery_monthly_count]   = battery_B_daily_volt;
				battery_record.battery_B_impedance_30days[battery_record.battery_monthly_count] = battery_B_daily_impedance;

				battery_record.battery_monthly_count = battery_record.battery_monthly_count + 1u;
				if (battery_record.battery_monthly_count == BATTERY_30DAYS_DATA) {
					battery_record.battery_monthly_full = true; /* monthly buffer is full */
					DEBUG_BATT("\nMonthly history full", false, 0u);
				} else {
					/* do nothing */
				}
			}
		}
	}
}

/**
 * @brief calculate the monthly battery data average
 * @req DCR0046
 */
void battery_calculate_monthly_average(void) {
	uint8_t count = 0u;
	uint32_t battery_A_voltage_sum = 0u;
	uint32_t battery_A_impedance_sum = 0u;
	uint32_t battery_B_voltage_sum = 0u;
	uint32_t battery_B_impedance_sum = 0u;

	uint16_t battery_A_monthly_volt = 0u;
	uint16_t battery_A_monthly_impedance = 0u;
	uint16_t battery_B_monthly_volt = 0u;
	uint16_t battery_B_monthly_impedance = 0u;

	dl_monthly_batt_level_t level;
	dl_monthly_batt_impedance_t impedance;

	/* calculate the monthly battery voltage and impedance levels */
	if (battery_record.battery_monthly_full == true) {
		for (count = 0u; count < BATTERY_30DAYS_DATA; count++) {
			battery_A_voltage_sum = battery_A_voltage_sum + battery_record.battery_A_voltage_30days[count];
			battery_A_impedance_sum = battery_A_impedance_sum + battery_record.battery_A_impedance_30days[count];
			battery_B_voltage_sum = battery_B_voltage_sum + battery_record.battery_B_voltage_30days[count];
			battery_B_impedance_sum = battery_B_impedance_sum + battery_record.battery_B_impedance_30days[count];
		}

		/* calculate the 30days average of battery data */
		battery_A_monthly_volt = (uint16_t) (battery_A_voltage_sum / BATTERY_30DAYS_DATA);
		battery_A_monthly_impedance = (uint16_t) (battery_A_impedance_sum / BATTERY_30DAYS_DATA);
		battery_B_monthly_volt = (uint16_t) (battery_B_voltage_sum / BATTERY_30DAYS_DATA);
		battery_B_monthly_impedance = (uint16_t) (battery_B_impedance_sum / BATTERY_30DAYS_DATA);

		DEBUG_BATT("\nbatt_A_volt_sum", true, battery_A_voltage_sum);
		DEBUG_BATT("\nbatt_A_impd_sum", true, battery_A_impedance_sum);
		DEBUG_BATT("\nbatt_B_volt_sum", true, battery_B_voltage_sum);
		DEBUG_BATT("\nbatt_B_impd_sum", true, battery_B_impedance_sum);

		DEBUG_BATT("\nbatt_A_month_volt", true, battery_A_monthly_volt);
		DEBUG_BATT("\nbatt_A_month_impd", true, battery_A_monthly_impedance);
		DEBUG_BATT("\nbatt_B_month_volt", true, battery_B_monthly_volt);
		DEBUG_BATT("\nbatt_B_month_impd", true, battery_B_monthly_impedance);

		battery_record.battery_monthly_full = false; /* mark the monthly record status to not full */
		battery_record.battery_monthly_count = 0u; /* reset the history array index */

		/* store the battery monthly average values into eeprom */
		level.Batt_A = battery_A_monthly_volt;
		level.Batt_B = battery_B_monthly_volt;
		DataLogging_SetMonthlyBatteryLevel(&level);

		impedance.Batt_A = battery_A_monthly_impedance;
		impedance.Batt_B = battery_B_monthly_impedance;
		DataLogging_SetMonthlyBatteryImpedance(&impedance);

	} else {
		/* calculate the average only when we have 30 days data */
	}
}


uint32_t get_ADC_Battery_A()
{
    return adc_battery_A;
}
uint32_t get_ADC_Battery_B()
{
    return adc_battery_B;
}

void set_ADC_Battery_A(uint32_t value)
{
    adc_battery_A = value;
}

void set_ADC_Battery_B(uint32_t value)
{   
    adc_battery_B = value;
}

void set_Batt_A_Impedance(uint32_t value)
{
    batt_impedance_A = value;
}

void set_Batt_B_Impedance(uint32_t value)
{
    batt_impedance_B = value;
}

bool get_Battery_Periodicity_Status()
{
    return battery_periodicity_status;
}

uint8_t getStrikeCount(void)
{
  return strikeBattCount;
}
uint16_t getLowBattThres(void)
{
  return lowBattThres;
}

bool is_battery_circuit_fault( void )
{
	return( battery_circuit_fault );
}

void set_battery_circuit_fault( void )
{
	battery_circuit_fault = true;
}

void set_low_battery_status(bool status)
{
  lowBattStatus = status;
}

bool get_low_battery_status(void)
{
  return lowBattStatus;
}
