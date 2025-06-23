/***************************************************************************//**
 * @file  timeHandler.c
 * @brief Handle the monitoring of the time since production
 * @project P0200 Techem Core Firmware
 * @date    6 April 2022
 * @author  abenrashed
 *******************************************************************************/
#include "acquisition_Smoke.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/source/os_priv.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>

#include "em_core.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "timeHandler.h"
#include "hal_BURTCTimer.h"
#include "events.h"
#include "app.h"
#include "battery_measurement.h"
#include "temp_humid.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "spi_comms.h"
#include "telegram.h"
#include "acquisition_Co.h"
#include "production.h"
#include "led_buzzer.h"


uint8_t spi_fail_Count=0u;
#define POWER_UP_SCRATCHPAD_MARK	(uint32_t)(0xDEADBEEFu)

/*These variables are placed in .noinit sector so that they are not cleared to 0 upon system reset*/
static volatile uint32_t currentTime __attribute__((section(".noinit"))) ; /** Main System Time counter,  Unit = seconds */
static uint32_t powerUpScratchPad __attribute__((section(".noinit"))); /** Scratch Pad of the Hot power up.
																			This will be scratched with a special mark and clears the time counter
																			so that subsequent resets will not clear the counter*/

static uint32_t commissioningTime = 0u;
static uint32_t userBistTime = 0u;

/**
 * Set the current time
 * @req
 * @brief sets the current time.
 * @param time: minutes since production
 */
void time_setCurrentTime(uint32_t time)
{
	currentTime = time;
	/* This will override the previous timestamp and update it with the time reference*/
	DataLogging_SetLatestTimestamp(currentTime);
}

/**
 * @brief Handle the update of the current time
 * @req PTR-1283, DCR0046
 */
void time_updateTimestamp(void)
{
  currentTime+=10;
}


void time_handleTimestamp(void)
{

  uint32_t deviceAge;
  uint32_t prod_flag = 0U;

	if (get_Battery_Periodicity_Status() == false)
	{
		DEBUG_TIME("\n fault periodicity", false, 0u);
		battery_store_data(); /* update the battery as per fault periodicity */
		battery_calculate_monthly_average();
	}

	if ((currentTime % TIME_HOUR) < 10u)
	{
		/*Update to EEPROM every hour*/
	  DataLogging_SetLatestTimestamp(currentTime);
	  DEBUG_TIME("\n <<<< HR >>>>", false, 0u);
		smoke_chamber_test = true; /* hourly smoke chamber test */
		calculateHumidityAvg(); /* Get the humidity every Hour */
	}

	if ((currentTime % TIME_DAY) < 10u)
	{
		/* normal periodicity, update the battery measurement every day for monthly averaging */
		if (get_Battery_Periodicity_Status() == true)
		{
			DEBUG_TIME("\n Normal periodicity time:", true, currentTime);
			battery_store_data(); /* update the battery as per normal periodicity */
			battery_calculate_monthly_average();
		}
	}

	if ((currentTime % TIME_MONTH) < 10u)
	{
		/* ABR Add log monthly required data */
	}

	prod_flag = prod_get_value();

  if((prod_flag == PROD_COMP_BB) || (prod_flag == PROD_COMP_AA))
	{
		deviceAge = currentTime - get_manufactureDate();

		if (deviceAge >= TIME_EOL)
		{
			uint32_t Faults = FaultHandler_GetFaultFlags();

			if ((Faults & DEF_CO_EOL_FAULT) == 0u)
			{
				/*When End Of Life, take required action*/
		    	FaultHandler_FaultSet(COEndOfLifeFault);
			}
		}
	}
	else
	{
	    /*Do noting*/
	}
}

/**
 * Return the current time
 * @return
 */
uint32_t get_currentTime(void) {
	uint32_t currTime = currentTime;	//Get a Local Copy
	return currTime;
}


/**
 * Initialize the system time handler
 * @return
 */
void time_initTime(void)
{
	/*Check if power-up*/
	if(powerUpScratchPad == POWER_UP_SCRATCHPAD_MARK)
	{
		/*This is a cold power-up, Ignore*/
	}
	else 
	{
		/* Write scratch the pad & read the last time-stamp from EEPROM */
		powerUpScratchPad = POWER_UP_SCRATCHPAD_MARK;

    /* Get last timestamp written to the EEPROM */
		const uint32_t eeprom_time = DataLogging_GetLatestTimestamp( );

    /* Is this valid? */
		if( eeprom_time == 0xFFFFFFFF )
		{
      /* No, reset current time */
			currentTime = 0ul;
		}
    else
    {
      /* Use last written EEPROM time */
			currentTime = eeprom_time;
    }
	}

  DEBUG_PRINTF( "Current Time = %lu", currentTime );
}

/*******************************************************************************
 * @brief set_commissioning_time
 * @details this method records time for every commissioning event
 * @param uint32-t time : current system time
 * @return n/a
 ******************************************************************************/
void set_commissioning_time(uint32_t time)
{
  commissioningTime = time;
}

/*******************************************************************************
 * @brief get_commissioning_time
 * @details this method is being called from SPI cmd, counter and dates to get recorded commissioning mode time
 * @param n/a
 * @return uint32_t commissioningTime
 ******************************************************************************/
uint32_t get_commissioning_time(void)
{
  return commissioningTime;
}

/*******************************************************************************
 * @brief set_userBistTest_time
 * @details this method records current time on every user bist test (standard or extended)
 * @param uint32-t time : current system time
 * @return n/a
 ******************************************************************************/
void set_userBistTest_time(uint32_t time)
{
  userBistTime = time;
}

/*******************************************************************************
 * @brief get_userBistTest_time
 * @details this method is being called from SPI cmd, counter and dates to get recorded user bist time
 * @param n/a
 * @return uint32_t userBistTime
 ******************************************************************************/
uint32_t get_userBistTest_time(void)
{
  return userBistTime;
}
