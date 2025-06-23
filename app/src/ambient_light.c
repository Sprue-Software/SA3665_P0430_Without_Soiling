/*
 * temp_humid.c
 *
 *  Created on: 6 october 2022
 *      Author: esambi
 */
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "os.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "hal_i2c.h"
#include "hal_AFE.h"
#include "em_iadc.h"
#include "hal_gpio.h"
#include "ambient_light.h"
#include "data_logging.h"
#include"spi_comms.h"
#include "hal_BURTCTimer.h"

static uint8_t light_status = (uint8_t)AMBIENT_LEVEL_BRIGHTNESS;
static uint8_t previous_light_status = (uint8_t)AMBIENT_LEVEL_BRIGHTNESS;
static uint32_t threshold_hys_brightness_buffer = 0U;
static uint32_t ambient_light_adc = 0U;
static uint32_t ambient_ftm_status = 0U;
static bool seven_days_darkness_status = false;

/**
 * @brief this function reads ambient light threshold value from eeprom
 */
void ambient_light_init(void) {
  threshold_hys_brightness_buffer = AMBIENT_THRESHOLD_VAL + (AMBIENT_THRESHOLD_VAL/AMBIENT_LIGHT_HYS_BUFFER_THRESHOLD);
  dl_ambient_light_cfg_data_t amb;
  DataLogging_GetAmbientLightConfig(&amb);
  if((amb.AmbientLightThreshold == 0U) || (amb.AmbientLightThreshold == 0xFFFFU))
  {
      amb.AmbientLightThreshold = (uint16_t)threshold_hys_brightness_buffer;
      amb.AmbientLightAcqPeriod = 6U;
      amb.AmbientLightBistPeriod = 0U;
      amb.AmbientLightValMin = 0U;
      amb.AmbientLightValMax = 0U;
      DataLogging_SetAmbientLightConfig(&amb);
  }
}

/*******************************************************************************
 * @brief ambient light measure
 * @details This function measures ambient light status and hardware error
 * @param uint32_t delay, uint8_t sample_count, uint16_t threshold_val,
 * @return Null
 * @req PTR-475
 ******************************************************************************/
void ambient_light_measure(uint32_t delay, uint8_t sample_count)
{
  uint32_t ADC_data                 = 0U;
  uint32_t millivolt                = 0U;
  uint32_t millivolt_average        = 0U;

  static bool dark_timer_running    = false;

  static uint8_t strike_count       = 3U;

  if( delay == 0U )
  {
    delay = 1000U;
  }

  hal_gpio_disable_ambient_light_power( );

  hal_AFE_AcquireADC( );

  for( uint8_t counter = 0U; counter < strike_count; counter++ )
  {
    hal_AFE_ADC_Setup( setup_FW_TEST_Adc_GPIO_LIGHT );

    /* Add delay only at first strike count. */
    if( counter == 0U )
    {
      hal_gpio_enable_ambient_light_power( );

      hal_AFE_delay_ms( delay );  /* delay to settling down light sensor */
    }

    for( uint8_t ctr = 0U; ctr < sample_count; ctr++ )
    {
      ADC_data = Measure_ADC( );

      millivolt = ( ( ADC_data * ADC_REF ) / ADC_16_BIT );

      millivolt_average += millivolt;
    }

    millivolt_average = millivolt_average / sample_count;

    ambient_light_adc = millivolt_average;

    if( millivolt_average >= threshold_hys_brightness_buffer )
    {
      light_status = ( uint8_t )AMBIENT_LEVEL_BRIGHTNESS;

      DEBUG_AMBIENT( "\n AMBIENT BRIGHTNESS - (mV):", true, millivolt_average );

      /* Is darkness timer running? */
      if( dark_timer_running )
      {
        /* Forget checking for 7 days of darkness */
        BURTCTimer_Stop( TMR_AmbientLight_7days_darkness_1 );
        /* Timer stopped */
        dark_timer_running = false;
        set_SevenDays_Darkness_Status(false);

      }
    }
    else
    {
      light_status = (uint8_t) AMBIENT_LEVEL_DARKNESS;

      /* Is darkness timer running? */
      if( !dark_timer_running )
      {
        /* Just a one shot */
        const bool periodic = false;

        /* Start 7 day timer */
        BURTCTimer_Start( TMR_AmbientLight_7days_darkness_1, periodic, AMBIENT_LIGHT_7DAYS_PERIOD );

        /* Timer running */
        dark_timer_running = true;
      }

      DEBUG_AMBIENT( "\n AMBIENT DARKNESS - (mV):", true, millivolt_average );
    }

    millivolt_average = 0U;

    /* If light level hasn't changed break from strike count. */
    if( light_status == previous_light_status )
    {
      break;
    }
    else
    {
        SPIComms_Send_Data_to_MCU2( SPI_CMD_Current_value );
    }
  }

  Stop_ADC();

  hal_AFE_ReleaseADC();
  previous_light_status = light_status;

  hal_gpio_disable_ambient_light_power( );
}

/*******************************************************************************
 * @brief ambient_light_get_status
 * @details This function measures ambient light status.
 * @param Null
 * @return (uint8_t) light_level
 * @req PTR-475
 ******************************************************************************/
uint8_t ambient_light_get_status(void) {
  return light_status;
}

/*******************************************************************************
 * @brief ambient_light_get_BIST
 * @details This function measures ambient light hardware error.
 * @param Null
 * @return (uint8_t) light_level
 * @req PTR-475
 ******************************************************************************/
bool ambient_light_get_BIST( const bool perform_measurement )
{
  bool ambient_light_bist_status = false;

  if( perform_measurement )
  {
    ambient_light_measure( AMBIENT_LIGHT_SETLING_TIME, AMBIENT_LIGHT_SAMPLE_COUNT );
  }

  if( light_status != ( uint8_t )AMBIENT_LEVEL_ERROR )
  {
      ambient_light_bist_status = true;
  }
  else
  {
      ambient_light_bist_status = false;
  }

  return ambient_light_bist_status;
}

/*******************************************************************************
 * @brief ambient_light_set_status
 * @details This function set ambient light status for FTM mode.
 * @param uint8_t status
 * @return N/A
 ******************************************************************************/
void ambient_light_set_status(uint8_t status) {
    if(status == 0U)
    {
        light_status = (uint8_t) AMBIENT_LEVEL_BRIGHTNESS;
    }
    else
    {
        light_status = (uint8_t) AMBIENT_LEVEL_DARKNESS;
    }
}

/*******************************************************************************
 * @brief ambient_light_ftm_data
 * @details This function is FTM for ambient light data send to MCU2 over SPI.
 * @param ambient_ftm_Data
 * @return N/A
 ******************************************************************************/
void ambient_light_ftm_data(ambient_ftm_Data *data) {
  data->ambient_status = ambient_ftm_status;
  data->adc_avg_val = ambient_light_adc;
}

/*******************************************************************************
 * @brief get_Ambient_Light_Status
 * @details This function measures ambient light status.
 * @param Null
 * @return true =bright & false =dark

 ******************************************************************************/
bool get_Ambient_Light_Status(void)
{
  bool status=false;
  if (ambient_light_get_status()==AMBIENT_LEVEL_BRIGHTNESS)
  {
      status=true;
  }
  return status;
}

/*******************************************************************************
 * @brief get_SevenDays_Darkness_Status
 * @details This function return 7 days of darkness status.
 * @param Null
 * @return true =bright & false =dark

 ******************************************************************************/
bool get_SevenDays_Darkness_Status(void)
{
    return seven_days_darkness_status;
}

/*******************************************************************************
 * @brief get_Ambient_Light_Status
 * @details This function measures ambient light status.
 * @param Null
 * @return true =bright & false =dark

 ******************************************************************************/
void set_SevenDays_Darkness_Status(bool status)
{
    seven_days_darkness_status = status;
}

