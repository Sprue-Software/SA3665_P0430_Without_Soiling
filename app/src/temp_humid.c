/*
 * temp_humid.c
 *
 *  Created on: 28 Apr 2022
 *      Author: uhegde
 *      NDI This Higher level application is modified for sht41  ( Use as wrapper driver for sht41)
 */
#include <stdbool.h>
#include <SHT41.h>
#include "em_gpio.h"
#include "sl_sleeptimer.h"
#include "temp_humid.h"
#include "hal_i2c.h"
#include "sht4x.h"
#include "data_logging.h"
#include "system_events.h"
#include "led_buzzer.h"
#include "fault_handler.h"


#define DATA_24HR (24u)
#define TEMP_HUMID_HW_FAULT_STRIKE_COUNT  (3u)  /* number of measurements to be taken before confirming the hardware fault */
#define TEMP_VALUE_MAX      (7000)              /* Maximum temperature limit, above which out of bound fault is set */
#define TEMP_VALUE_MIN      (-2000)             /* Minimum temperature limit, below which out of bound fault is set */
#define HUMID_VALUE_MAX     (100u)              /* Maximum humidity limit, above which out of bound fault is set */
#define HUMID_VALUE_MIN     (0u)                /* Minimum humidity limit, below which out of bound fault is set */


static bool buffer_full_day=false;
uint8_t HUM_Avg_24_hrs[DATA_24HR]; /* Buffer for storing 24hr data*/

static uint8_t fill_data_index=0U;
static int32_t temp_val = 0; /* measured temperature */
static uint32_t humid_val = 0U;                 /* measured humidity */
static uint8_t ftm_sub_command = 0U;
uint8_t temp_humid_fault_count = 0U;            /* counter to measure the number of consecutive sensor fault conditions observed */
uint8_t temp_ob_fault_count = 0U;               /* counter to measure the number of consecutive temp out of bound fault conditions observed */
uint8_t humid_ob_fault_count = 0U;              /* counter to measure the number of consecutive humid out of bound fault conditions observed */

static dl_temperature_cfg_data_t temp;
static dl_humidity_cfg_data_t hum;


void temp_humid_init(void)
{

  DataLogging_GetTempConfig(&temp);
  DataLogging_GetHumidityConfig(&hum);

  if((temp.TemperatureValMax == 0U) || (temp.TemperatureValMax == 0xFFFFU))
  {
      temp.TemperatureValMax = (uint16_t)TEMP_VALUE_MAX;
      temp.TemperatureValMin = (uint16_t)2000;
      temp.TemperatureBistPeriod = 2U;
      DataLogging_SetTempConfig(&temp);
  }

  if((hum.HumidityValMax == 0U) || (hum.HumidityValMax == 0xFFU))
  {
     hum.HumidityValMax =  (uint8_t)HUMID_VALUE_MAX;
     hum.HumidityValMin = 0U;
     hum.HumidityBistPeriod = 2U;
     DataLogging_SetHumidityConfig(&hum);
  }

}


/**
 * brief measure the temperature and humidity values from the sensor and update the result in global
 * variables. Use hdc2022_get_temperature(), hdc2022_get_humidity() to read te results
 */
bool temp_humid_measure_bist(bool *bistCheck)
{
  bool status = false;
  bool hw_fault_status = false;
  static bool hw_log_error = false;
  static bool log_TempOutOfBound = false;
  static bool log_HumidOutOfBound = false;
  uint8_t hw_check = 0U;
  uint8_t temp_ob_check = 0U;
  uint8_t humid_ob_check = 0U;
  RTOS_ERR err;

  status = SHT41_measure(); /* Measure the current humidity and temperature */

  uint32_t Faults = FaultHandler_GetFaultFlags();
  /* hardware BIST check */
  if ((status == false) && ((Faults & DEF_TEMP_SENSOR_HW_FAULT) == 0u))
  {
      hw_check = 1U;
      temp_humid_fault_count++;
      if (temp_humid_fault_count >= TEMP_HUMID_HW_FAULT_STRIKE_COUNT)
      {
          /* TODO: set the sensor hardware error start fault condition */
          if(hw_log_error == false)
          {
            DataLogging_SetEventLogbookRecord( DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_START, NULL ); /* PTR-1287 */
            FaultHandler_FaultSet(TemperatureSensorHwFault);
            FaultHandler_FaultSet(HumiditySensorHwFault);
            hw_log_error = true;
      	  }
          DEBUG_TEMP_HUMID("temp-humid fault!!!", false, 0u);
          hw_fault_status = true; /* sensor hardware fault */
      }
  }
  else if((Faults & DEF_TEMP_SENSOR_HW_FAULT) != 0u)
  {
      /* if the fault is previously set only return the HW status */
      hw_fault_status = true; /* sensor hardware fault */
      hw_check = 1U;
  }
  else
  {
      temp_humid_fault_count = 0u;
      /* TODO: clear the sensor fault condition */
      if(hw_log_error == true)
      {
          DataLogging_SetEventLogbookRecord( DEF_LBE_TEMP_HUMID_SENSOR_HW_ERR_END, NULL ); /* PTR-1287 */
          hw_log_error = false;
      }
      hw_fault_status = false;

      /* out of bound fault condition check */
      temp_val = (int32_t) SHT41_get_temperature(); /* get the temperature value */
      humid_val = SHT41_get_humidity(); /* get the humidity value */
      DEBUG_TEMP_HUMID("Temp values ", 1, temp_val);
      DEBUG_TEMP_HUMID("Humidity values ", 1, humid_val);
      if(temp_val < (((int32_t)temp.TemperatureValMin)*(-1)) || temp_val >= (int32_t)temp.TemperatureValMax)
      {
          /* TODO: set the temperature-humidity sensor out of bound fault */
          temp_ob_check = 1U;
          if(log_TempOutOfBound == false)
          {
              temp_ob_fault_count++;
              if((temp_ob_fault_count >= TEMP_HUMID_HW_FAULT_STRIKE_COUNT))
              {
                  DataLogging_SetEventLogbookRecord( DEF_LBE_TEMP_OOR_START, NULL ); /* PTR-1287 */
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                  FaultHandler_FaultSet(TempSensorOutOfBoundsFault);
                  log_TempOutOfBound = true;
                  DataLogging_SetMinorFault(FaultTemperatureOutOfBound, log_TempOutOfBound);
              }
              DEBUG_TEMP_HUMID("temperature out-of-bound!!!", false, 0u);
          }
      }
      else
      {
          /* TODO: clear the sensor out of bound fault status */
          temp_ob_fault_count = 0U;
          if(log_TempOutOfBound == true)
          {
              DataLogging_SetEventLogbookRecord( DEF_LBE_TEMP_OOR_END, NULL ); /* PTR-1287 */
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              FaultHandler_FaultClear(TempSensorOutOfBoundsFault);
              log_TempOutOfBound = false;
              DataLogging_SetMinorFault(FaultTemperatureOutOfBound, log_TempOutOfBound);
          }
      }

      if (humid_val > hum.HumidityValMax)
      {
          /* < 0 condition is not checked as unsigned humid_val cannot be negative */
          /* TODO: set the temperature-humidity sensor out of bound fault */
          humid_ob_check = 1U;
          if((log_HumidOutOfBound == false))
          {
              humid_ob_fault_count++;
              if((humid_ob_fault_count >= TEMP_HUMID_HW_FAULT_STRIKE_COUNT))
              {
                  DataLogging_SetEventLogbookRecord( DEF_LBE_HUMID_OOR_START, NULL ); /* PTR-1288 */
                  FaultHandler_FaultSet(HumiditySensorOutOfBoundsFault);
                  log_HumidOutOfBound = true;
                  DataLogging_SetMinorFault(FaultHumidityOutOfBound, log_HumidOutOfBound);
              }
              DEBUG_TEMP_HUMID("humidity out-of-bound!!!", false, 0u);
          }
      }
      else
      {
          /* TODO: clear the sensor out of bound fault status */
          humid_ob_fault_count = 0U;
          if(log_HumidOutOfBound == true)
          {
              DataLogging_SetEventLogbookRecord( DEF_LBE_HUMID_OOR_END, NULL ); /* PTR-1288 */
              FaultHandler_FaultClear(HumiditySensorOutOfBoundsFault);
              log_HumidOutOfBound = false;
              DataLogging_SetMinorFault(FaultHumidityOutOfBound, log_HumidOutOfBound);
          }
      }
  }

  if((hw_check) || (temp_ob_check) || (humid_ob_check))
  {
      *bistCheck = true;
  }
  else
  {
      *bistCheck = false;
  }

  return hw_fault_status;
}


/**
 *
 * @brief This function will return Humidity Average of 24 hours
 *  @return Average  Humidity
 */
uint16_t getdayHumidityAvg(void)
{
  uint16_t total_Humidity_24hrs = 0u;
  static uint16_t avg_24_hr_humidity;
  static uint16_t avg_reading = 0xffu;
#if 1
  /* Humidity Will be updated only after 24 hours */
  if (buffer_full_day == false){
    avg_24_hr_humidity = avg_reading; /* return 0 */
  }
  else{
    for (uint16_t data_index = 0u; data_index < DATA_24HR; data_index++){
      total_Humidity_24hrs += HUM_Avg_24_hrs[data_index];
    }
    avg_24_hr_humidity = (total_Humidity_24hrs / DATA_24HR);

  }
#endif


  return avg_24_hr_humidity;
}
/**
 *
 * @brief This function will fill Humidity Average of 24 hours
 * @return Null
 */
void calculateHumidityAvg(void)
{
  HUM_Avg_24_hrs[fill_data_index] = SHT41_get_humidity();
  fill_data_index++;
    if(fill_data_index>=DATA_24HR)
      {
        fill_data_index=0u;
        buffer_full_day=true;
      }

  }


/**
 *
 * @brief This function will return Temperature value
 * @param N/A
 * @return Temperature value
 */
int32_t get_TempVal()
{
    int32_t retVal = 0;

    if(ftm_sub_command == 0U)
    {
         retVal = temp_val;
    }
    else if(ftm_sub_command == 1U)
    {
          retVal = 0xFF;
    }
    else
    {
           /* LDRA */
    }
  return retVal;
}

/**
 *
 * @brief This function will return Humid Value value
 * @param N/A
 * @return Humid Value
 */
uint32_t get_HumidityVal()
{
    uint32_t retVal = 0U;

    if(ftm_sub_command == 0U)
    {
          retVal = humid_val;
    }
    else if(ftm_sub_command == 1U)
    {
          retVal = 0xFFU;
    }
    else if (ftm_sub_command == 2U)
    {
          retVal = 50U;
    }
    else
    {
          /* LDRA */
    }

  return retVal;
}

/**
 *
 * @brief This function set the FTM sub command
 * @param uint8_t sub_command
 * @return n/a
 */

void set_ftm_sub_command(uint8_t sub_command) {
  ftm_sub_command = sub_command;
}

/**
 *
 * @brief This function set the FTM simulated humid value
 * @param uint32_t ftm_humid_val
 * @return n/a
 */

void set_ftm_simulate_humid(uint32_t ftm_humid_val)
{
  humid_val = ftm_humid_val;
}

/**
 *
 * @brief This function set the FTM simulated temperature value
 * @param int32_t ftm_temp_val
 * @return n/a
 */

void set_ftm_simulate_temp(uint32_t ftm_temp_val)
{
  temp_val = ftm_temp_val;
}
