/*
 * Acquisition_heat.c
 *
 *  Created on: 1 Apr 2022
 *      Author:
 */
#include  <kernel/include/os.h>
#include "events.h"
#include "led_buzzer.h"
#include "system_events.h"
#include "acquisition_Heat.h"
#include "hal_AFE.h"
#include "hal_BURTCTimer.h"
#include "fault_handler.h"
#include "data_logging.h"
#include <math.h>

typedef double              FLOAT_64;

#define HEAT_BUFFER_LENGHT  18u    //every 10sec for 3 minutes
#define CURRENT_SAMPLE      (HEAT_BUFFER_LENGHT-1u)
#define THIRTY_SECONDS      3u     //3 samples apart at 10sec per sample
#define SIXTY_SECONDS       6u
#define NINETY_SECONDS      9u
#define FILTER_SCALING      100

#define LOWER_THRESHOLD             (350)
#define HIGHER_THRESHOLD            (600)

#define BATTERY_LVL_FACTORY_CALIB 3000U  // this value might change
                                         // check with HW team

/*Filter coefficients*/
 static int16_t coeff_thirty_deg_rise_in_thirty_sec = THIRTY_DEG_RISE_IN_THIRTY_SEC;
 static int16_t coeff_twenty_deg_rise_in_thirty_sec = TWENTY_DEG_RISE_IN_THIRTY_SEC;
 static int16_t coeff_ten_deg_rise_in_sixty_sec = TEN_DEG_RISE_IN_SIXTY_SEC;
 static int16_t coeff_five_deg_rise_in_sixty_sec = FIVE_DEG_RISE_IN_SIXTY_SEC;
 static int16_t coeff_three_deg_rise_in_ninety_sec = THREE_DEG_RISE_IN_NINETY_SEC;
 static int16_t coeff_one_deg_rise_in_ninety_sec = ONE_DEG_RISE_IN_NINETY_SEC;

/*Out of bound limits*/
 static uint16_t adc_out_of_bound_max = THERMISTOR_ADC_HIGH_LIMIT;
 static uint16_t adc_out_of_bound_min = THERMISTOR_ADC_LOW_LIMIT;

/* Fault Strike Count*/
static uint8_t fault_strike_count = THERMISTOR_MAX_STRIKE_COUNT;

/* Super Heat */
static int16_t super_heat = SUPER_HEAT;

static int32_t current_Heat_value = DEFAULT_TEMPERATURE;/*Heat 100C. means no measurements taken yet, or thermistor fault detected*/
static heat_state_enum HeatState = Heat_none;

static bool simulatedHeatValues = false;
static uint32_t currentHeatValue = 250;

/**
 * @brief  This Function will set the compensated temperature value.
 * @param  temperature
 * @return None
 */
static void setHeatAfterCompensation(int32_t temperature)
{

  current_Heat_value = temperature;
}

/**
 * @brief  This Function will return the compensated temperature value.
 * @return  Temperature value
 */
int32_t getHeatAfterCompensation(void) {

  return current_Heat_value;
}

/**
 * @brief  This Function calculates the compensated temperature in degrees.
 * @param  [in] Heat ADC count
 * @param  [out] ps32_temperature scaled temperature by 10
 * @return true , if successful, else false
 */
bool calculateTemperature(uint16_t Heat_adc_raw, int32_t *ps32_temperature, bool *bistResult) {

  static uint8_t thermistor_limits_strike_count = 0u;
  static bool thermistor_circuit_fault = false;  /*set to default*/ //thermistor hardware fault status*/
  int32_t   temperature_value = (int32_t)DEFAULT_TEMPERATURE; /*init to error reading*/
  uint16_t  uint16_value;//For LDRA
  FLOAT_64  float_value;//For LDRA
  FLOAT_64  temperature_numer;
  FLOAT_64  temperature_denom;
  FLOAT_64  thermistor_voltage;
  uint32_t  uiBatt_A_Vol = 0U;
  uint32_t  uiBatt_B_Vol = 0U;
  uint32_t  uiBatt_avg = 0U;

  uiBatt_A_Vol = get_battery_A_Voltage();
  uiBatt_B_Vol = get_battery_B_Voltage();
  uiBatt_avg = (uiBatt_A_Vol + uiBatt_B_Vol)/2;
  Heat_adc_raw = (uint16_t)(((int32_t)Heat_adc_raw * (int32_t)BATTERY_LVL_FACTORY_CALIB)/uiBatt_avg);

  /* Check ADC limits */
  if((Heat_adc_raw < adc_out_of_bound_min) || (Heat_adc_raw > adc_out_of_bound_max))
  {
      *bistResult = false;

      uint32_t Faults = FaultHandler_GetFaultFlags();

      if ((thermistor_circuit_fault == false) && ((Faults & DEF_HEAT_SENSOR_HW_FAULT) == 0u))
      {
      	  thermistor_limits_strike_count++;
	      if(thermistor_limits_strike_count >= fault_strike_count)
	      {
	        thermistor_circuit_fault = true; /* assert thermistor's circuit fault */
	        DEBUG_HEAT("\n Heat out of limit:", false, 0u);
            DataLogging_SetEventLogbookRecord( DEF_LBE_HEAT_DET_HW_ERR_START, NULL ); /* PTR-1243 */
            FaultHandler_FaultSet(HeatSensorHwFault);
	      }
      }
  }
  else
  {
      *bistResult = true;
      thermistor_limits_strike_count = 0u;
  }

  if((thermistor_circuit_fault == false) && (thermistor_limits_strike_count == 0))
    {
      uint16_value = (uint16_t)(((uint32_t)THERMISTOR_REF_VOLTAGE * Heat_adc_raw) >> 16u);
      thermistor_voltage = (FLOAT_64)(uint16_value);
      if(THERMISTOR_GAIN_FACTOR != 0u)
      {
        thermistor_voltage /= (FLOAT_64)THERMISTOR_GAIN_FACTOR;//divide by 1000 for mv and multiply by 2 for the 0.5 gain used
      }
      temperature_numer = THERMISTOR_T0_FACTOR * THERMISTOR_BETA_FACTOR;
      if(((uint16_t)THERMISTOR_R0_FACTOR != 0u))
        {
          temperature_denom = thermistor_voltage * THERMISTOR_R1_FACTOR / (3.0 - thermistor_voltage) / THERMISTOR_R0_FACTOR;
          if((uint32_t)(temperature_denom*1000.0) != 0u)
            {
              temperature_denom = (THERMISTOR_T0_FACTOR * log(temperature_denom)) + THERMISTOR_BETA_FACTOR;
              if(((uint32_t)(temperature_denom*1000.0) != 0u) && (TEMP_FACTOR != 0))
                {
                  float_value = ((FLOAT_64)10.0 * ((temperature_numer / temperature_denom) - (FLOAT_64)THERMISTOR_ZERO_CONSTANT));
                  temperature_value = (int32_t)float_value;//For LDRA
                  //temperature_value = ROOM_TEMP + ((temperature_value - ROOM_TEMP) * TEMP_GAIN / TEMP_FACTOR);
                  temperature_value = (temperature_value * TEMP_GAIN / TEMP_FACTOR) + TEMP_OFFSET;
                  DEBUG_HEAT("\n Temp: = ", true, temperature_value);
                }
              else
                {
                  //DEBUG_HEAT("\n Heat case3:", false, 0u);
                }
            }
          else
            {
              //DEBUG_HEAT("\n Heat case4:", false, 0u);
            }
        }
      else
        {
          //DEBUG_HEAT("\n Heat case5:", false, 0u);
        }
    }
  else
    {
      //DEBUG_HEAT("\n Heat case6:", false, 0u);
    }

  *ps32_temperature = temperature_value;
  setHeatAfterCompensation(temperature_value); /*Set Compensated temperature after each reading*/
  return !thermistor_circuit_fault;
}


/**
 * @brief  This Function detects and generates alarm for high heat.
 * @param  temperature
 * @return None
 */
void detectHeat(int32_t temperature)
{
  /*For LDRA compliance, declared here*/
  static int16_t heat_data[HEAT_BUFFER_LENGHT];
  static int16_t heat_change_rate_thirty_second = 0;
  static int16_t heat_change_rate_sixty_second = 0;
  static int16_t heat_change_rate_nintey_second = 0;
  static bool initial_start = true;
  static uint8_t high_temp_count = 0;

  uint8_t count;
  uint8_t n_sample, m_sample;
  int16_t heat_threshold;
  int16_t Xn_coeff = 90;
  int16_t Yn_coeff = 10;

  RTOS_ERR err;

  if(true == simulatedHeatValues)
  {
    temperature = currentHeatValue;
    setHeatAfterCompensation(temperature);
  }

  if(initial_start)
  {
      initial_start = false;
      for (count=0; count<HEAT_BUFFER_LENGHT;count++)
      {
          heat_data[count] = (int16_t)temperature;
      }
  }
  else
  {
      for (count=0; count<(HEAT_BUFFER_LENGHT-1u);count++)
      {
          heat_data[count] = heat_data[count+1u];
      }

      heat_data[HEAT_BUFFER_LENGHT-1u] = (int16_t)temperature;
  }

  n_sample = CURRENT_SAMPLE;
  m_sample = n_sample - THIRTY_SECONDS;
  heat_change_rate_thirty_second = (((heat_data[n_sample]-heat_data[m_sample])*Xn_coeff) + (heat_change_rate_thirty_second*Yn_coeff))/FILTER_SCALING;
  //DEBUG_HEAT("\n rate_30sec:", true, heat_change_rate_thirty_second);

  m_sample = n_sample - SIXTY_SECONDS;
  heat_change_rate_sixty_second = (((heat_data[n_sample]-heat_data[m_sample])*Xn_coeff) + (heat_change_rate_sixty_second*Yn_coeff))/FILTER_SCALING;
  //DEBUG_HEAT("\n rate_60sec:", true, heat_change_rate_sixty_second);

  m_sample = n_sample - NINETY_SECONDS;
  heat_change_rate_nintey_second = (((heat_data[n_sample]-heat_data[m_sample])*Xn_coeff) + (heat_change_rate_nintey_second*Yn_coeff))/FILTER_SCALING;
  //DEBUG_HEAT("\n rate_90sec:", true, heat_change_rate_nintey_second);

  if(heat_change_rate_thirty_second > coeff_thirty_deg_rise_in_thirty_sec)
  {
      heat_threshold = HEAT_THRESHOLD_32;
  }
  else if(heat_change_rate_thirty_second > coeff_twenty_deg_rise_in_thirty_sec)
  {
      heat_threshold = HEAT_THRESHOLD_33;
  }
  else if(heat_change_rate_sixty_second > coeff_ten_deg_rise_in_sixty_sec)
  {
      heat_threshold = HEAT_THRESHOLD_36;
  }
  else if(heat_change_rate_sixty_second > coeff_five_deg_rise_in_sixty_sec)
  {
        heat_threshold = HEAT_THRESHOLD_42;
  }
  else if(heat_change_rate_nintey_second > coeff_three_deg_rise_in_ninety_sec)
  {
      heat_threshold = HEAT_THRESHOLD_43;
  }
  else if(heat_change_rate_nintey_second > coeff_one_deg_rise_in_ninety_sec)
  {
        heat_threshold = HEAT_THRESHOLD_52;
  }
  else
  {
      heat_threshold = HIGHER_THRESHOLD;
  }

  if((heat_threshold == HIGHER_THRESHOLD) && (temperature > HIGHER_THRESHOLD))
  {
      if(high_temp_count == 0u)
      {
          high_temp_count++;
          return;
      }
  }
  else
  {
      high_temp_count = 0u;
  }

  DEBUG_HEAT("\n threshold:", true, heat_threshold);
  if(temperature > heat_threshold)
  {
      DEBUG_HEAT("\n Enter Alarm:", false, 0u);
      if(HeatState == Heat_none)
      {
          /* Log Heat Detect start*/
          DataLogging_SetEventLogbookRecord( DEF_LBE_HEAT_DET_START, NULL ); /* PTR-1243 */
          OSTimeDly(1, OS_OPT_TIME_DLY, &err);
          DataLogging_SetHeatEvent(EVENT_TYPE_LOCAL);
          OSTimeDly(1, OS_OPT_TIME_DLY, &err);
          DEBUG_HEAT("\nSTART HEAT ALARM!", false, 0u); /* PTR-1240 */
          OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)EVENT_HEAT_HIGH_SUPER_0,
                     OS_OPT_POST_FLAG_SET, &err);                            
          APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1); 
      }

      if(temperature>=super_heat)
      {
          if (HeatState != Heat_super)
          {
              //DEBUG_HEAT("\nLog Start Super Heat!!", false, 0u); /* PTR-1240 */
              DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_HEAT_START, NULL );
              OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)EVENT_HEAT_HIGH_SUPER_0,
                         OS_OPT_POST_FLAG_SET, &err);
              APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
              HeatState = Heat_super;
          }
      }
      else
      {
          if (HeatState == Heat_super)
          {
              //DEBUG_HEAT("\nLog End Super Heat", false, 0u); /* PTR-1240 */
              DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_HEAT_END, NULL );
          }
          HeatState = Heat_high;
      }

  }
  else if(temperature < LOWER_THRESHOLD)
  {
      if(HeatState != Heat_none)
      {
          if (HeatState == Heat_super)
          { /*Check if coming from super heat condition, Log end of super heat */
              DEBUG_HEAT("\nLog End Super Heat", false, 0u); /* PTR-1240 */
              DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_HEAT_END, NULL );
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
          }

          HeatState = Heat_none;
          /* Log Heat Detect End*/
          DataLogging_SetEventLogbookRecord( DEF_LBE_HEAT_DET_END, NULL ); /* PTR-1243 */
          OSTimeDly(1, OS_OPT_TIME_DLY, &err);
          DEBUG_HEAT("\nEnd heat alarm", false, 0u);
          /* post the Heat alarm event */
          OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)EVENT_HEAT_NONE_0,
                     OS_OPT_POST_FLAG_SET, &err);                            
          APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1); 
      }
      else
      {
          ;//LDRA compliance
      }
  }
  else
  {
      ;//LDRA compliance
  }
}

void initHeat(void)
{
#ifdef EEPROM_CALI
#if 0
  coeff_thirty_deg_rise_in_thirty_sec   = DataLogging_GetCoeffThirtyDegRiseInThirtySec( );
  coeff_twenty_deg_rise_in_thirty_sec   = DataLogging_GetCoeffTwentyDegRiseInThirtySec( );
  coeff_ten_deg_rise_in_sixty_sec       = DataLogging_GetCoeffTenDegRiseInSixtySec( );
  coeff_five_deg_rise_in_sixty_sec      = DataLogging_GetCoeffFiveDegRiseInSixtySec( );
  coeff_three_deg_rise_in_ninety_sec    = DataLogging_GetCoeffThreeDegRiseInNinetySec( );
  coeff_one_deg_rise_in_ninety_sec      = DataLogging_GetCoeffOneDegRiseInNinetySec( );
#endif

  adc_out_of_bound_max = DataLogging_GetMaxHeatOutOfBounds( );
  if(adc_out_of_bound_max  != THERMISTOR_ADC_HIGH_LIMIT)
  {
      adc_out_of_bound_max = THERMISTOR_ADC_HIGH_LIMIT;
      DataLogging_SetMaxHeatOutOfBounds(adc_out_of_bound_max);
  }

  adc_out_of_bound_min = DataLogging_GetMinHeatOutOfBounds( );
  if(adc_out_of_bound_min != THERMISTOR_ADC_LOW_LIMIT)
  {
      adc_out_of_bound_min = THERMISTOR_ADC_LOW_LIMIT;
      DataLogging_SetMinHeatOutOfBounds(adc_out_of_bound_min);
  }

  uint8_t heatAcqPeriod = DataLogging_GetHeatAcqPeriod();
  if(heatAcqPeriod != HEAT_MEASURMENT_BIST_PERIOD)
  {
      heatAcqPeriod = HEAT_MEASURMENT_BIST_PERIOD;
      DataLogging_SetHeatAcqPeriod(heatAcqPeriod);
  }

  fault_strike_count = DataLogging_GetHeatFaultStrikeCount( );
  if(fault_strike_count != THERMISTOR_MAX_STRIKE_COUNT)
  {
      fault_strike_count = THERMISTOR_MAX_STRIKE_COUNT;
      DataLogging_SetHeatFaultStrikeCount(fault_strike_count);
  }

  super_heat = DataLogging_GetSuperHeatThreshold( );
  if (super_heat != SUPER_HEAT)
  {
      super_heat = SUPER_HEAT;
      DataLogging_SetSuperHeatThreshold(super_heat);
  }

#endif
}

/**
 * @brief  This Function returns heat alarm state
 * @param  N/A
 * @return HeatState
 */
heat_state_enum getHeatState()
{
  return HeatState;
}

void setHeatState(uint8_t status)
{
   HeatState = (heat_state_enum)status;
}


void SetSimulatedHeatMode(bool simulated)
{
  simulatedHeatValues = simulated;
}

void InjectCurrentHeatValue(uint32_t heatValue)
{
  if(true == simulatedHeatValues)
  {
    currentHeatValue = heatValue;
  }
}

