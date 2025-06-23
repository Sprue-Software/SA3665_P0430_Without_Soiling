/*
 *
 *  @file acquisition_Heat.h
 *  @brief: This File describe variable required for Heat Sensing & BIST functionalities
 *  Created on: 4 Apr 2022
 *  Author:
 */

#ifndef APP_INC_ACQUISITION_HEAT_H_
#define APP_INC_ACQUISITION_HEAT_H_

#include "debug.h"
#include "system_events.h"

#define THERMISTOR_R0_FACTOR        (10000.0)
#define THERMISTOR_R1_FACTOR        (16000.0)
#define THERMISTOR_T0_FACTOR        (298.15)
#define THERMISTOR_BETA_FACTOR      (3434.0)
#define THERMISTOR_ZERO_CONSTANT    (273.15)
#define THERMISTOR_REF_VOLTAGE      (1210u)
#define THERMISTOR_GAIN_FACTOR      (500)
#define THERMISTOR_REF_ADC          (0xFFFFu) //16bit full scale
#define THERMISTOR_ADC_HIGH_LIMIT   (61604u)  //open circuit threshold
#define THERMISTOR_ADC_LOW_LIMIT    (3932u)   //Short circuit threshold
#define THERMISTOR_MAX_STRIKE_COUNT (3u)      // maximum strike count limit
#define SUPER_HEAT                  (540)     //ABR Heat 54C (PTR-1227)

#define HEAT_THRESHOLD_32           (320)
#define HEAT_THRESHOLD_33           (330)
#define HEAT_THRESHOLD_36           (360)
#define HEAT_THRESHOLD_42           (420)
#define HEAT_THRESHOLD_43           (430)
#define HEAT_THRESHOLD_44           (440)
#define HEAT_THRESHOLD_45           (450)
#define HEAT_THRESHOLD_46           (460)
#define HEAT_THRESHOLD_47           (475)
#define HEAT_THRESHOLD_48           (485)
#define HEAT_THRESHOLD_52           (520)


#define DEFAULT_TEMPERATURE         (250)
#define ONE_DEGREE_RISE_THIRTY_SEC  (5)      //for 6 tap filter, i.e. 30 seconds between IIR filter input samples
#define ONE_DEGREE_RISE_SIXTY_SEC   (10)
#define ONE_DEGREE_RISE_NINETY_SEC  (15)

#define THIRTY_DEG_RISE_IN_THIRTY_SEC   (45)      //for 6 tap filter, i.e. 30 seconds between IIR filter input samples
#define TWENTY_DEG_RISE_IN_THIRTY_SEC   (40)
#define TEN_DEG_RISE_IN_SIXTY_SEC       (48)
#define FIVE_DEG_RISE_IN_SIXTY_SEC      (32)
#define THREE_DEG_RISE_IN_NINETY_SEC    (25)
#define ONE_DEG_RISE_IN_NINETY_SEC      (8)


#define ROOM_TEMP                   (250)
#define TEMP_GAIN                   (99)
#define TEMP_OFFSET                 (-10)
#define TEMP_FACTOR                 (100)

int32_t getHeatAfterCompensation(void);
bool calculateTemperature(uint16_t Heat_adc_raw, int32_t *ptr_temperature, bool *bistResult);
void detectHeat(int32_t temperature);
heat_state_enum getHeatState(void);


/**
 * @brief  This Function initializes the heat module and loads any relevant calibration data from EEPROM
 * @param  temperature
 * @return None
 */
void initHeat(void);

void SetSimulatedHeatMode(bool simulated);
void InjectCurrentHeatValue(uint32_t heatValue);
void setHeatState(uint8_t status);

#endif /* APP_INC_ACQUISITION_HEAT_H_ */
