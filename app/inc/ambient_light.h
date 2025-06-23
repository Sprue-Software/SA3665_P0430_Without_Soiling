/*******************************************************************************
 * @file ambient_light.h
 * @brief  
 * @date 26 Oct 2022
 * @author Akif Turker
 ******************************************************************************/

#ifndef APP_INC_AMBIENT_LIGHT_H_
#define APP_INC_AMBIENT_LIGHT_H_

#include "debug.h"
#include<stdbool.h>
#include "os.h"
#include "comms_handler.h"


#define AFE_READ                            (0U)
#define ADC_REF                             (2420U)
#define ADC_12_BIT                          (4095U)
#define ADC_16_BIT                          (65535U)
#define AMBIENT_LIGHT_SETLING_TIME          (1000U) /* This time in ms for light sensor to settle down after power up */
#define AMBIENT_LIGHT_SAMPLE_COUNT          (1U)   /* This is the sample count to take light sensor average value */
#define AMBIENT_LIGHT_HYS_BUFFER_THRESHOLD  (10U) /* This is 10% Hys buffer threshold to toggle absolute brightness or darkness state */
#define AMBIENT_THRESHOLD_VAL               (460U)

/**
 * @brief ADC configuration registers
 */
struct ADC_config_ambient {
  uint8_t gain;
  uint8_t ref_voltage;
  uint8_t avg_Sample;
  uint8_t resolution;
};

typedef enum
{
  AMBIENT_LEVEL_OK =                         0,
  AMBIENT_LEVEL_BRIGHTNESS =                 1,
  AMBIENT_LEVEL_DARKNESS =                   2,
  AMBIENT_LEVEL_ERROR =                      3,
} AMBIENT_LEVEL;

typedef struct
{
  uint8_t ambient_status;
  uint32_t adc_avg_val;

}ambient_ftm_Data;

void
ambient_light_init(void);
void
ambient_light_measure(uint32_t delay, uint8_t sample_count);

uint8_t ambient_light_get_status(void);
bool  ambient_light_get_BIST( const bool perform_measurement );
void ambient_light_set_status(uint8_t status);
void ambient_light_ftm_data(ambient_ftm_Data *data);
/* This API for SPI Communication */
uint8_t ambient_light_get_level(void);
bool get_Ambient_Light_Status(void);
bool get_SevenDays_Darkness_Status(void);
void set_SevenDays_Darkness_Status(bool status);
#endif /* APP_INC_AMBIENT_LIGHT_H_ */
