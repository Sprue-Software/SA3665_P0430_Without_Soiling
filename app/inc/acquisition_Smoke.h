/*
 *
 *  @file acquisition_Smoke.h
 *  @brief: This File describe variable required for Smoke Sensing & BIST functionalities
 *  Created on: 4 Apr 2022
 *  Author:
 */

#ifndef APP_INC_ACQUISITION_SMOKE_H_
#define APP_INC_ACQUISITION_SMOKE_H_

#include "debug.h"
#include <stdbool.h>
#include <stdint.h>
#include "comms_handler.h"

#define SMOKE_INCREASED_SAMPLE_RATE (2000u) /* increased 2 sec sample rate */

typedef enum {
  Smoke_ftm_none, /**< Smoke_none, no Smoke detected*/
  Smoke_ftm_dectected, /**< Smoke_high, high levels of Smoke detected*/
  Smoke_ftm_super, /**< Smoke_super, very high levels of Smoke detected*/
  Smoke_ftm_alarm_faulty
} Smoke_FTM_state_enum;

extern uint16_t smoke_thermistek_meas; /* temperature ADC value read from thermistor */
extern bool smoke_fastflame_required; /* do we need to execute the fast flame mode?? */
extern uint16_t smoke_alarm_threshold; /* threshold limit for the smoke alarm threshold */
extern bool smoke_fast_flame_mode_st; /* fast flame mode running ?? */
extern bool smoke_chamber_test; /* is it time to do the smoke chamber test? */
extern bool super_smoke_st; /* super smoke status */
extern bool smoke_alarm_st; /* smoke alarm condition */

void smoke_init(void);
bool smoke_circuit_check(void);
bool smoke_measurement_and_bist(void);
uint16_t smoke_get_value(void);
uint16_t smoke_get_percentage(void);
void smoke_periodic_measurement(void);
bool smoke_measurement_and_bist(void);
bool super_smoke_detected(void);
bool smoke_detected(void);
uint16_t smoke_get_dark_measurement();
uint16_t smoke_get_bright_measurement();
void smoke_set_dark_measurement(const uint8_t dark_msb, const uint8_t dark_lsb);
void smoke_set_bright_measurement(const uint8_t bright_msb, const uint8_t bright_lsb);
Smoke_FTM_state_enum getFTMSmokeState();
void smoke_set_ftm_active(bool ftmSimulationState);
void smoke_set_ftm_chamber_test(bool enable);
bool smoke_get_ftm_chamber_status();
uint16_t get_smoke_level(void);
void set_cham_minor_fault(bool status);
bool get_cham_minor_fault(void);
void smoke_demount_init(void);

#ifdef DEBUG_BUILD
uint16_t getSmokeThreshold();
uint16_t getSuperSmokeThreshold();
void SetSimulatedSmokeMode(bool simulated);
void InjectCurrentSmokeValue(uint32_t heatValue);
#endif

#endif /* APP_INC_ACQUISITION_SMOKE_H_ */
