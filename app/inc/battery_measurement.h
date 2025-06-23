/*
 * battery_measurement.h
 *
 *  Created on: 7 Jun 2022
 *      Author: uhegde
 */

#ifndef APP_INC_BATTERY_MEASUREMENT_H_
#define APP_INC_BATTERY_MEASUREMENT_H_

#include "debug.h"

#include "hal_AFE.h"

#define BATT_LOW_THRESHOLD        (2700u) /* low battery threshold in millivolts */
#define BATT_DEAD_THRESHOLD       (2400u) /* dead battery threshold in millivolts */
#define BATT_HIGH_IMPEDANCE_THRESHOLD (3200u) /* milliohm is high impedance threshold limit */
#define BATT_MAX_STRIKE_COUNT     (5u) /* maximum strike count limit */
#define BATT_MAX_ADC_REF_GAIN     (4840u) /* ADC Vref * ABUF Gain */
#define BATT_ADC_RESOLUTION       (16u) /* 16-bit ADC */
#define BATT_MAX_VOLTAGE          (3200u) /* maximum possible battery voltage */

#define BATTERY_30DAYS_DATA     (30u) /* 30 battery measurements with normal periodicity */
#define BATTERY_1DAY_DATA     (8u)  /* 1 day data of battery(considering the average of 3hrs) */
#define BATTERY_3HRS_DATA     (180u) /* 3 hrs battery average data per minute */


typedef enum {
	battery_normal_st, /* normal battery condition */
	battery_inLowVoltFault_st, /* battery module is in low batt fault condition */
	battery_inHighImpdFault_st, /* battery module is in high impedance fault condition */
	battery_shutdown_st, /* battery is in shutdown condition */
} battery_states_t;

typedef struct {
	battery_states_t battery_curr_state; /* battery in normal state by default */
	uint8_t battery_low_strike_count; /* strike count for low battery condition */
	uint8_t battery_dead_strike_count; /* strike count for dead battery condition */
	uint8_t battery_high_imp_strike_count; /* Battery high impedance fault strike count */
} battery_meas_st;

void battery_init(void); /* battery status init */
void battery_measure(void); /* dual battery measurement function */
void battery_bist(void); /* dual battery BIST function */
void battery_store_data(void);

void set_ftm_Batt_A_Sim_Vol(uint32_t vol);
void set_ftm_Batt_B_Sim_Vol(uint32_t vol);
uint32_t get_ftm_Batt_A_Sim_Vol(void);
uint32_t get_ftm_Batt_B_Sim_Vol(void);
extern void battery_calculate_monthly_average(void);

uint32_t get_ADC_Battery_A();
uint32_t get_ADC_Battery_B();
void set_ADC_Battery_A(uint32_t value);
void set_ADC_Battery_B(uint32_t value);
void set_Batt_A_Impedance(uint32_t value);
void set_Batt_B_Impedance(uint32_t value);
bool get_Battery_Periodicity_Status();
uint8_t getStrikeCount(void);
uint16_t getLowBattThres(void);

bool is_battery_circuit_fault( void );
void set_battery_circuit_fault( void );

void set_low_battery_status(bool status);
bool get_low_battery_status(void);

#endif /* APP_INC_BATTERY_MEASUREMENT_H_ */
