/*
 *
 *  @file acquisition_CO.h
 *  @brief: This File describe variable required for CO Sensing & BIST functionalities
 *  Created on: 4 Apr 2022
 *  Author: NDI
 */

#ifndef APP_INC_ACQUISITION_CO_H_
#define APP_INC_ACQUISITION_CO_H_

#include "debug.h"

#define INT_SAMPLE_RATE  /* For Intelligent sample rate */
#define STRIKE_COUNT_FAULTS 15u         /**<   fault Strike Count change from 20 to 15 as  Diagnostic period change to 3 min*/
#define NEW_SC_STRIKE_COUNT_FAULTS 5u  /**<   new short circuit strike count as per DCR0139 >**/
#define CO_OUT_STRIKE_COUNT_FAULTS 3u  /**<    >**/

/* COHB Threshold values for Alarm*/
#define COHB_THRESHOLD 1320U
/* PPM Values */
#define CO_10PPM       10U
#define CO_20PPM       20U
#define CO_30PPM       30U
#define CO_37PPM  37U
#define CO_40PPM       40U
#define CO_73PPM  73U
#define CO_79PPM       79U
#define CO_80PPM       80U
#define CO_150PPM      150U   /**< SuperCO */
#define CO_180PPM      180U   /**< CO_intelligentSampleData.Co_intelligent_sample */
#define CO_179PPM      179U
#define CO_350PPM      350U
#define CO_449PPM      449U
#define CO_380PPM 380u
#define CO_600PPM 600U
#define CO_2000PPM     2000U

/**< Diagnostic shall be stop if CO above > Threshold */
#define CO_Diagnostic_40PPM_THRESHOLD       40U
#define CO_Diagnostic_180PPM_THRESHOLD 180U

/* Default Timing for the CO reading mode */
#define DEFAULT_SAMPLE      50U   /**< 50 seconds */
#define INTELLEGENT_SAMPLE  10U   /**< 10 Seconds */
#define SNIFF_MODE_SAMPLE   10U   /**< 10 Seconds */

#define RelativeTemperatureHi 60 /**< Upper index limit for relative temperature */
#define RelativeTemperatureLo 0U /**< Lower index limit for relative temperature */


#define MAX_TMPR_VALUE 5000U /**< Maximum temperature for scale factor calc. is 100 times actual value */
#define MIN_TMPR_VALUE -1000 /**< Maximum temperature for scale factor calc. is 100 times actual value */

/**
 Low Level CO warning timings
 */
typedef enum {
  NO_LOW_LEVEL_CO = 0, /**< NO_LOW_LEVEL_CO */
  LOW_LEVEL_CO_WARNING_4HRS, /**< LOW_LEVEL_CO_WARNING_4HRS */
  LOW_LEVEL_CO_WARNING_35MIN,/**< LOW_LEVEL_CO_WARNING_35MIN */
  LOW_LEVEL_CO_WARNING_20MIN /**< LOW_LEVEL_CO_WARNING_20MIN */
} LOW_LEVEL_CO_WARNING;

/**
 * CO Alarm Level
 */
typedef enum
{
  NO_ALARM = 88,        /**< NO_ALARM *///12.5
  CO_ALARM_MIN_74 = 15, /**< CO_ALARM_MIN_74 */
  CO_ALARM_MIN_25 = 45, /**< CO_ALARM_MIN_25*/
 
  CO_ALARM_SEC_100 = 660, /**<CO_ALARM_SEC_100*/

  CO_ALARM_MIN_0 = 1320 /**< CO_ALARM_MIN_0*/
} CO_ALARM_TIME;

/**
 *CO  Values level
 */
typedef enum {
  NO_CO = 0, /**< NO_CO */
  HIGH_CO_LESS_THAN_150PPM = 2,/**< HIGH_CO_LESS_THAN_150PPM */
  SUPERCO = 150, /**< SUPERCO if CO>150  */
} CO_LEVEL;

/**
 * CO  Mode
 */
typedef enum {
  NORMAL_MODE = 0,/**< In NORMAL_MODE  */
  REMOTE_ALARM, /**<  In REMOTE_ALARM */
  LOCAL_ALARM, /**<  In LOCAL ALARM */
  SNIFFMODE, /**<  SNIFFMODE */
  CO_TEST, /**<  TEST Mode */
  CO_FALSE_ALARM_MONITOR /**<  Monitor false alarm */
} OPERATING_MODE;

/**
 *  CO Circuit Fault Values
 */
typedef enum {
  NO_CO_FAULT = 0, /**<No faults*/
  OC_CO_FAULT, /**< Open Circuit fault  */
  SC_CO_FAULT, /**< Short Circuit fault */
  FLASH_CRC_FAULT, /**< Calibration Fault*/
  CO_HW_FAULT

} FAULT_CIRCUIT;

typedef enum
{
  ACQCO_SIM_MODE_NONE,
  ACQCO_SIM_MODE_RAW,
  ACQCO_SIM_MODE_PPM
}
ACQCO_SIM_MODE;

#define HOURS_TEN_YEAR          87600U                    /**< 10 years (in hours) */
#define HOURS_12_YEARS         105120U                    /**< total number of hours for 12 years */
#define HOURS_IN_QUARTERLY       2190U                    /**< 0.25 year (in hours) */

#define HOURS_IN_YEAR            8760U
#define HOURS_IN_MONTH            730U

#define MIN_QUARTERLY           0U      /**< The minimum value for quarterly  */
#define MAX_QUARTERLY           59    //46U  /**< The maximum value for quarterly  thirteen years - remembering index starts at 12  */

/* reference Voltage Given By Hardware team */

/* Timer PWM Channel Number*/
#define TEMP_VAL_CAL 1000

#define TEMP_VAL_CAL_DEV 100u
#define Hours_4          14400U /*(60*60*4) */
#define MIN_35           2100U  /*(60*60*4) */
#define MIN_20           1200U  /* (*4) */
/* delay Used In the code */
#define MSC_5  5U
#define MSC_2  2U
#define MSC_1  1U
/*Humidity Logical values based on Stuart Compensation formula  */
#define VAL_25                      25U
#define VAL_23                      23U
#define VAL_11                      11U
#define VAL_5                       5U
#define VAl_2                       2U
#define VAl_1                       1U
#define VAL_1K                      1000U
#define VAL_9                       9U
#define MAX_COUNT                   3000
#define HIGH_GAIN                   1U
#define LOW_GAIN                    2U
#define FLASH_READ_SIZE_PTR         4U
#define CO_CAL_DEFAULT              20U
#define Intelligent_SAMPLE_RATE_CTR 5U
#define SAMPLE_CT                   2
#define SAMPLE_BFR_CTR              7
#define COHB_DIV                    5U
#define HIGH_GAIN_FACTOR            1U
#define LOW_GAIN_FACTOR             3U
#define BUFFER_INDEX_VAL            10U
#define DEFAULT_RH                  40u
#define HUMIDITY_STEP               5u

#define NA_PER_PPM_DEFAULT (8u)

uint8_t get_Co_values(uint16_t *result);
CO_LEVEL get_CO_Level(void);
void Set_Low_Co_Warning(const LOW_LEVEL_CO_WARNING state_warining, bool Low_Level_timer_start);
LOW_LEVEL_CO_WARNING Get_Low_CO_Warning(void);
uint32_t getCoAfterCompensation(void);
uint16_t getRawCo(void);
void SetMode(const OPERATING_MODE mode_Co);
void SetIntelligentSampleRate(bool state);

void acquisitionCO(bool doCOdiag);
void defaultVariableCO(void);

/* Flash Location*/
uint16_t GetLowLevel_CO_PeakValue(void);

uint16_t GetMaxAlarmCO(void);
void resetMaxAlarmCO(void);
void SetOverLoadFlag(void);
void setoffbaseCO_variance(bool flag);
void disable_variance(bool status_flag);
bool getvariance_status(void);
void Ack_CO_Overload_events(void);
bool get_CO_Overload_Status(void);
void co_demount_init(void);
/* Flash Locations*/
#define OC_TH_LOCATION_LSB   234U
#define OC_TH_LOCATION_MSB   235U

#define SC_TH_LOCATION_LSB   236U
#define SC_TH_LOCATION_MSB   237U

#define CRC_LOCATION_LSB 223U
#define CRC_LOCATION_MSB 222U
#define VARIANCE_READ    234u
#define HUMIDITY_READ    233u
#define TEMP_READ    232u
#define COCF_READ   230u
#define CO_CAL_READ  231u


#define CRC_CAL_SIZE   222U
#define CRC_FLASH_SIZE_   223U
#define FLASH_READ_SIZE   250U
#define AGING_START_LOCATION 141U
#define AGING_START_LOCATION_AFTER_YEAR 153U
#define CO_ADDTIONAL_COMPENSATION_DAYS_REQUIRED 6u
#define CO_OVERLOAD_EVENTS_COUNT  2u
#define SUPER_CO 150 /*150 PPM*/



uint16_t calc_crc16(uint16_t crc,uint8_t data);
uint16_t get_crc_flash(uint8_t data2[]);
FAULT_CIRCUIT getFTM_CO_HW_Fault(void);
uint16_t getFTM_CO_RawData(void);
uint16_t getFTM_CO_AfterCompData(void);

void disable_variance(bool status_flag);
bool getvariance_status(void);
FAULT_CIRCUIT SensorTestCarbonMonoxide(void);
void setgain(const uint8_t gain);
uint8_t getgain(void);
uint8_t get_Co_values(uint16_t *result);
void SetIntelligentSampleRate(bool state);
bool getIntelligentSampleRate(void);
void SetMode(const OPERATING_MODE mode_Co);
OPERATING_MODE getmode(void);
uint16_t getRawCo(void);
void setCOValue(const uint32_t CO_Value);
uint32_t GetPWM_Cycle(void);
LOW_LEVEL_CO_WARNING Get_Low_CO_Warning(void);
void Set_Low_Co_Warning(const LOW_LEVEL_CO_WARNING state_warining, bool Low_Level_timer_start);

uint32_t get_manufactureDate(void);
void setSCCounter(void);

/*******************************************************************************
 * @brief set_ftm_strike_count
 *
 * This function sets SC and OC strike count to zero to enable FTM CO BIST simulation
 *
 * @param n/a
 */
void set_ftm_strike_count(void);

/*******************************************************************************
 * @brief get_ftm_co_bist
 *
 * This function restores SC and OC strike count value to its default;
 *
 * @return[out]bool   true/false = enable/disable
 */
void restore_co_bist_strike_count(void);

/*******************************************************************************
 * @brief Set raw co reading simulation mode
 * 
 * This function enables/disables the raw/ppm co simulation mode
 * 
 * @param[in] sim_mode   Simulation mode
 */
void acqco_simulate_raw_co_raw_reading( const bool sim_mode );

/*******************************************************************************
 * @brief Set simulated ppm co reading
 * 
 * This function sets the simulated ppm co value to be use when the
 * raw co/ppm simulation mode has been enabled
 * 
 * @param[in] co_ppm   Simulated value
 */
void acqco_simulated_ppm_co_reading( const uint16_t co_ppm );

/*******************************************************************************
 * @brief Set simulated raw co reading
 * 
 * This function sets the simulated raw co value to be use when the
 * raw co/ppm simulation mode has been enabled
 * 
 * @param[in] co_raw   Simulated value
 */
void acqco_simulated_raw_co_reading( const uint16_t co_raw );

#endif /* APP_INC_ACQUISITION_CO_H_ */
