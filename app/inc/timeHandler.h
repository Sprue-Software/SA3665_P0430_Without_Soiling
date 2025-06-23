/**
 * @file  timeHandler.h
 * @brief Handle the monitoring of the time since production
 * @project P0200 Techem Core Firmware
 * @date    4 April 2022
 * @author  abenrashed
 */
#ifndef TIMEHANDLER_H_
#define TIMEHANDLER_H_

#include"stdint.h"

#include "debug.h"

#define TIME_MINUTE			        (1u)            /**< 1 minute */
#define TIME_TWO_MINUTE         (2u)            /**< 2 minute */
#define TIME_HOUR	  			      (3600u)	        /**< 1 hour in seconds */
#define TIME_DAY				        (86400u)	      /**< 1 day in seconds */
#define TIME_MONTH				      (2592000u)      /**< 30 day in seconds */
#define TIME_EOL	  			      (386316000u)    /**< 12.25 year in seconds */
#define TIME_PRODUCTION_LOCKOUT (432000u)       /**< 120 hour in seconds */

void time_initTime(void);
void time_setCurrentTime(uint32_t time);
void time_updateTimestamp(void);
void time_handleTimestamp(void);
uint32_t get_currentTime(void);
void set_commissioning_time(uint32_t time);
uint32_t get_commissioning_time(void);
void set_userBistTest_time(uint32_t time);
uint32_t get_userBistTest_time(void);


#endif /* TIMEHANDLER_H_ */
