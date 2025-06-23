/**
 * @file    events.h
 * @brief   Definition of system events.
 * @project P0200 Techem Core Firmware
 * @date    16 March 2022
 * @author  ABR
 */

#ifndef EVENTS_H_
#define EVENTS_H_

extern OS_FLAG_GRP Event_Flags_TopGroup;
extern OS_FLAG_GRP Event_Flags_SubGroup[2];
/* ************************************************* */
/* Behavioural Events                                */
/*                                                   */
/* ************************************************* */

//ABR. To be moved to relevant modules
extern OS_FLAG_GRP switchFlags;

#define EVENT_TIMESTAMP_0   				(1u << 1)   /**<1 Not stopped during disable, started on power-up, periodic */
#define EVENT_CO_VARIENCE_0 				(1u << 2) 	/**<2 Not stopped during disable, started on power-up, periodic*/
#define EVENT_HEARTBEAT_0   				(1u << 3)	/**<3 Started when entering active state, periodic*/
#define EVENT_TEMP_HUMIDITY_MEASURMENT_0 	(1u << 4) 	/**<4 Temperature & Humidity BIST & measurement,periodic */
#define EVENT_SMOKE_MEASUREMENT_0 			(1u << 5)	/**<5 Smoke Detection available in Operation Mode, periodic */
#define EVENT_SMOKE_BIST_0 					(1u << 6)   /**<6 Smoke Chamber BIST available in Operation Mode, periodic */
#define EVENT_SMOKE_INCREASED_ACQ_RATE_0 	(1u << 7) 	/**<7 if Increased smoke detected in Operation Mode, */
#define EVENT_SOILING_0 					(1u << 8) 	/**<8  Soiling Measurement & BIST is done during same time in Operation Mode, periodic */
#define EVENT_HEAT_BIST_MEASUREMENT_0 		(1u << 9) 	/**<9 Heat Measurement & BIST, periodic */
#define EVENT_CO_MEASUREMENT_0 				(1u << 10) 	/**<10 CO Measurement, periodic */
#define EVENT_CO_BIST_0  					(1u << 11) 	/**<11 CO BIST, periodic */
#define EVENT_CO_INCREASED_ACQ_RATE_0  		(1u << 12)	/**<12 Increased CO detected change rate of acquisition  , periodic */
#define EVENT_BUZZER_BIST_0 				(1u << 13)	/**<13 Buzzer BIST  , periodic */
#define EVENT_OBSTACLE_COVERAGE_BIST_MEASUREMENT_0 (1u << 14)/**<14 Obstacle & Coverage  Detection/BIST , periodic */
#define EVENT_FAULT_SILENCE_TIMEOUT_0 		(1u << 15)	/**<4 Started when button pressed to silence battery fault chirp, one-shot*/
#define EVENT_AIRING_CONFIGURATION_TIMEOUT_0 (1u << 16) /**<16 Airing Configuration, Non-periodic */
#define EVENT_AMBIENT_MEASUREMENT_0 		(1u << 17)  /**<17 AmbientLight Power, periodic */
#define EVENT_DEMOUNT_TOO_LONG_0 				(1u << 18)  /**<18 Demount too long one-shot */
#define EVENT_SMOKE_HIGH_SUPER_0  			(1u << 19) 	/**<19 Smoke High Super detected, Non-periodic */
#define EVENT_SMOKE_NONE_0 					(1u << 20)	/**<20 No Smoke detected, Non-periodic */
#define EVENT_HEAT_HIGH_SUPER_0 			(1u << 21)	/**<21 Heat High Super detected, Non-periodic */
#define EVENT_HEAT_NONE_0 					(1u << 22)	/**<22 No Heat , Non-periodic */
#define EVENT_COHB_HIGH_COHB_SUPER_0  		(1u << 23)	/**<23 CO Hig /Super detected, Non-periodic */
#define EVENT_COHB_NONE_0 					(1u << 24) 	/**<24 No CO Configuration, Non-periodic */
#define EVENT_REMOTE_ALARM_0 				(1u << 25) 	/**<25 Remote Alarm , Non-periodic */
#define EVENT_MODE_CHANGE_0 				(1u << 26)  /**<26 Mode change event, Non-periodic */
#define EVENT_STATE_TIMEOUT_0 				(1u << 27)  /**<27 System State change event, Non-periodic */
#define EVENT_BUTTON_PRESS_0 				(1u << 28) 	/**<28 Button Press By Switch Module, Non-periodic */
#define EVNET_ADS_ENABLE_0 					(1u << 29)	/**<29 Event group 1 1 ADS Enabled, Non-periodic */
#define EVNET_ADS_DISABLE_0 				(1u << 30)	/**<30 ADS Disabled, Non-periodic */
#define EVENT_SHUTDOWN_0 					(1u << 31)	/**<31 device shutdown low Voltage below 2.7/2.4 v, Non-periodic */
/*******END of 32 Event of Flag 0* ****************************************************************/

#define EVENT_SPI_COMS_1 (1u ) /**<1 SPI Comm Task , periodic Event 1 */
#define EVENT_WATCHDOG_1 (1u << 1)  /**<2 Watchdog Task , periodic */
#define EVENT_USER_EXTND_LASER_BIST_1 (1u << 17)  /**<17 extended user bist laser test wait time */
/*********************************** END of Flag 1*************************************************/

#endif /* APP_INC_EVENTS_H_ */
