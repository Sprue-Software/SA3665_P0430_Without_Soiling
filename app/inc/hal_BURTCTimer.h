/**
 * @file  hal_BURTCTimer.h
 * @brief
 * @project P0200 Techem Core Firmware
 * @date    16 March 2022
 * @author  abenrashed
 */

#ifndef HAL_BURTCTIMER_H_
#define HAL_BURTCTIMER_H_

#include "em_burtc.h"

#define burtcClk                  32768u/burtcClkDiv_128 /**< Divide clock by 128. */
// BURTC interrupts period
#define BURTC_COMPARE_FOR_IRQ_PERIOD_10SEC    (burtcClk * 10)       //ABR Techem.
#define BURTC_COMPARE_FOR_IRQ_PERIOD_8SEC     (burtcClk * 8)        //ABR Techem.
#define BURTC_COMPARE_FOR_IRQ_PERIOD_5SEC     (burtcClk * 5)        //ABR Techem.
#define BURTC_COMPARE_FOR_IRQ_PERIOD_4SEC     (burtcClk * 4)        //ABR Techem.
#define BURTC_COMPARE_FOR_IRQ_PERIOD_2SEC     (burtcClk * 2)        //ABR Techem.
#define BURTC_COMPARE_FOR_IRQ_PERIOD_1SEC     (burtcClk * 1)        //ABR Techem.

#define One_Second_Period   1u
#define Two_Second_Period   2u
#define Ten_Second_Period   10u


typedef struct {
	bool enabled;
	bool periodic;
	uint32_t period;
	uint32_t value;
} BURTCTimer_TypeDef;

/* ************************************************* */
/* Behavioural Events                                */
/*                                                   */
/* ************************************************* */
typedef enum {
	TMR_Battery_Measurement_BIST_event_0, /**<0 Not stopped during disable, started on power-up, periodic */
	TMR_timestamp_event_0, /**<1 Not stopped during disable, started on power-up, periodic */
	TMR_CO_Variance_Acquisition_event_0, /**<2 Not stopped during disable, started on power-up, periodic*/
	TMR_AmbientLight_measure_event_0,/**<3 AmbientLight Power, periodic*/
	TMR_TempHum_measure_BIST_event_0,/**<4 Temperature & Humidity BIST & measurement,periodic */
	TMR_Smoke_measure_event_0,/**<5 Smoke Detection available in Operation Mode, periodic */
	TMR_Smoke_BIST_event_0, /**<6 Smoke Chamber BIST available in Operation Mode, periodic */
	TMR_Smoke_IncreasedSampleRate_event_0, /**<7 if Increased smoke detected in Operation Mode, */
	TMR_Soling_Measurement_BIST_event_0,/**<8  Soiling Measurement & BIST is done during same time in Operation Mode, periodic */
	TMR_Heat_measure_BIST_event_0, /**<9 Heat Measurement & BIST, periodic */
	TMR_CO_measure_event_0, /**<10 CO Measurement, periodic */
	TMR_CO_BIST_event_0, /**<11 CO BIST, periodic */
	TMR_CO_IntelligentsampleRate_event_0,/**<12 Increased CO detected change rate of acquisition  , periodic */
	TMR_BUZZER_BIST_event_0,/**<13 Buzzer BIST  , periodic */
	TMR_Obstacle_Coverage_BIST_event_0,/**<14 Obstacle & Coverage  Detection/BIST , periodic */
	TMR_faultSilence_event_0, /**<15 Started when button pressed to silence battery fault chirp, one-shot*/
	TMR_Airing_Configuration_Timeout_event_0, /**<16 Airing Configuration, Non-periodic */
	TMR_heartbeat_event_0, /**<17 Started when entering active state, periodic */
	TMR_Demount_too_long_event_0, /**<18 Demount too long event Non-periodic */
	TMR_Smoke_HIGH_SUPER_event_0,/**<19 Smoke detected, Non-periodic */
	TMR_Smoke_NONE_event_0,/**<20 No Smoke , Non-periodic */
	TMR_Heat_HIGH_SUPER_event_0,/**<21 Heat detected, Non-periodic */
	TMR_Heat_NONE_event_0, /**<22 No Heat Configuration, Non-periodic */
	TMR_COHB_HIGH_SUPER_event_0,/**<23 CO detected, Non-periodic */
	TMR_COHB_NONE_event_0, /**<24 No CO , Non-periodic */
	TMR_RADIO_Alarm_event_0,/**<25 Remote Alarm , Non-periodic */
	TMR_MODE_Change_event_0,/**<26 Mode change  , Non-periodic */
	TMR_State_Timeout_event_0,/**<27 State  timeout , Non-periodic */
	TMR_Button_Press_0,/**<28 State  timeout , Non-periodic */
	TMR_Device_enable_event_0,/**<29 device Enabled, Non-periodic */
	TMR_Device_disable_event_0,/**<30 device Disabled, Non-periodic */
	TMR_Device_Shutdown_event_0,/**<31 device shutdown low Voltage below 2.7/2.4 v, Non-periodic */
	SpiComms_event_1, /**<1 SPI Comm Task , periodic */
	WdogTimer_event_1, /**<2 Watchdog Task , periodic */
	LedBuzz_event_1, /**<3 LED Buzzer event, non-periodic */
	CO_OVERLOAD_event_1, /**<4  CO Overload Compensation , non-periodic */
	FTM_Test_Btn_event_1, /**<5 FTM for test button, non-periodic  */
	FTM_Timeout_event_1,  /**<6 FTM Timeout Event, non-periodic */
	Assistance_Light_event_1,  /**<7 Assistance light timeout event, non-periodic */
	TMR_EVENT_COUNTERS_SPI_SEND_1,  /**<8 Send event counters to MCU2 event, periodic */
	Start_Laser_BIST_1,  /**<9 Starts Laser BIST test, non-periodic */
	Check_BIST_Results_1, /**<10 Checks the BIST result, non-periodic */
	TMR_Time_Production_Lockout_1, /**<11 time production lockout period, non-periodic */
	TMR_AmbientLight_7days_darkness_1, /**<12 AmbientLight light 7 days in darkness, periodic */
	TMR_Demount_One_min_period_event_1, /**<13 Demount status validation of one min, Non-periodic */
	TMR_Production_complete_1,  /**<14 production complete, 120hrs Non-periodic */
	TMR_FAILED_SPI_COMMS_TIMEOUT_1, /**<15 SPI fail timer timeout, 20min Non-periodic */
	TMR_Disable_DBG_Port_1, /**<16 DBG port disable, 10s Non-periodic */
	TMR_Smoke_Fast_Flame_Disable_1, /**<17 fast flame disable 3min -Non Periodic */
	TMR_Extended_Laser_BIST_1, /**<18 extended BIST of laser check */
	NO_OF_EVENTS /** Insert any new event before this item. This item represents the enum length*/
} BURTCTimer_Events_TypeDef;

/*BURTC TICK= 10 seconds, Please change the period if BURTC_COMPARE_FOR_IRQ_PERIOD_XSEC  */
#define BURTC_PERIOD  						          10u    /**<BURTC TICK period BURTC_PERIOD = X SEC*/
#define PERIOD_SET(p)    					          (p / BURTC_PERIOD)
#define EVENT0_LIMIT 31
/*Periods are defined in multiples of 10 seconds  Please re-verify*/
#define BATTERY_MEAS_BIST_PERIOD  			    PERIOD_SET(86400u)        /* 24 Hr =86400 sec */
#define BATTERY_FAULT_BIST_PERIOD 			    PERIOD_SET(60u) 	        /* battery in fault condition periodicity 1 minute */
#define TIMESTAMP_PERIOD 					          PERIOD_SET(10u) 	        /* 10 sec*/
#define VARIANCE_PERIOD_NON_OPERATIONAL     PERIOD_SET(180u)          /* 3 min*/
#define AMBIENT_MEASUREMENT_PERIOD          PERIOD_SET(360u)          /* AmbientLight Power, periodic= every 6 min */
#define FAULT_SILENCE_TIMEOUT 				      PERIOD_SET(84600u)        /* PTR-267  23.5 Hr =84600 sec */
#define SMOKE_MEASUREMENT 					        PERIOD_SET(10u)		        /* Smoke Detection 10 sec */
#define SMOKE_BIST_MEASUREMENT 				      PERIOD_SET(3600u)         /* 6 PTR-1188  Smoke � Degraded Chamber every hour Need to change*/
#define SMOKE_INCRESED_RATE 				        PERIOD_SET(2u) 		        /* if Increased smoke detected, This need to be handled by LETIMER because of 2 s time BURTC is 10 sec*/
#define HEAT_MEASURMENT_BIST_PERIOD 		    PERIOD_SET(10u)  	        /* Heat Measurement & BIST, periodic 10 sec (PTR-1108) */
#define CO_MEASUREMENT_PERIOD 				      PERIOD_SET(50u) 	        /* CO Measurement, periodic 50 sec */
#define CO_BIST_PERIOD 						          PERIOD_SET(180u) 	        /* CO BIST, periodic= 3 min */
#define CO_INCREASED_SAMPLE_RATE 			      PERIOD_SET(10u)		        /* Increased CO detected change rate of acquisition , periodic  10sec */
#define BUZZER_BIST_PERIOD 					        PERIOD_SET(604800u)	      /* Buzzer BIST  , periodic  every 7 days*/
#define OBSTACLE_COVARAGE_PERIOD 			      PERIOD_SET(604800u)       /* Obstacle & Coverage  Detection/BIST , periodic every 7 days*/
#define TEMP_HUMIDITY_PERIOD 				        PERIOD_SET(120u) 	        /* Temperature & Humidity BIST & measurement,periodic= 2 min */
#define HEARTBEAT_PERIOD                    PERIOD_SET(60u)           /* 3 every 60 sec */
#define AMBIENT_LIGHT_7DAYS_PERIOD 			    PERIOD_SET(604800u)  	    /* AmbientLight in Darkness, periodic= every 7 days */
#define RADIOAIRING_CONFIGURATION_TIMEOUT 	PERIOD_SET(60u)		        /* Airing Configuration, Non-periodic 60 sec */
#define SPI_COMMUMICATION_TX_RX  			      PERIOD_SET(120u) 	        /* SPI Comm Task , periodic = 2min */
//#define WATCHDOG_TIMER 						          PERIOD_SET(180u)  	      /* Watchdog Task , periodic periodic = 3min, Need to change  */
#define ALARM_SILENCE_TIMEOUT 				      PERIOD_SET(870)  	        /* TimeOut  = 14.30 min PTR-1385*/
#define DEMOUNTING_LONGTERM_PERIOD 			    PERIOD_SET(1814400u)      /* demounted too long for 21 days */
#define AMBIENTLIGHT_MEASURE_EVENT_PERIOD   PERIOD_SET(360u)          /* 36 BURTC tick period i.e. 360sec*/
#define TIMESTAMP_EVENT_PERIOD              PERIOD_SET(60u)           /* 6 BURTC tick period i.e. 60sec*/
#define WDOGTIMER_EVENT_PERIOD_TWO_MIN      PERIOD_SET(120u)          /* 12 BURTC tick period i.e. 120sec*/
#define WDOGTIMER_EVENT_PERIOD_TEN_SEC      PERIOD_SET(10u)           /* 1 BURTC tick period i.e. 10sec*/
#define DEMOUNTED_SHORT_MONITOR_PERIOD		  PERIOD_SET(60u)           /* confirmation of demounted state 1 minute */
#define EVENT_COUNTERS_PERIOD               PERIOD_SET(86400u) 	      /* Daily, 24 hours */
#define EVENT_START_LASER_BIST_PERIOD       PERIOD_SET(10u)           /* 10sec Non-periodic */
#define EVENT_CHECK_BIST_RESULT_PERIOD      PERIOD_SET(30u)           /* 30sec Non-periodic */
#define TIME_PRODUCTION_LOCKOUT_PERIOD      PERIOD_SET(432000u)       /* 120 hour in seconds */
#define DEMOUNT_ONE_MIN_PERIOD              PERIOD_SET(60u)           /* 1 min */
#define COMMISSIONMODE_FAIL_SILENCE_PRERIOD PERIOD_SET(900u)          /* 15 min */
#define TIME_PRODUCTION_COMPLETE_PERIOD     PERIOD_SET(432000u)       /* 120hrs in seconds*/
#define SPI_FAILURE_TIMEOUT_PERIOD          PERIOD_SET(1200u)         /* 20min in seconds*/
#define HEARTBEAT_EVENT                     PERIOD_SET(10u)           /* 10s Green LED event*/
#define DBG_PORT_EVENT                      PERIOD_SET(10u)
#define FAST_FLAME_DISABLE_EVENT            PERIOD_SET(180u)


#define OVERLOAD_COMPENSESTION_PERIOD     PERIOD_SET(14400u)  /* 4hrs in seconds*/

//FTM changes
#define FTM_MODE_TIMEOUT                    PERIOD_SET(180u)          /* FTM mode, non-periodic = 3min */
#define ASSISTANCE_LIGHT_PERIOD             PERIOD_SET(180u)          /* Assistance Light, non-periodic = 3min */

#define periodical                          true
#define one_shot                            false

#define FLAGS_SUBGROUP_INDEX(x)             (x/32)
//#define FLAGS_BIT_INDEX(x)                  (1u << (x%32))

void GPIO_init(void);
void BURTC_init(void);
extern void BURTC_reinit(uint8_t compVal);
void BURTCTimer_Start(BURTCTimer_Events_TypeDef event, bool periodic, uint32_t period);
uint32_t BURTCTimer_Stop(BURTCTimer_Events_TypeDef event);
void BURTCTimer_StopFrom(BURTCTimer_Events_TypeDef event);
void BURTCTimerEvent0_StopFrom(BURTCTimer_Events_TypeDef event);
uint32_t FLAGS_BIT_INDEX (uint8_t data);
void set_BURTCTimer_ads_polling(uint8_t newState);
bool BURTCTimer_Get_Event_Enable(BURTCTimer_Events_TypeDef event);
#endif  /* HAL_BURTCTIMER_H_ */
