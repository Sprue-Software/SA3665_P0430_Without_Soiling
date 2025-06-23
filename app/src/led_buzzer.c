/********************************************************************************************************
*
* 									LED AND BUZZER PATTERN HANDLER
*
* Filename			: led_buzzer.c
* Version			: V1.00
* Programmers(s)	: AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/
#include "os.h"
#include "app.h"
#include "comms_handler.h"
#include "hal_LETimer.h"
#include "hal_BURTCTimer.h"
#include "led_buzzer.h"
#include "hal_gpio.h"
#include "hal_AFE.h"
#include "hal_Timer0.h"
#include "spi_comms.h"
#include "ambient_light.h"
#include "system_events.h"
#include "string.h"
#include "assistance_light.h"
#include "data_logging.h"
#include "hal_switches.h"
#include "fault_handler.h"
#include "system_events.h"
/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

/* Signal Priority */
#define DEF_PRIORITY_LOCAL_SMOKE_ALARM		    (0u)     /*Highest Priority*/
#define DEF_PRIORITY_LOCAL_HEAT_ALARM		      (1u)
#define DEF_PRIORITY_REMOTE_SMOKE_ALARM 	    (2u)
#define DEF_PRIORITY_REMOTE_HEAT_ALARM		    (3u)
#define DEF_PRIORITY_LOCAL_CO_ALARM			      (4u)
#define DEF_PRIORITY_REMOTE_CO_ALARM 		      (5u)
#define DEF_PRIORITY_HEARTBEAT				   	    (6u)
#define DEF_PRIORITY_USER_TEST				   	    (7u)
#define DEF_PRIORITY_DEV_GROUP_TEST			   	  (8u)
#define DEF_PRIORITY_OPERATIONAL_MODE		   	  (9u)
#define DEF_PRIORITY_LOW_BATT                 (10u)
#define DEF_PRIORITY_PRI_RADIO_ACT            (11u)
#define DEF_PRIORITY_PRI_RADIO_INACT          (12u)
#define DEF_PRIORITY_MAJOR_FAULT			        (13u)
#define DEF_PRIORITY_MINOR_FAULT			        (14u)

#define DEF_PRIORITY_IDLE					   	        (15u)		 /*Lowest Priority*/

#define DEF_HEARTBEAT_PERIOD_LE_TIMER_ms	    (58000u) /* 58s 				  */

#define DEF_BUZZER_LOW_FREQUENCY_hz        	  (3000u)	 /* 3.0 KHz 			*/
#define DEF_BUZZER_LOW_REDUCED_FREQUENCY_hz   (2600u)	 /* 2.6 KHz 			*/
#define DEF_BUZZER_HIGH_FREQUENCY_hz       	  (2700u)	 /* 2.7 KHz 			*/


#define DEF_PATTERN_QUEUE_SIZE 				        ((uint8_t)(TotalPatterns))

/* Phases counts in each sequence*/
#define DEF_PHASES_GAP									      (1u)
#define DEF_PHASES_SMOKE_HEAT_ALARM						(4u)
#define DEF_PHASES_CO_ALARM								    (12u)
#define DEF_PHASES_MAJOR_FAULT							  (9u)
#define DEF_PHASES_MINOR_FAULT							  (10u)
#define DEF_PHASES_USER_TEST_OK							  (3u)
#define DEF_PHASES_EXTND_USR_TST_OK						(17u)
#define DEF_PHASES_EXTND_USR_TST_OK_PHASE2		(12u)
#define DEF_PHASES_FULL_SELF_TEST_RUNNING			(2u)
#define DEF_PHASES_MOUNTED_USER_TEST_OK				(5u)
#define DEF_PHASES_TRANSPORT_MODE_CHECK				(2u)
#define DEF_PHASES_STANDBY_MODE_ACTIVATE			(2u)
#define DEF_PHASES_STANDBY_MODE_CHECK					(4u)
#define DEF_PHASES_FULL_SELF_TEST_PASS				(5u)
#define DEF_PHASES_TOM_TEST_ALARM						  (2u)
#define DEF_PHASES_TRANSPORT_MODE_ACTIVE_MOUNTED		(2u)
#define DEF_PHASES_DEV_GROUP							    (3u)
#define DEF_PHASES_RADIO_PRIDATA_ACTIVE       (2u)
#define DEF_PHASES_RADIO_PRIDAT_INACTIVE      (2u)
#define DEF_PHASES_COMMISION_FAIL             (4u)
#define DEF_PHASES_EOL                        (6u)
#define DEF_PHASES_BATT_FAULT                 (4U)
#define DEF_PHASES_LOW_BATT                   (2U)
#define DEF_PHASES_MOUNTED                    (2U)


/* Repeat phases and counts */
#define DEF_REPEAT_COUNT_SMOKE_HEAT     				(0u)
#define DEF_REPEAT_PHASE_SMOKE_HEAT     				(0u)
#define DEF_REPEAT_COUNT_CO     						    (0u)
#define DEF_REPEAT_PHASE_CO     						    (0u)
#define DEF_REPEAT_COUNT_MAJOR_FAULT    				(29u)
#define DEF_REPEAT_PHASE_MAJOR_FAULT    				(7u)
#define DEF_REPEAT_COUNT_MINOR_FAULT    				(19u)
#define DEF_REPEAT_PHASE_MINOR_FAULT    				(6u)
#define DEF_REPEAT_COUNT_USER_TEST_OK 					(1u)
#define DEF_REPEAT_COUNT_EXTND_USR_TST_OK			 	(3u)
#define DEF_REPEAT_PHASE_EXTND_USR_TST_OK    		(5u)
#define DEF_REPEAT_COUNT_PHASE2_EXTND_USR_TST_OK		(6u)
#define DEF_REPEAT_PHASE_PHASE2_EXTND_USR_TST_OK		(0u)
#define DEF_REPEAT_COUNT_FULL_TEST_RUNNING 		 		(1u)
#define DEF_REPEAT_COUNT_MOUNTED_USER_TEST_OK  	  (5u)
#define DEF_REPEAT_PHASE_MOUNTED_USER_TEST_OK     (1u)
#define DEF_REPEAT_COUNT_TRANSPORT_CHECK 			 	  (1u)
#define DEF_REPEAT_COUNT_STANDBY_MODE_ACTIVATE 		(1u)
#define DEF_REPEAT_COUNT_STANDBY_MODE_CHECK 			(1u)
#define DEF_REPEAT_PHASE_FULL_TEST_PASS		    		(5u)
#define DEF_REPEAT_COUNT_FULL_TEST_PASS			 		  (4u)
#define DEF_REPEAT_COUNT_TOM_TEST_ALARM   				(60u)
#define DEF_REPEAT_PHASE_TOM_TEST_ALARM   				(0u)
#define DEF_REPEAT_COUNT_DEV_GROUP_TEST  				  (60u)
#define DEF_REPEAT_PHASE_DEV_GROUP_TEST  				  (0u)
#define DEF_REPEAT_COUNT_TRANSPORT_ACTIVE_MOUNTED (0u)
#define DEF_REPEAT_PHASE_TRANSPORT_ACTIVE_MOUNTED (0u)
#define DEF_REPEAT_PHASE_COMMISION_FAIL           (0u)
#define DEF_REPEAT_PHASE_MOUNTED                  (0u)
/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef enum {
	allEDOff,	 					                    /* all LEDs off 						*/
	smokeLEDOn, 						                /* Red LED 									*/
	smokeLEDOff, 						                /* Red LED 									*/
	coLEDOn, 							                  /* Red LED 									*/
	coLEDOff,		 					                  /* Red LED 									*/
	faultLEDOn, 						                /* Yellow LED 						  */
	faultLEDOff, 						                /* Yellow LED 						  */
	airingLEDOn, 						                /* Orange or Blue LED			  */
	airingLEDOff, 						              /* Orange or Blue LED 			*/
	assistanceLEDOn, 					              /* White LED								*/
	assistanceLEDOff, 					            /* White LED 								*/
	powerLEDOn, 						                /* Green LED 								*/
	powerLEDOff 						                /* Green LED 								*/
} LEDBuzz_LED;

typedef enum {
	buzzerOff, 							                /* Buzzer off 								*/
	buzzerOnLow, 						                /* Buzzer On with low volume 	*/
	buzzerOnHigh, 						              /* Buzzer On with high volume */
	buzzerNochange,						              /* No change of state 				*/
	buzzerOnReduced						              /* Buzzer On with reduced volume 				*/
} LEDBuzz_BuzzerState;

typedef enum {
	timer0Stop, 						                /* Timer stop 								*/
	timer0StartLowFreq, 				            /* Timer On with low frequency */
	timer0StartLowReducedFreq, 				            /* Timer On with low frequency reduced				*/
	timer0StartHighFreq, 				            /* Timer On with high frequency				*/
	timer0StopStartLowFreq, 			          /* Timer stop and start with low frequency 	*/
	timer0StopStartLowReducedFreq, 			          /* Timer stop and start with low frequency reduced 	*/
	timer0StopStartHighFreq, 			          /* Timer stop and start with high frequency	*/
	timer0NoChange						              /* No change of state						*/
} LEDBuzz_Timer0State;

typedef struct {
	uint32_t 			LETimerPeriod;          	/* Period for LE timer 						*/
	uint32_t 			BURTCPeriod; 	            /* Period to set BURTC timer			*/
	LEDBuzz_LED 		led; 			              /* LED drive state 							  */
	LEDBuzz_BuzzerState buzzerState; 	      /* Buzzer drive state 						*/
  bool                acquireMutex;
} LEDBuzz_Sequence;

typedef struct {
	uint8_t prioity; 					              /* priority of the pattern 					*/
	const LEDBuzz_Sequence *sequence; 	    /* sequence of phases in the pattern*/
	uint8_t phaseCount; 				            /* total number of phases in the sequence	*/
	uint8_t repeat; 					              /* 0 = periodic 							      */
	uint8_t repeat_phase; 				          /* phase to repeat 							    */
	bool reset_repeat; 					            /* reset repeat counter 					  */
} LEDBuzz_PatternInfo;

typedef struct {
	uint8_t id;							                /* id of the pattern 						    */
	uint8_t priority;					              /* priority of the pattern 					*/
	uint8_t phase;						              /* current phase of the pattern 	  */
	uint8_t repeatCounter;				          /* repeat counter for the pattern	  */
	uint8_t repeatPhase;				            /* repeat phase of the pattern 		  */
} LEDBuzz_PatternState;

typedef struct {
	uint8_t patternID;
	uint32_t BURTCTimer;
	uint32_t LETimer;
	bool isFree;
	bool isBuzzerSilent;
} LEDBuzz_PendingPattern;

typedef enum {
	dark,	 							                     /* Night time 								*/
	bright 								                   /* Day time 								*/
} LEDBuzz_AmbientLightStatus;

/********************************************************************************************************
*********************************************************************************************************
*                                            VARIABLES
*********************************************************************************************************
********************************************************************************************************/

static OS_TCB LEDBuzz_TaskTCB;
static CPU_STK LEDBuzz_TaskStack[LED_BUZZ_TASK_STK_SIZE];

static LEDBuzz_PatternState LEDBuzz_CurrentPattern 		= { (uint8_t) PatternIdle, DEF_PRIORITY_IDLE, 0u, 0u, 0u };
static LEDBuzz_BuzzerState LEDBuzz_BuzzerStatus 		= buzzerOff;
static LEDBuzz_BuzzerState LEDBuzz_PreviousBuzzerStatus	= buzzerOff;

static bool LEDBuzz_HighPriorityPatternPending 			= false;
static bool LEDBuzz_Silence 							= false;
static bool LEDBuzz_AssistanceLEDOn 					= false;

static LEDBuzz_PendingPattern pendingPatternpool[DEF_PATTERN_QUEUE_SIZE];
static bool LEDBuzz_GAP = true;
static AFERspMessage_t LEDBuzzTaskResponse;

static OS_MUTEX buzzer_mutex;
static bool buzzer_mutex_locked;
static bool FTMTestPulseTone = false;

static bool heartBeatOn = false;

/********************************************************************************************************
*********************************************************************************************************
*                                            CONSTANTS
*********************************************************************************************************
********************************************************************************************************/
static uint8_t MSG_IDs [DEF_PATTERN_QUEUE_SIZE] = 
{
	(uint8_t)(PatternIdle),                           //0
	(uint8_t)(PatternGap),                            //1
	(uint8_t)(PatternAlarmSmoke),                     //2
	(uint8_t)(PatternAlarmSmokeStop),                 //3
	(uint8_t)(PatternAlarmHeat),                      //4
	(uint8_t)(PatternAlarmHeatStop),                  //5
	(uint8_t)(PatternAlarmCO),                        //6
	(uint8_t)(PatternAlarmCOStop),                    //7
	(uint8_t)(PatternAlarmRemoteSmoke),               //8
	(uint8_t)(PatternAlarmRemoteSmokeStop),           //9
	(uint8_t)(PatternAlarmRemoteHeat),                //10
	(uint8_t)(PatternAlarmRemoteHeatStop),            //11
	(uint8_t)(PatternAlarmRemoteCO),                  //12
	(uint8_t)(PatternAlarmRemoteCOStop),              //13
	(uint8_t)(PatternMajorFault),                     //14
	(uint8_t)(PatternMinorFault),                     //15
	(uint8_t)(PatternStopMajorFault),                 //16
	(uint8_t)(PatternStopMinorFault),                 //17
	(uint8_t)(PatternUserTestPass),                   //18
	(uint8_t)(PatternExtendedUserTestPass),           //19
	(uint8_t)(PatternPhase2ExtendedUserTestPass),     //20
	(uint8_t)(PatternExtendedUserTestPassStop),       //21
	(uint8_t)(PatternFullSelfTestRunning),            //22
	(uint8_t)(PatternFullSelfTestRunningStop),        //23
	(uint8_t)(PatternFullSelfTestPassCommissioning),  //24
	(uint8_t)(PatternFullSelfTestPassCommissioningStop),//25
	(uint8_t)(PatternDeviceGroupTestRunning),         //26
	(uint8_t)(PatternDeviceGroupTestStop),            //27
	(uint8_t)(PatternTransportActiveMounted),         //28
	(uint8_t)(PatternTransportActiveMountedStop),     //29
	(uint8_t)(PatternTransportModeCheck),             //30
	(uint8_t)(PatternStandbyModeActivate),            //31
	(uint8_t)(PatternStandbyModeCheck),               //32
	(uint8_t)(PatternCommissioningFail),              //33
	(uint8_t)(PatternCommissioningFailStop),          //34
	(uint8_t)(PatternLowBatt),                        //35
	(uint8_t)(PatternLowBattStop),                    //36
	(uint8_t)(PatternRadioPvtDataActive),             //37
	(uint8_t)(PatternRadioPvtDataActiveStop),         //38
	(uint8_t)(PatternRadioPvtDataInactive),           //39
	(uint8_t)(PatternRadioPvtDataInactiveStop),       //40
	(uint8_t)(PatternTOMTestAlarm),                   //41
	(uint8_t)(PatternHeartbeat),                      //42
	(uint8_t)(PatternAlarmSilence),                   //43
	(uint8_t)(PatternStopAll),                        //44
	(uint8_t)(LEDBuzz_LETimerTimeout),                //45
	(uint8_t)(LEDBuzz_BURTCTimerTimeout),             //46
	(uint8_t)(PatternFaultSilenceTimeout),            //47
	(uint8_t)(PatternAlarmSmokeSilcenceTimeout),      //48
	(uint8_t)(PatternAlarmHeatSilcenceTimeout),       //49
	(uint8_t)(PatternAlarmCOSilenceTimeout),          //50
	(uint8_t)(PatternCommissionFailSilenceTimeout),   //51
	(uint8_t)(PatternMounted),                        //52
	(uint8_t)(PatternNull)                            //53
};

/* Gap sequence */
const LEDBuzz_Sequence SEQUENCE_GAP[DEF_PHASES_GAP] = {
/*phase 00*/{ 500u, 0u, smokeLEDOff, buzzerOff, false }, };

/* Smoke and Heat alarm sequence */
const LEDBuzz_Sequence SEQUENCE_SMOKE_HEAT[DEF_PHASES_SMOKE_HEAT_ALARM] = {
/*phase 00*/{  500u, 0u, smokeLEDOn,  buzzerOnHigh, true },  /* 500 ms  */
/*phase 01*/{  500u, 0u, smokeLEDOn,  buzzerOnLow,  true },  /* 1000 ms */
/*phase 02*/{  990u, 0u, smokeLEDOff, buzzerOff,    true },  /* 1990 ms */
/*phase 03*/{  10u,  0u, smokeLEDOff, buzzerOff,    false},  /* 2000 ms *//* restart from phase 0 until conditions persists */
};

/* Smoke and Heat remote alarm sequence */
const LEDBuzz_Sequence SEQUENCE_SMOKE_HEAT_REMOTE[DEF_PHASES_SMOKE_HEAT_ALARM] = {
/*phase 00*/{  500u, 0u, smokeLEDOff,  buzzerOnHigh, true },  /* 500 ms  */
/*phase 01*/{  500u, 0u, smokeLEDOff,  buzzerOnLow , true },  /* 1000 ms */
/*phase 02*/{  990u, 0u, smokeLEDOff,  buzzerOff   , true }, /* 1990 ms */
/*phase 03*/{  10u,  0u, smokeLEDOff,  buzzerOff   , false }, /* 2000 ms *//* restart from phase 0 until conditions persists */
};

/* CO alarm sequence */
const LEDBuzz_Sequence SEQUENCE_CO[DEF_PHASES_CO_ALARM] = {
/*phase 00*/{  250u, 0u, coLEDOn,  buzzerOnHigh, true }, /* 250 ms  */
/*phase 01*/{  250u, 0u, coLEDOn,  buzzerOnLow , true }, /* 500 ms  */
/*phase 02*/{  100u, 0u, coLEDOff, buzzerOff   , true }, /* 600 ms  */
/*phase 03*/{  250u, 0u, coLEDOn,  buzzerOnHigh, true }, /* 850 ms  */
/*phase 04*/{  250u, 0u, coLEDOn,  buzzerOnLow , true }, /* 1100 ms */
/*phase 05*/{  100u, 0u, coLEDOff, buzzerOff   , true }, /* 1200 ms */
/*phase 06*/{  250u, 0u, coLEDOn,  buzzerOnHigh, true }, /* 1450 ms */
/*phase 07*/{  250u, 0u, coLEDOn,  buzzerOnLow , true }, /* 1700 ms */
/*phase 08*/{  100u, 0u, coLEDOff, buzzerOff   , true }, /* 1800 ms */
/*phase 09*/{  250u, 0u, coLEDOn,  buzzerOnHigh, true }, /* 2050 ms */
/*phase 10*/{  250u, 0u, coLEDOn,  buzzerOnLow , true }, /* 2300 ms */
/*phase 11*/{ 4100u, 0u, coLEDOff, buzzerOff   , false }, /* 6400 ms *//* restart from phase 0 until conditions persists */
};

/* CO remote alarm sequence */
const LEDBuzz_Sequence SEQUENCE_CO_REMOTE[DEF_PHASES_CO_ALARM] = {
/*phase 00*/{  250u, 0u, coLEDOff,  buzzerOnHigh, true },  /* 250 ms  */
/*phase 01*/{  250u, 0u, coLEDOff,  buzzerOnLow , true },  /* 500 ms  */
/*phase 02*/{  100u, 0u, coLEDOff,  buzzerOff   , true },  /* 600 ms  */
/*phase 03*/{  250u, 0u, coLEDOff,  buzzerOnHigh, true },  /* 850 ms  */
/*phase 04*/{  250u, 0u, coLEDOff,  buzzerOnLow , true },  /* 1100 ms */
/*phase 05*/{  100u, 0u, coLEDOff,  buzzerOff   , true },  /* 1200 ms */
/*phase 06*/{  250u, 0u, coLEDOff,  buzzerOnHigh, true },  /* 1450 ms */
/*phase 07*/{  250u, 0u, coLEDOff,  buzzerOnLow , true },  /* 1700 ms */
/*phase 08*/{  100u, 0u, coLEDOff,  buzzerOff   , true },  /* 1800 ms */
/*phase 09*/{  250u, 0u, coLEDOff,  buzzerOnHigh, true },  /* 2050 ms */
/*phase 10*/{  250u, 0u, coLEDOff,  buzzerOnLow , true },  /* 2300 ms */
/*phase 11*/{ 4100u, 0u, coLEDOff,  buzzerOff   , false }, /* 6400 ms *//* restart from phase 0 until conditions persists */
};

/* Major Faults sequence */
const LEDBuzz_Sequence SEQUENCE_MAJOR_FAULT[DEF_PHASES_MAJOR_FAULT] = {
/*phase 00*/{   10u, 0u, faultLEDOn,  buzzerOnReduced , true }, /* 10 ms   */
/*phase 01*/{   20u, 0u, faultLEDOff, buzzerOnReduced , true }, /* 30 ms   */
/*phase 02*/{  170u, 0u, faultLEDOff, buzzerOff   , true }, /* 200 ms  */
/*phase 03*/{   30u, 0u, faultLEDOff, buzzerOnReduced , true }, /* 230 ms  */
/*phase 04*/{  170u, 0u, faultLEDOff, buzzerOff   , true }, /* 400 ms  */
/*phase 05*/{   30u, 0u, faultLEDOff, buzzerOnReduced , true }, /* 430 ms  */
/*phase 06*/{ 0u, 1u, faultLEDOff, buzzerOff   , true }, /* 10000 ms */

/*phase 07*/{ 10u, 0u, faultLEDOn, buzzerOff    , true}, /* 10 ms   */
/*phase 08*/{ 0u, 1u, faultLEDOff, buzzerOff , false}, /* 10000 ms *//* repeat phase 7 and 8, 60 times every 5ms until 5min then restart from phase 0 */
};

/* Minor Faults sequence */
const LEDBuzz_Sequence SEQUENCE_MINOR_FAULT[DEF_PHASES_MINOR_FAULT] = {
/*phase 00*/{   10u, 0u, faultLEDOn,  buzzerOnReduced , true}, /* 10 ms    */
/*phase 01*/{   20u, 0u, faultLEDOff, buzzerOnReduced , true}, /* 30 ms    */
/*phase 02*/{  170u, 0u, faultLEDOff, buzzerOff   , true}, /* 170 ms   */
/*phase 03*/{   10u, 0u, faultLEDOn,  buzzerOnReduced , true}, /* 10 ms   */
/*phase 04*/{   20u, 0u, faultLEDOff, buzzerOnReduced , true}, /* 20 ms   */
/*phase 05*/{    0u,  3u, faultLEDOff, buzzerOff   , false}, /* 30000 ms */

/*phase 06*/{   10u, 0u, faultLEDOn,  buzzerOff   , false}, /* 10 ms    */
/*phase 07*/{  190u, 0u, faultLEDOff, buzzerOff   , false}, /* 190 ms   */
/*phase 08*/{   10u, 0u, faultLEDOn,  buzzerOff   , false}, /* 10 ms   */
/*phase 09*/{ 0u, 3u, faultLEDOff, buzzerOff   , false}, /* 30000 ms  *//* repeat phase 6 to 9, 20 times every 30s until 10min then restart from phase 0 */
};

/* User test ok sequence */
const LEDBuzz_Sequence SEQUENCE_USER_TEST_OK[DEF_PHASES_USER_TEST_OK] = {
/*phase 00*/{  500u, 0u, powerLEDOn,  buzzerOff    , false  }, /* 500 ms  */
/*phase 01*/{  50u, 0u, powerLEDOff,  buzzerOnReduced , true  }, /* 550 ms */
/*phase 02*/{  10u, 0u, powerLEDOff,  buzzerOff    , false  }, /* 560 ms *//* runs only once and does not repeat */
};

/* Extended user test ok sequence */
const LEDBuzz_Sequence SEQUENCE_EXTND_USR_TST_OK[DEF_PHASES_EXTND_USR_TST_OK] = {
/*phase 00*/{  50u,  0u, powerLEDOn,      buzzerOnReduced , true}, /* 50 ms   */
/*phase 01*/{  50u,  0u, powerLEDOn,      buzzerOff    , true}, /* 100 ms  */
/*phase 02*/{  50u,  0u, powerLEDOn,      buzzerOff    , true}, /* 150 ms  */
/*phase 03*/{  350u, 0u, powerLEDOn,      buzzerOff    , true}, /* 500 ms  */
/*phase 04*/{ 2500u, 0u, powerLEDOff,     buzzerOff    , true}, /* 3000 ms */

/*phase 05*/{  50u, 0u, smokeLEDOn,       buzzerOnLow  , true}, /* 50 ms   */
/*phase 06*/{ 100u, 0u, smokeLEDOff,      buzzerOnLow  , true}, /* 150 ms  */
/*phase 07*/{  50u, 0u, coLEDOn,          buzzerOnLow  , true}, /* 200 ms  */
/*phase 08*/{ 100u, 0u, coLEDOff,         buzzerOff    , true}, /* 300 ms  */
/*phase 09*/{  50u, 0u, assistanceLEDOn,  buzzerOff    , true}, /* 350 ms  */
/*phase 10*/{ 100u, 0u, assistanceLEDOff, buzzerOff    , true}, /* 450 ms  */
/*phase 11*/{  50u, 0u, faultLEDOn,       buzzerOff    , true}, /* 500 ms  */
/*phase 12*/{ 100u, 0u, faultLEDOff,      buzzerOff    , true}, /* 600 ms  */
/*phase 13*/{  50u, 0u, airingLEDOn,      buzzerOff    , true}, /* 650 ms  */
/*phase 14*/{ 100u, 0u, airingLEDOff,     buzzerOff    , true}, /* 750 ms  */
/*phase 15*/{  50u, 0u, powerLEDOn,       buzzerOff    , true}, /* 800 ms  */
/*phase 16*/{ 200u, 0u, powerLEDOff,      buzzerOff    , false}, /* 1000 ms *//* repeat phase 5 to 16, 3 times every 1s until 3s then jump to the SEQUENCE_EXTND_USR_TST_OK_PHASE2 */
};

/* Extended user test ok sequence */
const LEDBuzz_Sequence SEQUENCE_EXTND_USR_TST_OK_PHASE2[DEF_PHASES_EXTND_USR_TST_OK_PHASE2] = {
/*phase 00*/{  50u, 0u, smokeLEDOn,       buzzerOnHigh , true}, /* 50 ms   */
/*phase 01*/{ 100u, 0u, smokeLEDOff,      buzzerOnHigh , true}, /* 150 ms  */
/*phase 02*/{  50u, 0u, coLEDOn,          buzzerOnHigh , true}, /* 200 ms  */
/*phase 03*/{ 100u, 0u, coLEDOff,         buzzerOff    , true}, /* 300 ms  */
/*phase 04*/{  50u, 0u, assistanceLEDOn,  buzzerOff    , true}, /* 350 ms  */
/*phase 05*/{ 100u, 0u, assistanceLEDOff, buzzerOff    , true}, /* 450 ms  */
/*phase 06*/{  50u, 0u, faultLEDOn,       buzzerOff    , true}, /* 500 ms  */
/*phase 07*/{ 100u, 0u, faultLEDOff,      buzzerOff    , true}, /* 600 ms  */
/*phase 08*/{  50u, 0u, airingLEDOn,      buzzerOff    , true}, /* 650 ms  */
/*phase 09*/{ 100u, 0u, airingLEDOff,     buzzerOff    , true}, /* 750 ms  */
/*phase 10*/{  50u, 0u, powerLEDOn,       buzzerOff    , true}, /* 800 ms  */
/*phase 11*/{ 200u, 0u, powerLEDOff,      buzzerOff    , false}, /* 1000 ms *//* repeat phase 0 to 11, 6 times every 1s and then stop */
};

/* Full Self test running sequence */
const LEDBuzz_Sequence SEQUENCE_FULL_SELF_TEST_RUNNING[DEF_PHASES_FULL_SELF_TEST_RUNNING] = {
/*phase 00*/{  50u, 0u, faultLEDOn,  buzzerOff , false}, /* 50 ms   */
/*phase 01*/{ 950u, 0u, faultLEDOff, buzzerOff , false}, /* 1000 ms *//* restart from phase 0 until conditions persists */
};

/* Full Self test ok sequence */
const LEDBuzz_Sequence SEQUENCE_FULL_SELT_TEST_OK[DEF_PHASES_FULL_SELF_TEST_PASS] = {
/*phase 00*/{  500u, 0u, powerLEDOn,  buzzerOff   , false }, /* 500 ms  */
/*phase 01*/{  50u, 0u, powerLEDOff, buzzerOnReduced , true },  /* 550 ms  */
/*phase 02*/{ 200u, 0u, powerLEDOff, buzzerOff     , true }, /* 730 ms  */
/*phase 03*/{  50u, 0u, powerLEDOff, buzzerOnReduced , true },  /* 780 ms  */
/*phase 04*/{ 450u, 0u, powerLEDOff, buzzerOff    , false }, /* 1250 ms */ /* repeat phase 1 to 4, 5 times every 500ms and then stop */
};

/* Device Group test running sequence */
const LEDBuzz_Sequence SEQUENCE_DEVICE_GROUP_TEST_RUNNING[DEF_PHASES_DEV_GROUP] = {
/*phase 00*/{  50u, 0u, powerLEDOff,  buzzerOnReduced , true }, /* 50 ms   */
/*phase 01*/{  50u, 0u, powerLEDOn,  buzzerOff     , false }, /* 100 ms   */
/*phase 02*/{ 4900u, 0u, powerLEDOff, buzzerOff    , false }, /* 5000 ms *//* Repeat 60 times (5-min timeout) */
};

/* Transport Mode is active and device is mounted */
const LEDBuzz_Sequence SEQUENCE_TRANSPORT_MODE_ACTIVE_MOUNTED[DEF_PHASES_TRANSPORT_MODE_ACTIVE_MOUNTED] = {
/*phase 00*/{   10u, 0u, faultLEDOn,  buzzerOff   , false }, /* 10 ms   */
/*phase 01*/{   4990u, 0u, faultLEDOff, buzzerOff , false }, /* 4990 ms */ /*Repeats forever*/
};

const LEDBuzz_Sequence SEQUENCE_TRANSPORT_MODE_CHECK[DEF_PHASES_TRANSPORT_MODE_CHECK] = {
/*phase 00*/{ 500u, 0u, faultLEDOn,  buzzerOff , false}, /* 500 ms */
/*phase 00*/{  10u, 0u, faultLEDOff, buzzerOff , false}, /* 510 ms *//* runs only once and does not repeat */
};

/* Standby mode activate sequence */
const LEDBuzz_Sequence SEQUENCE_STANDBY_MODE_ACTIVATE[DEF_PHASES_STANDBY_MODE_ACTIVATE] = {
/*phase 00*/{ 500u, 0u, faultLEDOn,  buzzerOff , false}, /* 500 ms */
/*phase 00*/{  10u, 0u, faultLEDOff, buzzerOff , false}, /* 510 ms *//* runs only once and does not repeat */
};

/* Standby mode check sequence */
const LEDBuzz_Sequence SEQUENCE_STANDBY_MODE_CHECK[DEF_PHASES_STANDBY_MODE_CHECK] = {
/*phase 00*/{ 250u, 0u, faultLEDOn,  buzzerOff , false}, /* 250 ms */
/*phase 01*/{ 250u, 0u, faultLEDOff, buzzerOff , false}, /* 500 ms */
/*phase 02*/{ 250u, 0u, faultLEDOn,  buzzerOff , false}, /* 750 ms */
/*phase 03*/{  10u, 0u, faultLEDOff, buzzerOff , false}, /* 760 ms *//* runs only once and does not repeat */
};

/* TOM Test ALarm */
const LEDBuzz_Sequence SEQUENCE_TOM_TEST_ALARM[DEF_PHASES_TOM_TEST_ALARM] = {
/*phase 00*/{   50u, 0u, powerLEDOn,  buzzerOnReduced , true}, /* 50 ms   */
/*phase 01*/{ 4950u, 0u, powerLEDOff, buzzerOff   , false}, /* 5000 ms *//* repeat phase 1 and 2, 60 times every 500ms and then stop */
};

/* Commissioning Fail */
const LEDBuzz_Sequence SEQUENCE_COMMISSIONING_FAIL[DEF_PHASES_COMMISION_FAIL] = {
/*phase 00*/{ 50u, 0u, faultLEDOn,  buzzerOnReduced , true}, /* 50 ms */
/*phase 01*/{ 450u, 0u, faultLEDOff, buzzerOff , false}, /* 450 ms */
/*phase 02*/{ 50u, 0u, faultLEDOn,  buzzerOnReduced , true}, /* 50 ms */
/*phase 03*/{ 450u, 0u, faultLEDOff, buzzerOff , false}, /* 450 ms */  /*Repeats forever*/
};

const LEDBuzz_Sequence SEQUENCE_LOW_BATT[DEF_PHASES_LOW_BATT] = {
/*phase 00*/{ 30u, 0u, faultLEDOn,  buzzerOnReduced , true}, /* 100 ms  */
/*phase 01*/{ 0u, 5u, faultLEDOff, buzzerOff , false},  /* 50sec */ /*Repeats forever*/
};

/* Radio Pvt Data Active sequence */
const LEDBuzz_Sequence SEQUENCE_RADIO_PRIDATA_ACTIVATE[DEF_PHASES_RADIO_PRIDATA_ACTIVE] = {
/*phase 00*/{ 500u, 0u, faultLEDOn,  buzzerOff , false}, /* 500 ms */
/*phase 00*/{ 500u, 0u, faultLEDOff, buzzerOff , false}, /* 510 ms *//* runs only once and does not repeat */
};

/* Radio Pvt Data Inactive sequence */
const LEDBuzz_Sequence SEQUENCE_RADIO_PRIDATA_INACTIVATE[DEF_PHASES_RADIO_PRIDAT_INACTIVE] = {
/*phase 00*/{ 100u, 0u, faultLEDOn,  buzzerOff , false}, /* 500 ms */
/*phase 00*/{ 900u, 0u, faultLEDOff, buzzerOff , false}, /* 510 ms *//* runs only once and does not repeat */
};

/* Mounted state pattern sequence */
const LEDBuzz_Sequence SEQUENCE_MOUNTED[DEF_PHASES_MOUNTED] = {
/*phase 00*/{ 50u, 0u, allEDOff,  buzzerOnReduced , true}, /* 50 ms */
/*phase 01*/{ 10u, 0u, allEDOff, buzzerOff , false}, /* 60 ms *//* runs only once and does not repeat */
};


const LEDBuzz_PatternInfo patterns[TotalPatterns] = {
/* 00 idle pattern*/
/* This pattern stops the current pattern */
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/NULL,
/* phaseCount 	*/0u,
/* repeat 		*/0u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 01 gap pattern */
/* This pattern introduces a gap between current pattern and a new high priority pattern */
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 02 smoke alarm */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_LOCAL_SMOKE_ALARM,
/* sequence 	*/SEQUENCE_SMOKE_HEAT,
/* phaseCount 	*/DEF_PHASES_SMOKE_HEAT_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_SMOKE_HEAT,
/* repeat_phase */DEF_REPEAT_PHASE_SMOKE_HEAT,
/* reset_repeat */false },

/* 03 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 04 heat */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_LOCAL_HEAT_ALARM,
/* sequence 	*/SEQUENCE_SMOKE_HEAT,
/* phaseCount 	*/DEF_PHASES_SMOKE_HEAT_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_SMOKE_HEAT,
/* repeat_phase */DEF_REPEAT_PHASE_SMOKE_HEAT,
/* reset_repeat */false },

/* 05 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 06 co */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_LOCAL_CO_ALARM,
/* sequence 	*/SEQUENCE_CO,
/* phaseCount 	*/DEF_PHASES_CO_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_CO,
/* repeat_phase */DEF_REPEAT_PHASE_CO,
/* reset_repeat */false },

/* 07 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 08 remote smoke alarm pattern */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_REMOTE_SMOKE_ALARM,
/* sequence 	*/SEQUENCE_SMOKE_HEAT_REMOTE,
/* phaseCount 	*/DEF_PHASES_SMOKE_HEAT_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_SMOKE_HEAT,
/* repeat_phase */DEF_REPEAT_PHASE_SMOKE_HEAT,
/* reset_repeat */false },

/* 09 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 10 remote heat alarm pattern */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_REMOTE_HEAT_ALARM,
/* sequence 	*/SEQUENCE_SMOKE_HEAT_REMOTE,
/* phaseCount 	*/DEF_PHASES_SMOKE_HEAT_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_SMOKE_HEAT,
/* repeat_phase */DEF_REPEAT_PHASE_SMOKE_HEAT,
/* reset_repeat */false },

/* 11 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 12 remote co alarm pattern */
/* This pattern keeps repeating until the condition persists */
{ /* priority 	*/DEF_PRIORITY_REMOTE_CO_ALARM,
/* sequence 	*/SEQUENCE_CO_REMOTE,
/* phaseCount 	*/DEF_PHASES_CO_ALARM,
/* repeat 		*/DEF_REPEAT_COUNT_CO,
/* repeat_phase */DEF_REPEAT_PHASE_CO,
/* reset_repeat */false },

/* 13 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 14 major fault */
/* This pattern repeats phases 7 and 8 up to 60 times (i.e. 5 minutes) and then resets to phase 0 */
/* This pattern keeps repeating until the button is pressed by the user for temporary silencing */
{ /* priority 	*/DEF_PRIORITY_MAJOR_FAULT,
/* sequence 	*/SEQUENCE_MAJOR_FAULT,
/* phaseCount 	*/DEF_PHASES_MAJOR_FAULT,
/* repeat 		*/DEF_REPEAT_COUNT_MAJOR_FAULT,
/* repeat_phase */DEF_REPEAT_PHASE_MAJOR_FAULT,
/* reset_repeat */true },

/* 15 minor fault */
/* This pattern repeats phases 7 to 11 every 30s up to 20 times (i.e. 10 minutes) and then resets to phase 0 */
/* This pattern keeps repeating until the button is pressed by the user for temporary silencing */
{ /* priority 	*/DEF_PRIORITY_MINOR_FAULT,
/* sequence 	*/SEQUENCE_MINOR_FAULT,
/* phaseCount 	*/DEF_PHASES_MINOR_FAULT,
/* repeat 		*/DEF_REPEAT_COUNT_MINOR_FAULT,
/* repeat_phase */DEF_REPEAT_PHASE_MINOR_FAULT,
/* reset_repeat */true },

/* 16 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 17 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 18 user test pass */
{ /* priority 	*/DEF_PRIORITY_USER_TEST,
/* sequence 	*/SEQUENCE_USER_TEST_OK,
/* phaseCount 	*/DEF_PHASES_USER_TEST_OK,
/* repeat 		*/DEF_REPEAT_COUNT_USER_TEST_OK,
/* repeat_phase */0,
/* reset_repeat */false },

/* 19 extended user test pass */
{ /* priority 	*/DEF_PRIORITY_USER_TEST,
/* sequence 	*/SEQUENCE_EXTND_USR_TST_OK,
/* phaseCount 	*/DEF_PHASES_EXTND_USR_TST_OK,
/* repeat 		*/DEF_REPEAT_COUNT_EXTND_USR_TST_OK,
/* repeat_phase */DEF_REPEAT_PHASE_EXTND_USR_TST_OK,
/* reset_repeat */false },

/* 20 extended user test pass phase 2 */
{ /* priority 	*/DEF_PRIORITY_USER_TEST,
/* sequence 	*/SEQUENCE_EXTND_USR_TST_OK_PHASE2,
/* phaseCount 	*/DEF_PHASES_EXTND_USR_TST_OK_PHASE2,
/* repeat 		*/DEF_REPEAT_COUNT_PHASE2_EXTND_USR_TST_OK,
/* repeat_phase */DEF_REPEAT_PHASE_PHASE2_EXTND_USR_TST_OK,
/* reset_repeat */false },

/* 21 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 22 full self test running */
{ /* priority 	*/DEF_PRIORITY_USER_TEST,
/* sequence 	*/SEQUENCE_FULL_SELF_TEST_RUNNING,
/* phaseCount 	*/DEF_PHASES_FULL_SELF_TEST_RUNNING,
/* repeat 		*/DEF_REPEAT_COUNT_FULL_TEST_RUNNING,
/* repeat_phase */0,
/* reset_repeat */false },

/* 23 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 24 Full Self test pass */
{ /* priority 	*/DEF_PRIORITY_USER_TEST,
/* sequence 	*/SEQUENCE_FULL_SELT_TEST_OK,
/* phaseCount 	*/DEF_PHASES_MOUNTED_USER_TEST_OK,
/* repeat 		*/DEF_REPEAT_COUNT_MOUNTED_USER_TEST_OK,
/* repeat_phase */DEF_REPEAT_PHASE_MOUNTED_USER_TEST_OK,
/* reset_repeat */false },

/* 25 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 26 Device group test runing*/
{ /* priority 	*/DEF_PRIORITY_DEV_GROUP_TEST,
/* sequence 	*/SEQUENCE_DEVICE_GROUP_TEST_RUNNING,
/* phaseCount 	*/DEF_PHASES_DEV_GROUP,
/* repeat 		*/DEF_REPEAT_COUNT_DEV_GROUP_TEST,
/* repeat_phase */DEF_REPEAT_PHASE_DEV_GROUP_TEST,
/* reset_repeat */false },

/* 27 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 28 Transport mode active , device is mounted*/
{ /* priority 	*/DEF_PRIORITY_OPERATIONAL_MODE,
/* sequence 	*/SEQUENCE_TRANSPORT_MODE_ACTIVE_MOUNTED,
/* phaseCount 	*/DEF_PHASES_TRANSPORT_MODE_ACTIVE_MOUNTED,
/* repeat 		*/DEF_REPEAT_COUNT_TRANSPORT_ACTIVE_MOUNTED,
/* repeat_phase */DEF_REPEAT_PHASE_TRANSPORT_ACTIVE_MOUNTED,
/* reset_repeat */true },

/* 29 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 30 transport mode check */
{ /* priority 	*/DEF_PRIORITY_OPERATIONAL_MODE,
/* sequence 	*/SEQUENCE_TRANSPORT_MODE_CHECK,
/* phaseCount 	*/DEF_PHASES_TRANSPORT_MODE_CHECK,
/* repeat 		*/DEF_REPEAT_COUNT_TRANSPORT_CHECK,
/* repeat_phase */0,
/* reset_repeat */false },

/* 31 standby mode activate */
{ /* priority 	*/DEF_PRIORITY_OPERATIONAL_MODE,
/* sequence 	*/SEQUENCE_STANDBY_MODE_ACTIVATE,
/* phaseCount 	*/DEF_PHASES_STANDBY_MODE_ACTIVATE,
/* repeat 		*/DEF_REPEAT_COUNT_STANDBY_MODE_ACTIVATE,
/* repeat_phase */0,
/* reset_repeat */false },

/* 32 standby mode check */
{ /* priority 	*/DEF_PRIORITY_OPERATIONAL_MODE,
/* sequence 	*/SEQUENCE_STANDBY_MODE_CHECK,
/* phaseCount 	*/DEF_PHASES_STANDBY_MODE_CHECK,
/* repeat 		*/DEF_REPEAT_COUNT_STANDBY_MODE_CHECK,
/* repeat_phase */0,
/* reset_repeat */false },

/* 33 Commissioning Fails */
{ /* priority   */DEF_PRIORITY_USER_TEST,
/* sequence   */SEQUENCE_COMMISSIONING_FAIL,
/* phaseCount   */DEF_PHASES_COMMISION_FAIL,
/* repeat     */DEF_REPEAT_PHASE_COMMISION_FAIL,
/* repeat_phase */0,
/* reset_repeat */false },

/* 34 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 35 Low Battery */
{ /* priority   */DEF_PRIORITY_LOW_BATT,
/* sequence   */SEQUENCE_LOW_BATT,
/* phaseCount   */DEF_PHASES_LOW_BATT,
/* repeat     */0,
/* repeat_phase */0,
/* reset_repeat */false },

/* 36 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 37 Radio Private Data active */
{ /* priority   */DEF_PRIORITY_PRI_RADIO_ACT,
/* sequence   */SEQUENCE_RADIO_PRIDATA_ACTIVATE,
/* phaseCount   */DEF_PHASES_RADIO_PRIDATA_ACTIVE,
/* repeat     */60u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 38 gap pattern */
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 39 Radio Private Data inactive */
{ /* priority   */DEF_PRIORITY_PRI_RADIO_INACT,
/* sequence   */SEQUENCE_RADIO_PRIDATA_INACTIVATE,
/* phaseCount   */DEF_PHASES_RADIO_PRIDAT_INACTIVE,
/* repeat     */60u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 40 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_IDLE,
/* sequence   */SEQUENCE_GAP,
/* phaseCount   */DEF_PHASES_GAP,
/* repeat     */1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 41 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 42 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 43 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 44 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 45 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 46 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 47 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 48 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 49 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 50 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 51 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

/* 52 mounted pattern */
/* Insert a Gap in place of unused location*/
{ /* priority   */DEF_PRIORITY_OPERATIONAL_MODE,
/* sequence   */SEQUENCE_MOUNTED,
/* phaseCount   */DEF_PHASES_MOUNTED,
/* repeat     */1u,
/* repeat_phase */DEF_REPEAT_PHASE_MOUNTED,
/* reset_repeat */false },

/* 53 gap pattern */
/* Insert a Gap in place of unused location*/
{ /* priority 	*/DEF_PRIORITY_IDLE,
/* sequence 	*/SEQUENCE_GAP,
/* phaseCount 	*/DEF_PHASES_GAP,
/* repeat 		*/1u,
/* repeat_phase */0,
/* reset_repeat */false },

};


/********************************************************************************************************
*********************************************************************************************************
*                                            PROTOTYPES
*********************************************************************************************************
********************************************************************************************************/

void LEDBuzz_Task(void *p_arg) ;

static void LEDBuzz_PatternQueueEmpty(void);
static bool LEDBuzz_IsPatternQueueEmpty(void);
static uint8_t LEDBuzz_PatternQueueGet(void);
static void LEDBuzz_PatternQueueAdd(uint8_t pattern, bool isBuzzerSilent);
static void LEDBuzz_PatternQueueRemove(uint8_t pattern);

static void LEDBuzz_HandlePhaseTimeout(void);
static void LEDBuzz_HandleSilenceTimeout(uint8_t pattern);
static void LEDBuzz_HandlePatternStop(uint8_t pattern);
static void LEDBuzz_HandleStandardPatterns(uint8_t pattern);

static uint8_t LEDBuzz_PriorityGet(uint8_t pattern);
static void LEDBuzz_GoToIdleState(void);
static void LEDBuzz_PatternPause(void);
static void LEDBuzz_PausedPatternResume(void);
static void LEDBuzz_NewPatternSetup(uint8_t newPattern);
static void LEDBuzz_NextPhaseAction(void);
static void LEDBuzz_LEDBuzzStateConfigure(const LEDBuzz_Sequence *active_sequence);
static bool LEDBuzz_IsAllowedInDarkMode(void);
static void LEDBuzz_LEDRequestProcess(LEDBuzz_LED led);
static void LEDBuzz_AssistanceLEDSet(bool led_on);
static void LEDBuzz_TurnPowerLEDOn(void);
static void LEDBuzz_TurnPowerLEDOff(void);
static bool LEBBuzz_IsGAP(const LEDBuzz_Sequence *active_sequence); /*Check if thread should be forced to stay running*/

/* ToDo: This should call ambient light function */
static LEDBuzz_AmbientLightStatus LEDBuzz_AmbientLightStatusGet(void);

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

/*******************************************************************************
 * @brief   Lock buzzer
 *
 * @details This function locks the buzzer for exclusive use. It is achieved 
 *          using a Mutex. The Mutex is only obtained if it currently is not
 *          locked.
 * 
 * @note This is an internal function only to be used by the Buzzer functions.
 *       External lock/unlock functions exist for other code to use which does
 *       not use the internal flag.
*/
static void lock( void )
{
  if( buzzer_mutex_locked == false )
  {
    RTOS_ERR err;

    OSMutexPend( &buzzer_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err );
    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

    buzzer_mutex_locked = true;

    DEBUG_BUZZER( "Buzzer Locked", false, 0UL );
  }
}

/*******************************************************************************
 * @brief   UnLock buzzer
 *
 * @details This function un-locks the buzzer so it can be used by other code.
 *          It is achieved using a Mutex. The Mutex is only released if it currently
 *          locked.
 * 
 * @note This is an internal function only to be used by the Buzzer functions.
 *       External lock/unlock functions exist for other code to use which does
 *       not use the internal flag.
*/
static void unlock( void )
{
  if( buzzer_mutex_locked )
  {
    RTOS_ERR err;

    OSMutexPost( &buzzer_mutex, OS_OPT_POST_NONE, &err);
    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

    buzzer_mutex_locked = false;

    DEBUG_BUZZER( "Buzzer Un-Locked", false, 0UL );
  }
}

static bool is_phase_timeout_msg(uint8_t msg)
{
	bool ret = true; 
	if(msg == (uint8_t)(LEDBuzz_LETimerTimeout))
	{

	}
	else if(msg == (uint8_t)(LEDBuzz_BURTCTimerTimeout))
	{

	}
	else 
	{
		ret = false;
	}
	return ret;
}

static bool is_stop_msg(uint8_t msg)
{
	bool ret = true; 

	if(msg == (uint8_t)(PatternIdle))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmSmokeStop))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmHeatStop))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmCOStop))
	{

	}
	else if(msg == (uint8_t)(PatternStopMajorFault))
	{

	}
	else if(msg == (uint8_t)(PatternStopMinorFault))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmRemoteSmokeStop))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmRemoteHeatStop))
	{

	}
	else if(msg == (uint8_t)(PatternAlarmRemoteCOStop))
	{

	}
	else if(msg == (uint8_t)(PatternExtendedUserTestPassStop))
	{

	}
	else if(msg == (uint8_t)(PatternFullSelfTestRunningStop))
	{

	}
	else if(msg == (uint8_t)(PatternFullSelfTestPassCommissioningStop))
  {

  }
	else if(msg == (uint8_t)(PatternDeviceGroupTestStop))
	{

	}
	else if(msg == (uint8_t)(PatternTransportActiveMountedStop))
	{

	}
	else if(msg == (uint8_t)(PatternCommissioningFailStop))
	{

	}
	else if(msg == (uint8_t)(PatternLowBattStop))
	{

	}
	else if(msg == (uint8_t)(PatternRadioPvtDataActiveStop))
	{

	}
	else if(msg == (uint8_t)(PatternRadioPvtDataInactiveStop))
  {

	}
	else 
	{
		ret = false;
	}
	return ret;
}

static bool is_silence_timeout_msg(uint8_t msg)
{
	bool ret = true; 

	switch(msg)
	{
		case (uint8_t)(PatternFaultSilenceTimeout):
		case (uint8_t)(PatternAlarmSmokeSilcenceTimeout):
		case (uint8_t)(PatternAlarmHeatSilcenceTimeout):
		case (uint8_t)(PatternAlarmCOSilenceTimeout):
		case (uint8_t)(PatternCommissionFailSilenceTimeout):
		{
			break;
		}
		default:
		{
			ret = false;
			break;
		}
	}
	return ret;
}

static bool is_start_pattern_msg(uint8_t msg)
{
	bool ret = true;

	switch (msg)
	{
	case (uint8_t)(PatternAlarmSmoke):
	case (uint8_t)(PatternAlarmHeat):
	case (uint8_t)(PatternAlarmCO):
	case (uint8_t)(PatternUserTestPass):
	case (uint8_t)(PatternExtendedUserTestPass):
	case (uint8_t)(PatternStandbyModeActivate):
	case (uint8_t)(PatternHeartbeat):
	case (uint8_t)(PatternRadioPvtDataActive):
	case (uint8_t)(PatternRadioPvtDataInactive):
	case (uint8_t)(PatternAlarmRemoteSmoke):
	case (uint8_t)(PatternAlarmRemoteHeat):
	case (uint8_t)(PatternAlarmRemoteCO):
	case (uint8_t)(PatternDeviceGroupTestRunning):
	{
		if ((Operational_Mode == getBehavioural_System_Modes(false)) || (Functional_Test_Mode == getBehavioural_System_Modes(false)))
		{
			/*Do nothing*/
		}
		else
		{
			ret = false;
		}
		break;
	}

	case (uint8_t)(PatternMajorFault):
	case (uint8_t)(PatternMinorFault):
	case (uint8_t)(PatternFullSelfTestRunning):
	case (uint8_t)(PatternLowBatt):
	{
		if ((Operational_Mode == getBehavioural_System_Modes(false)) || (Commisioning_Mode == getBehavioural_System_Modes(false))
			|| (Functional_Test_Mode == getBehavioural_System_Modes(false)))
		{
			/*Do nothing*/
		}
		else
		{
			ret = false;
		}
		break;
	}

	case (uint8_t)(PatternFullSelfTestPassCommissioning):
	case (uint8_t)(PatternCommissioningFail):
	{
	  if ((Operational_Mode == getBehavioural_System_Modes(false)) || (Commisioning_Mode == getBehavioural_System_Modes(false))
	        || (Functional_Test_Mode == getBehavioural_System_Modes(false)))
	  {
	        /*Do nothing*/
	  }
		else
		{
			ret = false;
		}
		break;
	}

	case (uint8_t)(PatternAlarmSilence):
	{
		break;
	}

	case (uint8_t)(PatternStandbyModeCheck):
	{
		if ((Standby_Mode == getBehavioural_System_Modes(false)) || (Functional_Test_Mode == getBehavioural_System_Modes(false)))
		{
			/*Do nothing*/
		}
		else
		{
			ret = false;
		}
		break;
	}

	case (uint8_t)(PatternTransportModeCheck):
	case (uint8_t)(PatternTransportActiveMounted):
	{
		if ((Transport_Mode == getBehavioural_System_Modes(false)) || (Functional_Test_Mode == getBehavioural_System_Modes(false)))
		{
			/*Do nothing*/
		}
		else
		{
			ret = false;
		}
		break;
	}

	default:
	{
		break;
	}
	}
	return ret;
}

/****************************************************************************************************//**
*                                               LEDBuzz_Init()
*
* @brief   Create LEDBuzz task and flags.
*
* @note	(1) The following function needs to be called at the time of application initialisation
********************************************************************************************************/
void LEDBuzz_Init(void) {

	RTOS_ERR err;
    OSMutexCreate(&buzzer_mutex, "Buzzer mutex", &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
	/* LDRA_EXCLUDE 458 S : Param 3 is the same type as LEDBuzz_Task */
	/* LDRA_EXCLUDE 458 S : Param 4 is the same type as (void *)0 */
	OSTaskCreate(&LEDBuzz_TaskTCB, 										/* Pointer to the task's TCB.  				*/
	             "LedBuzzTask", 										/* Name to help debugging.     				*/
				  LEDBuzz_Task, 										/* Pointer to the task's code.	 			*/
				  &LEDBuzz_TaskTCB, 									/* Pointer to task's argument. 				*/
				  LED_BUZZ_TASK_PRIO, 									/* Task's priority.            				*/
				  &LEDBuzz_TaskStack[0], 								/* Pointer to base of stack.   				*/
				  (LED_BUZZ_TASK_STK_SIZE / 10u), 						/* Stack limit, from base.     				*/
				  LED_BUZZ_TASK_STK_SIZE, 								/* Stack size, in CPU_STK.          		*/
				  DEF_PATTERN_QUEUE_SIZE, 								/* Messages in task queue.     				*/
				  0u, 													/* Round-Robin time quanta.    				*/
				  (void *)0, 											/* External TCB data.          				*/
				  (OS_OPT_TASK_STK_CLR 									/* Task options.               				*/
				  | OS_OPT_TASK_STK_CHK ), &err);

	/* LDRA_EXCLUDE 496 S : APP_RTOS_ASSERT_DBG is defined */
	/* LDRA_EXCLUDE 496 S : RTOS_ERR_CODE_GET is defined */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	LEDBuzz_PatternQueueEmpty();										/* clear the queue 							*/
}

/****************************************************************************************************//**
*                                               LEDBuzz_Task()
*
* @brief   Task to handle led and buzzer pattern requests.
*
* @param	p_arg	pointer to arguments.
********************************************************************************************************/
void LEDBuzz_Task(void *p_arg) {
	const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)p_arg;
    RTOS_ERR err;
    OSTaskRegSet(DEF_NULL, 0, ptrToMyTCB, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    bool result = hal_AFE_RegisterResponseVar(&LEDBuzzTaskResponse);
    
	/* LDRA_EXCLUDE 28 D : This is not infinite loop. It is the task body */
	while (true)
  {
		RTOS_ERR err;
		OS_MSG_SIZE size;

	  const uint8_t *pmsg = (uint8_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err );

		/* LDRA_EXCLUDE 496 S : RTOS_ERR_CODE_GET is defined */
		if( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE )
    {
      const uint8_t requested_pattern = ( *pmsg );

      DEBUG_BUZZER( "Buzzer Pattern Request: ", true, requested_pattern );

			if( requested_pattern == ( uint8_t )( PatternStopAll ) )
      {
        /* Stop all and move to idle state */
				LEDBuzz_GoToIdleState( );
				LEDBuzz_PatternQueueEmpty( );

        DEBUG_BUZZER( "Buzzer Stop All", false, 0ul );

        unlock( );
			}
			else if( is_phase_timeout_msg( requested_pattern ) )
      {
        /* phase timed out, begin next phase */
				LEDBuzz_HandlePhaseTimeout( );

        DEBUG_BUZZER( "Buzzer Phase Timeout", false, 0ul );
			}
			else if( is_stop_msg( requested_pattern) )
      {
        /* idle pattern - to stop current pattern	*/
				LEDBuzz_HandlePatternStop( requested_pattern );

        DEBUG_BUZZER( "Buzzer Idle", false, 0ul );
			}
			else if( is_silence_timeout_msg( requested_pattern ) )
      {
				LEDBuzz_HandleSilenceTimeout( requested_pattern );

        DEBUG_BUZZER( "Buzzer Silence", false, 0ul );
			}
			else if( is_start_pattern_msg( requested_pattern ) )
      {
        /* pattern start request */
				LEDBuzz_HandleStandardPatterns( requested_pattern );

        DEBUG_BUZZER( "Buzzer Start Request", false, 0ul );
			}
			else 
			{
				/*Do nothing*/
			}
		}
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_Post()
*
* @brief	Post pattern flag to start a new pattern
*
* @param	LEDBUZZ_PATTERN_ID	flag for the pattern
********************************************************************************************************/
void LEDBuzz_Post(LEDBUZZ_PATTERN_ID flag) {
	RTOS_ERR err;
	uint8_t idx = (uint8_t) (flag);
	
	OSTaskQPost(&LEDBuzz_TaskTCB, 
				&MSG_IDs[idx],
				sizeof(uint8_t),
				OS_OPT_POST_FIFO,
				&err);
	/* LDRA_EXCLUDE 496 S : APP_RTOS_ASSERT_DBG is defined */
	/* LDRA_EXCLUDE 496 S : RTOS_ERR_CODE_GET is defined */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/****************************************************************************************************//**
*                                               LEDBuzz_PatternQueueEmpty()
*
* @brief   Clear the pattern queue
********************************************************************************************************/
static void LEDBuzz_PatternQueueEmpty(void) {
	
	uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
	for(uint8_t idx = 0; idx <= (queue_sz - 1u); idx++)
	{
		pendingPatternpool[idx].isFree = true;
	}
	LEDBuzz_CurrentPattern.id 				= (uint8_t) (PatternIdle);		/* reset current pattern 			*/
}

/****************************************************************************************************//**
*                                               LEDBuzz_IsPatternQueueEmpty()
*
* @brief   Check if pattern queue is empty
*
* @return	true, if queue is empty
* 			false, if queue is not empty
********************************************************************************************************/
static bool LEDBuzz_IsPatternQueueEmpty(void) {
	bool empty = true;
	uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
	for(uint8_t idx = 0; idx <= (queue_sz - 1u); idx++)
	{
		if(pendingPatternpool[idx].isFree == false)
		{
			empty = false;
			break;
		}
		else 
		{
			/*Do nothing*/
		}
	}
	return empty;
}

/****************************************************************************************************//**
*                                               LEDBuzz_PatternQueueGet()
*
* @brief   Read pattern from the pattern queue
*
* @return	index of the pattern
********************************************************************************************************/
static uint8_t LEDBuzz_PatternQueueGet(void) {
	uint8_t next_pattern;
	uint8_t priority = DEF_PRIORITY_IDLE; 
	uint8_t highest_priority_idx = 0;
	uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
	for(uint8_t idx = 0; idx <= (queue_sz - 1u); idx++)
	{
		if(pendingPatternpool[idx].isFree == false)
		{
			uint8_t tmp;
			tmp = LEDBuzz_PriorityGet(pendingPatternpool[idx].patternID);
			if(priority > tmp) /* Iterate till we get highest priority ID*/
			{
				priority = tmp;
				highest_priority_idx = idx;
				(void)highest_priority_idx; /*For LDRA Compliance*/
			}
			else 
			{
				/*Do nothing*/
			} 
		}
		else 
		{
			/*Do nothing*/
		}
	}
	next_pattern = pendingPatternpool[highest_priority_idx].patternID;
	LEDBuzz_Silence = pendingPatternpool[highest_priority_idx].isBuzzerSilent;
	pendingPatternpool[highest_priority_idx].isFree = true;				/*Remove Pattern from Queue*/
	return next_pattern;
}

/****************************************************************************************************//**
*                                               LEDBuzz_PatternQueueAdd()
*
* @brief   Add pattern to the the pattern queue
*
* @param	pattern	index of the pattern to add to the queue
* @param	isBuzzerSilent	Buzzer Silent State
********************************************************************************************************/
static void LEDBuzz_PatternQueueAdd(uint8_t pattern, bool isBuzzerSilent)
{

	if (pattern != (uint8_t)(PatternIdle))
	{
		bool pattern_exist = false;
		uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
		uint8_t idx;
		for(idx = 0; idx <= (queue_sz - 1u); idx++) /*Check if pattern already exists*/
		{
			if (pendingPatternpool[idx].isFree == true)
			{
				/*Do nothing*/
			}
			else
			{
				/*Occupied location, check if pattern already exists*/
				if (pendingPatternpool[idx].patternID == pattern)
				{
					pattern_exist = true;
					break; /* Pattern already exists, no need to add it */
				}
				else
				{
					/*Do nothing*/
				}
			}
		}

		if (false == pattern_exist)
		{
			for (idx = 0; idx <= (queue_sz - 1u); idx++) /*Add the new pattern into the first empty location*/
			{
				if (pendingPatternpool[idx].isFree == true)
				{
					pendingPatternpool[idx].patternID = pattern;
					pendingPatternpool[idx].isFree = false;
					pendingPatternpool[idx].isBuzzerSilent = isBuzzerSilent;
					break;
				}
				else
				{
					/*Do nothing*/
				}
			}
		}
		else
		{
			/*Do nothing*/
		}
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_PatternQueueRemove()
*
* @brief   remove pattern from the the pattern queue
*
* @param	pattern	index of the pattern to remove from the queue
********************************************************************************************************/
static void LEDBuzz_PatternQueueRemove(uint8_t pattern)
{
	uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
	for(uint8_t idx = 0; idx <= (queue_sz - 1u); idx++)
	{
		if (pendingPatternpool[idx].patternID == pattern)
		{
			pendingPatternpool[idx].isFree = true;
			break; 
		}
		else
		{
			/*Do nothing*/
		}
	}
}
/****************************************************************************************************//**
*                                               LEDBuzz_HandlePhaseTimeout()
*
* @brief   Handler for Phase Timeout
********************************************************************************************************/
static void LEDBuzz_HandlePhaseTimeout(void)
{
	RTOS_ERR err;
	if (LEDBuzz_HighPriorityPatternPending == true)
	{
	    /* check if high priority pattern is pending due to buzzer? 								*/
	    LEDBuzz_GoToIdleState();
	    if (LEDBuzz_CurrentPattern.id != (uint8_t)(PatternGap))
	    {
	        /* add 500ms delay in led flashing and Buzzer sound before starting high priority pattern 	*/
	        LEDBuzz_PatternPause();
	        LEDBuzz_NewPatternSetup((uint8_t) PatternGap);
	        LEDBuzz_NextPhaseAction();
	    }
	    else
	    {
	        LEDBuzz_PausedPatternResume();
	        LEDBuzz_HighPriorityPatternPending = false;
	    }
	}
	else if(LEDBuzz_CurrentPattern.id == (uint8_t)(PatternIdle))
	{
	    unlock( );
		/*Ignore, late message of a stopped pattern*/
	}
	else
	{
	    bool do_next_phase = false;
	    uint8_t last_phase;																							          /* current phase has timed out so move to next phase of pattern */
	    LEDBuzz_CurrentPattern.phase++;

	    last_phase = patterns[LEDBuzz_CurrentPattern.id].phaseCount - 1u;					/* calculate the last phase of the current pattern */

	    if (LEDBuzz_CurrentPattern.phase <= last_phase) 	                        /* in middle of pattern, run the next phase */
	    {
	        do_next_phase = true;
	    }
	    else
	    {																								                          /* last phase is complete	*/
	        if (patterns[LEDBuzz_CurrentPattern.id].repeat == 0u) 							  /* if the pattern needs to repeat indefinitely? */
	        {
	            LEDBuzz_CurrentPattern.phase = patterns[LEDBuzz_CurrentPattern.id].repeat_phase; 			/* start from the specific phase */
	            do_next_phase = true;
	        }
	        else 																						                      /* pattern needs to repeat only certain number of times */
	        {
	            LEDBuzz_CurrentPattern.repeatCounter++;														/* increase the repeat counter value */

	            if (LEDBuzz_CurrentPattern.repeatCounter < patterns[LEDBuzz_CurrentPattern.id].repeat) 	/* if the max number of repeats not reached yet */
	            {
	                LEDBuzz_CurrentPattern.phase = patterns[LEDBuzz_CurrentPattern.id].repeat_phase;		/* start from the specific phase */
	                do_next_phase = true;
	            }
	            else
	            {																						                      /* the max number of repeats has reached */
	                if (patterns[LEDBuzz_CurrentPattern.id].reset_repeat == true) /* if reseting the repeat counter required? */
	                {
	                    LEDBuzz_CurrentPattern.repeatCounter = 0;									/* reset the repeat counter */
	                    LEDBuzz_CurrentPattern.phase = 0;
	                    do_next_phase = true;
	                }
	                else
	                {																					                    /* nothing else to do in the current pattern */
	                    /* Do nothing */
	                }
	            }
	        }
		  }

	    if (do_next_phase == true)
	    {
	        LEDBuzz_NextPhaseAction();
	    }
	    else
	    {
	        if (LEDBuzz_CurrentPattern.id == (uint8_t)(PatternExtendedUserTestPass))
	        {
	            LEDBuzz_NewPatternSetup((uint8_t)(PatternPhase2ExtendedUserTestPass));
	            LEDBuzz_NextPhaseAction();
	        }
	        else
	        {
	            LEDBuzz_GoToIdleState();
	            LEDBuzz_PausedPatternResume();
	        }
	    }
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_HandlePatternStop()
*
* @brief   Handler for Pattern Stop request
* @param	pattern	pattern to stop
 ********************************************************************************************************/
static void LEDBuzz_HandlePatternStop(uint8_t pattern) {
	
    RTOS_ERR err;
	uint8_t pattern_to_stop = (uint8_t)(PatternIdle);
	if(pattern == (uint8_t)(PatternAlarmSmokeStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmSmoke);
	}
	else if(pattern == (uint8_t)(PatternAlarmHeatStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmHeat);
	}
	else if(pattern == (uint8_t)(PatternAlarmCOStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmCO);
	}
	else if(pattern == (uint8_t)(PatternStopMajorFault))
	{
		pattern_to_stop = (uint8_t)(PatternMajorFault);
	}
	else if(pattern == (uint8_t)(PatternStopMinorFault))
	{
		pattern_to_stop = (uint8_t)(PatternMinorFault);
	}
	else if(pattern == (uint8_t)(PatternAlarmRemoteSmokeStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmRemoteSmoke);
	}
	else if(pattern == (uint8_t)(PatternAlarmRemoteHeatStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmRemoteHeat);
	}
	else if(pattern == (uint8_t)(PatternAlarmRemoteCOStop))
	{
		pattern_to_stop = (uint8_t)(PatternAlarmRemoteCO);
	}
	else if(pattern == (uint8_t)(PatternExtendedUserTestPassStop))
	{
		pattern_to_stop = (uint8_t)(PatternExtendedUserTestPass);
	}
	else if(pattern == (uint8_t)(PatternFullSelfTestRunningStop))
	{
		pattern_to_stop = (uint8_t)(PatternFullSelfTestRunning);
	}
	else if(pattern == (uint8_t)(PatternFullSelfTestPassCommissioningStop))
	{
	   pattern_to_stop = (uint8_t)(PatternFullSelfTestPassCommissioning);
	}
	else if(pattern == (uint8_t)(PatternDeviceGroupTestStop))
	{
		pattern_to_stop = (uint8_t)(PatternDeviceGroupTestRunning);
	}
	else if(pattern == (uint8_t)(PatternTransportActiveMountedStop))
	{
		pattern_to_stop = (uint8_t)(PatternTransportActiveMounted);
	}
	else if(pattern == (uint8_t)(PatternCommissioningFailStop))
	{
	  pattern_to_stop = (PatternCommissioningFail);
	}
	else if(pattern == (uint8_t)(PatternLowBattStop))
	{
	  pattern_to_stop = (uint8_t)(PatternLowBatt);
	}
	else if(pattern == (uint8_t)(PatternRadioPvtDataActiveStop))
	{
	    pattern_to_stop = (uint8_t)(PatternRadioPvtDataActive);
	}
	else if (pattern == (uint8_t)(PatternRadioPvtDataInactiveStop))
	{
	      pattern_to_stop = (uint8_t)(PatternRadioPvtDataInactive);
	}
	else 
	{
	    /*Do nothing*/
	}

	LEDBuzz_PatternQueueRemove(pattern_to_stop);
	/*Check if the pattern to be stopped is currently on display*/
	if((LEDBuzz_CurrentPattern.id == pattern_to_stop) || 
	/*Count for the special pattern of Extended user test*/
	((LEDBuzz_CurrentPattern.id == (uint8_t)(PatternPhase2ExtendedUserTestPass)) && (pattern_to_stop == (uint8_t)(PatternExtendedUserTestPass))))
	{
		/*Stop patterns and move to next*/
		LEDBuzz_GoToIdleState();
		if (LEDBuzz_IsPatternQueueEmpty() == false) {			/* check if there is any pattern in the queue? 		*/
			uint8_t next_pattern;
			next_pattern = LEDBuzz_PatternQueueGet(); 				/* get the pattern from the queue					*/
			LEDBuzz_NewPatternSetup(next_pattern); 					/* set all variables for new pattern				*/
			LEDBuzz_NextPhaseAction(); 								/* start first phase of pattern						*/
		}
		else {
      LEDBuzz_PatternQueueEmpty();

      unlock( );
		}
	}
	else 
	{
		/*Do nothing*/
	}

}

/****************************************************************************************************//**
*                                               LEDBuzz_HandleSilenceTimeout()
*
* @brief   Reenable pattern buzzer
********************************************************************************************************/
void LEDBuzz_ReenablePatternBuzzer(uint8_t pattern)
{
	if (pattern == LEDBuzz_CurrentPattern.id)
	{
		LEDBuzz_Silence = false;
		LEDBuzz_NewPatternSetup(pattern); /* Used to modify the fault prompt sound immediately after the end of fault silence */
		LEDBuzz_NextPhaseAction();
	}
	else
	{
		/*Search for the pattern in the queue, so that when the pattern is resumed, it will remember its silence state */
		uint8_t queue_sz = (uint8_t)(DEF_PATTERN_QUEUE_SIZE);
		for (uint8_t idx = 0; idx <= (queue_sz - 1u); idx++)
		{
		    if ((pendingPatternpool[idx].patternID == pattern) && (pendingPatternpool[idx].isFree == false))
		    {
		        pendingPatternpool[idx].isBuzzerSilent = false;
		        break;
		    }
		    else
		    {
		        /*Do nothing*/
		    }
		}
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_HandleSilenceTimeout()
*
* @brief   Handle Silence timeout
********************************************************************************************************/
static void LEDBuzz_HandleSilenceTimeout(uint8_t pattern)
{
	if(pattern == (uint8_t)(PatternFaultSilenceTimeout))
	{
		/**/
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternMinorFault));
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternMajorFault));
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternLowBatt));
	}
	else if(pattern == (uint8_t)(PatternAlarmSmokeSilcenceTimeout))
	{
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternAlarmSmoke));
	}
	else if(pattern == (uint8_t)(PatternAlarmHeatSilcenceTimeout))
	{
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternAlarmHeat));
	}
	else if(pattern == (uint8_t)(PatternAlarmCOSilenceTimeout))
	{
		LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternAlarmCO));
	}
	else if(pattern == (uint8_t)(PatternCommissionFailSilenceTimeout))
	{
	    LEDBuzz_ReenablePatternBuzzer((uint8_t)(PatternCommissioningFail));
	}
	else 
	{
		/*Do nothing*/
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_GoToIdleState()
*
* @brief   Initiate Idle State
********************************************************************************************************/
static void LEDBuzz_GoToIdleState(void) {
    RTOS_ERR err;
	(void)LETimer_stop(LETIMER_LEDBUZZ);											/* stop LE timer 					*/
	(void)BURTCTimer_Stop(LedBuzz_event_1);										/* stop BURTC timer 				*/
	
	PWM_Timer0_Stop();

	if(LEDBuzz_PreviousBuzzerStatus == buzzerOnHigh){						/* turn off buzzer if it is on 		*/

		LEDBuzz_BuzzerTurnOffHighFreq();
	}
	else if((LEDBuzz_PreviousBuzzerStatus == buzzerOnLow) || (LEDBuzz_PreviousBuzzerStatus == buzzerOnReduced)){
		LEDBuzz_BuzzerTurnOffLowFreq();
	}
	else {
		/* nothing to do - buzzer is already off */
	}

	LEDBuzz_GAP = true;
	LEDBuzz_PreviousBuzzerStatus 	= buzzerOff;
	GPIO_TurnSmokeLEDOff();													/* turn off all leds 				*/
	GPIO_TurnCOLEDOff();
	GPIO_TurnFaultLEDOff();
	LEDBuzz_TurnPowerLEDOff();
}

/****************************************************************************************************//**
*                                               LEDBuzz_HandleStandardPatterns()
*
* @brief	Handler for standard patterns
*
* @param	pattern	pattern to run
*
* @note		(1)  PTR-1488
* 			(2)  PTR-1385
* 			(3)  PTR-1465
* 			(4)  PTR-1464
* 			(5)  PTR-1457
* 			(6)  PTR-1452
* 			(7)  PTR-1361
* 			(8)  PTR-1232
* 			(9)  PTR-1166
* 			(10) PTR-1430
* 			(11) PTR-1429
* 			(12) PTR-1489
*			(13) PTR-1483
*			(14) PTR-1431
********************************************************************************************************/
static void LEDBuzz_HandleStandardPatterns(uint8_t pattern)
{
  behaviour_state_enum_operational_States op_state = getBehavioural_Operational_State();
  if(((hal_get_ads_state() == Ads_onBase) || ((FaultHandler_GetFaultFlags() & DEF_DEM_TOO_LONG_FAULT) != 0u))
       || ((hal_get_ads_state() == Ads_offBase) && 
          (Standby_Mode == getBehavioural_System_Modes(false)) && (GetStandByModeCheckButton() == true)) 
       || ((hal_get_ads_state() == Ads_offBase) && 
       ((State_Radio_PriData_Configuration == op_state) || ((State_Airing_Configuration == op_state)))))
  {

      SetStandByModeCheckButton(false);

      if(pattern == (uint8_t)(PatternAlarmSilence))
      {
          LEDBuzz_Silence = true;
          (void)(LEDBuzz_Silence); /*LDRA Compliance*/
      }
      else if (pattern == LEDBuzz_CurrentPattern.id) /* Same pattern request was sent, only reset buzzer Silencer*/
      {
          /* Same pattern request received, Ignore*/
      }
      else if (LEDBuzz_PriorityGet(pattern) < LEDBuzz_PriorityGet(LEDBuzz_CurrentPattern.id))
      {	  /* compare the priority of the requested pattern */

          bool pend_pattern = false;
          if (LEDBuzz_CurrentPattern.id != (uint8_t) (PatternIdle))
          {
              /* if current pattern is not the idle pattern */
              if (LEDBuzz_BuzzerStatus == buzzerOff)
              {
                  /* buzzer is currently off*/
                  pend_pattern = false;										/* requested high priority pattern can run immediately 					*/
              }
              else
              {
                  /* buzzer is currently on */
                  pend_pattern = true;										/* requested high priority pattern cannot run immediately 				*/
              }
          }
          else
          {																/* current pattern is the idle pattern 									*/
              pend_pattern = false;											/* requested high priority pattern can run immediately 					*/
          }

          if (pend_pattern == true)
          {
             /* buzzer is on , hence the requested pattern cannot start immediately 	*/
              (void)(LEDBuzz_HighPriorityPatternPending);
              LEDBuzz_HighPriorityPatternPending = true;
              LEDBuzz_PatternQueueAdd(pattern, false); /* add to queue 										*/
          }
          else
          {
              LEDBuzz_GoToIdleState();															/* buzzer is off, pause the current pattern and start the new pattern 	*/
              LEDBuzz_PatternPause();
              LEDBuzz_NewPatternSetup(pattern);
              LEDBuzz_NextPhaseAction();
          }
      }
      else
      {
          LEDBuzz_PatternQueueAdd(pattern, false); /* add to queue 										*/
      }
  }
}

/****************************************************************************************************//**
*                                               LEDBuzz_PriorityGet()
*
* @brief   Get priority of the pattern
*
* @param	pattern	index value of the pattern
*
* @return  priority of the pattern
*
* @note		(1) PTR-1488
********************************************************************************************************/
static uint8_t LEDBuzz_PriorityGet(uint8_t pattern) {
	uint8_t priority;


	switch (pattern) {
	case (uint8_t)(PatternAlarmSmoke):
		priority = DEF_PRIORITY_LOCAL_SMOKE_ALARM;
		break;

	case (uint8_t)(PatternAlarmHeat):
		priority = DEF_PRIORITY_LOCAL_HEAT_ALARM;
		break;

	case (uint8_t)(PatternAlarmCO):
		priority = DEF_PRIORITY_LOCAL_CO_ALARM;
		break;

	case (uint8_t)(PatternAlarmRemoteSmoke):
		priority = DEF_PRIORITY_REMOTE_SMOKE_ALARM;
		break;

	case (uint8_t)(PatternAlarmRemoteHeat):
		priority = DEF_PRIORITY_REMOTE_HEAT_ALARM;
		break;

	case (uint8_t)(PatternAlarmRemoteCO):
		priority = DEF_PRIORITY_REMOTE_CO_ALARM;
		break;

	case (uint8_t)(PatternMajorFault):
		priority = DEF_PRIORITY_MAJOR_FAULT;
		break;

	case (uint8_t)(PatternMinorFault):
		priority = DEF_PRIORITY_MINOR_FAULT;
		break;

	case (uint8_t)(PatternRadioPvtDataActive):
    priority = DEF_PRIORITY_PRI_RADIO_ACT;
  break;

	case (uint8_t)(PatternRadioPvtDataInactive):
	  priority = DEF_PRIORITY_PRI_RADIO_INACT;
	break;

	case (uint8_t)(PatternLowBatt):
    priority = DEF_PRIORITY_LOW_BATT;
	break;

	case (uint8_t)(PatternUserTestPass):
	case (uint8_t)(PatternExtendedUserTestPass):
	case (uint8_t)(PatternFullSelfTestRunning):
	case (uint8_t)(PatternFullSelfTestPassCommissioning):
	case (uint8_t)(PatternDeviceGroupTestRunning):
	case (uint8_t)(PatternTransportActiveMounted):
	case (uint8_t)(PatternTransportModeCheck):
	case (uint8_t)(PatternStandbyModeActivate):
	case (uint8_t)(PatternStandbyModeCheck):
	case (uint8_t)(PatternTOMTestAlarm):
	case (uint8_t)(PatternCommissioningFail):
	case (uint8_t)(PatternMounted): 
		priority = DEF_PRIORITY_USER_TEST;
		break;

	default:
		priority = DEF_PRIORITY_IDLE;
		break;
	}

	return priority;
}

/****************************************************************************************************//**
*                                               LEDBuzz_PatternPause()
*
* @brief    Pause current pattern
********************************************************************************************************/
static void LEDBuzz_PatternPause(void) {

	LEDBuzz_PatternQueueAdd(LEDBuzz_CurrentPattern.id, LEDBuzz_Silence); /*Enqueue the paused pattern*/
	LEDBuzz_Silence = false;
}

/****************************************************************************************************//**
*                                               LEDBuzz_PausedPatternResume()
*
* @brief    Resume previously paused pattern
********************************************************************************************************/
static void LEDBuzz_PausedPatternResume(void) {
    RTOS_ERR err;
	if (LEDBuzz_IsPatternQueueEmpty() == false)
	{ /* if no pattern pending in the queue 							*/
		LEDBuzz_NewPatternSetup(LEDBuzz_PatternQueueGet()); /* begin new pattern from the Q 								*/
		LEDBuzz_NextPhaseAction();							/* start first phase of new pattern 							*/
	}
	else
	{
    unlock( );

		LEDBuzz_CurrentPattern.id = (uint8_t)(PatternIdle); /* reset current pattern 			*/
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_NewPatternSetup()
*
* @brief   Setup new pattern
*
* @param	newPattern index value of the pattern
********************************************************************************************************/
static void LEDBuzz_NewPatternSetup(uint8_t newPattern) {
	(void)(LEDBuzz_CurrentPattern.phase);   /*LDRA Compliance*/
	(void)(LEDBuzz_CurrentPattern.repeatPhase); /*LDRA Compliance*/
	(void)(LEDBuzz_CurrentPattern.repeatCounter); /*LDRA Compliance*/
	(void)(LEDBuzz_CurrentPattern.priority); /*LDRA Compliance*/
	LEDBuzz_CurrentPattern.id 				= newPattern;
	LEDBuzz_CurrentPattern.phase 			= 0u;
	LEDBuzz_CurrentPattern.repeatCounter 	= 0u;
	LEDBuzz_CurrentPattern.priority 		= patterns[LEDBuzz_CurrentPattern.id].prioity;
	LEDBuzz_CurrentPattern.repeatPhase 		= patterns[LEDBuzz_CurrentPattern.id].repeat_phase;
}

/****************************************************************************************************//**
*                                               LEDBuzz_NextPhaseAction()
*
* @brief   Commit actions required by the current phase
********************************************************************************************************/
static void LEDBuzz_NextPhaseAction(void) {
	LEDBuzz_Sequence activeSequence;

	if (patterns[LEDBuzz_CurrentPattern.id].sequence != NULL) {
		memcpy(&activeSequence, &patterns[LEDBuzz_CurrentPattern.id].sequence[LEDBuzz_CurrentPattern.phase], sizeof(LEDBuzz_Sequence));	/* get the sequence 				*/

		LEDBuzz_LEDBuzzStateConfigure(&activeSequence);													/* configure LED & Buzzer states	*/
		(void)LETimer_stop(LETIMER_LEDBUZZ);															/* stop LE timer 					*/
		(void)BURTCTimer_Stop(LedBuzz_event_1);															/* stop BURTC timer 				*/
		if (activeSequence.BURTCPeriod > 0u)
		{
			BURTCTimer_Start(LedBuzz_event_1, false, activeSequence.BURTCPeriod);						/* configure BURTC timer			*/
		}

		if (activeSequence.LETimerPeriod > 0u) {
			LETimer_start(LETIMER_LEDBUZZ, activeSequence.LETimerPeriod);								/* configure LE timer				*/
		}

		LEDBuzz_GAP = LEBBuzz_IsGAP(&activeSequence);
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_LEDBuzzStateConfigure()
*
* @brief	Configure the state of the LEDs and Buzzer for the current phase
*
* @param	active_sequence current phase in the sequence
*
* @note		(1) PTR-1487
* 			(2) PTR-1385
* 			(3) PTR-1362
* 			(6) PTR-1489
* 			(7) PTR-1435
********************************************************************************************************/
static void LEDBuzz_LEDBuzzStateConfigure(const LEDBuzz_Sequence *active_sequence) {
	static LEDBuzz_Timer0State LEDBuzz_Timer0Status	= timer0Stop;
	LEDBuzz_LED led = active_sequence->led;
    RTOS_ERR err;

    if(true == active_sequence->acquireMutex)
    {
      lock( );
    }
    else
    {
      unlock( );
    }

	switch (active_sequence->buzzerState) {								/* configure Buzzer state for next phase									*/
	case buzzerOnLow:
		if (( (LEDBuzz_AmbientLightStatusGet() == dark) && (LEDBuzz_IsAllowedInDarkMode() == false)) || (LEDBuzz_Silence == true)) {
			LEDBuzz_BuzzerStatus = buzzerOff;
		}
		else {
			LEDBuzz_BuzzerStatus = buzzerOnLow;
		}
		break;

	case buzzerOnHigh:
		if (( (LEDBuzz_AmbientLightStatusGet() == dark) && (LEDBuzz_IsAllowedInDarkMode() == false)) || (LEDBuzz_Silence == true)) {
			LEDBuzz_BuzzerStatus = buzzerOff;
		}
		else {
			LEDBuzz_BuzzerStatus = buzzerOnHigh;
		}
		break;

	case buzzerOff:
		LEDBuzz_BuzzerStatus = buzzerOff;
		break;


	case buzzerOnReduced:
	    if (( (LEDBuzz_AmbientLightStatusGet() == dark) && (LEDBuzz_IsAllowedInDarkMode() == false)) || (LEDBuzz_Silence == true)) {
	      LEDBuzz_BuzzerStatus = buzzerOff;
	    }
	    else {
	      LEDBuzz_BuzzerStatus = buzzerOnReduced;
	    }
	    break;

	default:
		/* no other condition to test */
		break;
	}

	if (LEDBuzz_BuzzerStatus == buzzerOnHigh) {							/* work out next timer state - using the previous and current buzzer status	*/

		if (LEDBuzz_PreviousBuzzerStatus == buzzerOnHigh) {
			LEDBuzz_Timer0Status = timer0NoChange;
		}
		else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnLow) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0StopStartHighFreq;
		}
        else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnReduced) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0StopStartHighFreq;
		}
		else {															/* LEDBuzz_BuzzerStatus == buzzerOff 										*/
			LEDBuzz_Timer0Status = timer0StartHighFreq;
		}

		LEDBuzz_PreviousBuzzerStatus = buzzerOnHigh;
	}
	else if (LEDBuzz_BuzzerStatus == buzzerOnLow) {

		if (LEDBuzz_PreviousBuzzerStatus == buzzerOnHigh) {
			LEDBuzz_BuzzerTurnOffHighFreq();
			LEDBuzz_Timer0Status = timer0StopStartLowFreq;
		}
		else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnLow) {
			LEDBuzz_Timer0Status = timer0NoChange;
		}
        else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnReduced) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0StopStartLowFreq;
		}
		else {															/* LEDBuzz_BuzzerStatus == buzzerOff 										*/
			LEDBuzz_Timer0Status = timer0StartLowFreq;
		}

		LEDBuzz_PreviousBuzzerStatus = buzzerOnLow;
	}
    else if (LEDBuzz_BuzzerStatus == buzzerOnReduced) {

		if (LEDBuzz_PreviousBuzzerStatus == buzzerOnHigh) {
			LEDBuzz_BuzzerTurnOffHighFreq();
			LEDBuzz_Timer0Status = timer0StopStartLowReducedFreq;
		}
		else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnLow) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0StopStartLowReducedFreq;
		}
        else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnReduced) {
			LEDBuzz_Timer0Status = timer0NoChange;
		}
		else {															/* LEDBuzz_BuzzerStatus == buzzerOff 										*/
			LEDBuzz_Timer0Status = timer0StartLowReducedFreq;
		}

		LEDBuzz_PreviousBuzzerStatus = buzzerOnReduced;
	}
	else 																/* LEDBuzz_BuzzerStatus == buzzerOff 										*/
	{

		if (LEDBuzz_PreviousBuzzerStatus == buzzerOnHigh) {
			LEDBuzz_BuzzerTurnOffHighFreq();
			LEDBuzz_Timer0Status = timer0Stop;
		}
		else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnLow) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0Stop;
		}
		else if (LEDBuzz_PreviousBuzzerStatus == buzzerOnReduced) {
			LEDBuzz_BuzzerTurnOffLowFreq();
			LEDBuzz_Timer0Status = timer0Stop;
		}
		else {															/* LEDBuzz_BuzzerStatus == buzzerOff 										*/
			LEDBuzz_Timer0Status = timer0NoChange;
		}

		LEDBuzz_PreviousBuzzerStatus = buzzerOff;
	}


	switch (LEDBuzz_Timer0Status) {										/* configure timer-0 and buzzer 											*/
	case timer0Stop:
		PWM_Timer0_Stop();
		break;

	case timer0StartLowFreq:
		LEDBuzz_BuzzerTurnOnLowFreq();
		PWM_Timer0_Start(DEF_BUZZER_LOW_FREQUENCY_hz, 0.5);
		break;

	case timer0StartLowReducedFreq:
	  LEDBuzz_BuzzerTurnOnLowFreq();
		PWM_Timer0_Start(DEF_BUZZER_LOW_REDUCED_FREQUENCY_hz, 0.5);
		break;
        
	case timer0StartHighFreq:
		PWM_Timer0_Stop(); /* No PWM in Hi Sounder*/
		LEDBuzz_BuzzerTurnOnHighFreq();
		break;

	case timer0StopStartLowFreq:
		PWM_Timer0_Stop();
		LEDBuzz_BuzzerTurnOnLowFreq();
		PWM_Timer0_Start(DEF_BUZZER_LOW_FREQUENCY_hz, 0.5);
		break;

	case timer0StopStartLowReducedFreq:
		PWM_Timer0_Stop();
		LEDBuzz_BuzzerTurnOnLowFreq();
		PWM_Timer0_Start(DEF_BUZZER_LOW_REDUCED_FREQUENCY_hz, 0.5);
		break;

	case timer0StopStartHighFreq:
		PWM_Timer0_Stop();
		LEDBuzz_BuzzerTurnOnHighFreq();
		break;

	case timer0NoChange:												/* if timer-0 is on then leave it on, if it is off then leave it off 		*/
		break;

	default:
		/* no action */
		break;

	}

	/*Service LED after Buzzer so that there will be no time shift*/
	LEDBuzz_LEDRequestProcess(led);										/* configure LED state 														*/
}

/****************************************************************************************************//**
*                                               LEDBuzz_LEDRequestProcess()
*
* @brief	Turn the LED on or off
*
* @param	led target led to turn off or turn on
********************************************************************************************************/
static void LEDBuzz_LEDRequestProcess(LEDBuzz_LED led) {

	switch (led) {
	case allEDOff:
		GPIO_TurnSmokeLEDOff();
		GPIO_TurnCOLEDOff();
		GPIO_TurnFaultLEDOff();
		LEDBuzz_TurnPowerLEDOff();
		break;

	case smokeLEDOn:
	  if(LEDBuzz_GetFTMPulseTone() == false)    /* This is always false except in FTM mode pulse tone test*/
	  {
	      GPIO_TurnSmokeLEDOn();
	  }
		break;

	case smokeLEDOff:
		GPIO_TurnSmokeLEDOff();
		break;

	case coLEDOn:
		GPIO_TurnCOLEDOn();
		break;

	case coLEDOff:
		GPIO_TurnCOLEDOff();
		break;

	case faultLEDOn:
		if ((LEDBuzz_AmbientLightStatusGet() == dark) && (LEDBuzz_IsAllowedInDarkMode() == false)) {
			GPIO_TurnFaultLEDOff();
		}
		else {
			GPIO_TurnFaultLEDOn();
		}
		break;

	case faultLEDOff:
		GPIO_TurnFaultLEDOff();
		break;

	case airingLEDOn:
	  SPIComms_Send_Data_to_MCU2(SPI_CMD_Airing_Light);
		break;

	case airingLEDOff:
	  SPIComms_Send_Data_to_MCU2(SPI_CMD_Airing_Light);
		break;

	case assistanceLEDOn:
		LEDBuzz_AssistanceLEDSet(true);
		break;

	case assistanceLEDOff:
		LEDBuzz_AssistanceLEDSet(false);
		break;

	case powerLEDOn:
		if (LEDBuzz_IsAllowedInDarkMode() == true) {
			LEDBuzz_TurnPowerLEDOn();
		}
		else {
			LEDBuzz_TurnPowerLEDOff();
		}
		break;

	case powerLEDOff:
		LEDBuzz_TurnPowerLEDOff();
		break;

	default:
		/* no action */
		break;
	}
}

/****************************************************************************************************//**
*                                               LEDBuzz_IsAllowedInDarkMode()
*
* @brief	Check if the pattern is allowed to turn on LED and Buzzer in the dark
*
* return	true, pattern allowed in the dark mode
* 			false, pattern not allowed in the dark mode
*
* @note		(1) PTR-1487
********************************************************************************************************/
static bool LEDBuzz_IsAllowedInDarkMode(void) {
	bool allowedInDark = false;

	switch (LEDBuzz_CurrentPattern.id) {
	case (uint8_t)(PatternAlarmSmoke):
	case (uint8_t)(PatternAlarmHeat):
	case (uint8_t)(PatternAlarmCO):
	case ((uint8_t)(PatternAlarmRemoteSmoke)):
	case ((uint8_t)(PatternAlarmRemoteHeat)):
	case (uint8_t)(PatternAlarmRemoteCO):
	case (uint8_t)(PatternMajorFault):
	case (uint8_t)(PatternUserTestPass):
	case ((uint8_t)(PatternExtendedUserTestPass)):
	case (uint8_t)(PatternPhase2ExtendedUserTestPass):
	case ((uint8_t)(PatternFullSelfTestRunning)):
	case ((uint8_t)(PatternTransportModeCheck)):
	case ((uint8_t)(PatternStandbyModeActivate)):
	case ((uint8_t)(PatternStandbyModeCheck)):
	case ((uint8_t)(PatternDeviceGroupTestRunning)):
	case ((uint8_t)(PatternFullSelfTestPassCommissioning)):
	case ((uint8_t)(PatternRadioPvtDataActive)):
	case ((uint8_t)(PatternRadioPvtDataInactive)):
	case ((uint8_t)(PatternCommissioningFail)):
	case ((uint8_t)(PatternLowBatt)):
	case ((uint8_t)(PatternHeartbeat)):
	case ((uint8_t)(PatternMounted)):

		allowedInDark = true;
		break;

	case (uint8_t)(PatternMinorFault):
	  if(get_SevenDays_Darkness_Status() == true)
	  {
	      allowedInDark = true;
	  }
	  else
	  {
	      allowedInDark = false;
	  }
		break;

	default:
		allowedInDark = false;
		break;
	}

	return allowedInDark;
}

/****************************************************************************************************//**
*                                               LEDBuzz_HeartbeatRequestProcess()
*
* @brief	Process heart-beat request
********************************************************************************************************/
void LEDBuzz_HeartbeatRequestProcess(void) {
    RTOS_ERR err;
    OS_MSG_SIZE size;
		(void)hal_AFE_Post(setup_Heartbeat, NULL, NULL, true);
        AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

}

/****************************************************************************************************//**
*                                               LEDBuzz_TurnPowerLEDOn()
*
* @brief	Process heart-beat request
********************************************************************************************************/
static void LEDBuzz_TurnPowerLEDOn(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;

	(void)hal_AFE_Post(setup_powerLEDOn, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/****************************************************************************************************//**
*                                               LEDBuzz_TurnPowerLEDOff()
*
* @brief	Process heart-beat request
********************************************************************************************************/
static void LEDBuzz_TurnPowerLEDOff(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;

	(void)hal_AFE_Post(setup_powerLEDOff, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/****************************************************************************************************//**
*                                        LEDBuzz_BuzzerTurnOnHighFreq()
*
* @brief	Turn the buzzer on - high frequency and high volume
********************************************************************************************************/
void LEDBuzz_BuzzerTurnOnHighFreq(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;
    
	(void)hal_AFE_Post(setup_Buzzer_3_wire_init, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	(void)hal_AFE_Post(setup_Buzzer_3_wire_on, &bist_result, &result, true);
    afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	GPIO_PinModeSet(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN, gpioModePushPull, 1);
}

/****************************************************************************************************//**
*                                         LEDBuzz_BuzzerTurnOffHighFreq()
*
* @brief	Turn the buzzer off
********************************************************************************************************/
void LEDBuzz_BuzzerTurnOffHighFreq(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;

	(void)hal_AFE_Post(setup_Buzzer_3_wire_oFF, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	(void)hal_AFE_Post(setup_Buzzer_3_wire_deinit, &bist_result, &result, true);
    afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	GPIO_PinModeSet(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN, gpioModePushPull, 0);
}

/****************************************************************************************************//**
*                                         LEDBuzz_BuzzerTurnOnLowFreq()
*
* @brief	Turn the buzzer on - Low frequency and low volume
********************************************************************************************************/
void LEDBuzz_BuzzerTurnOnLowFreq(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;

	(void)hal_AFE_Post(setup_Buzzer_2_wire_init, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	(void)hal_AFE_Post(setup_Buzzer_2_wire_on, &bist_result, &result, true);
    afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/****************************************************************************************************//**
*                                         LEDBuzz_BuzzerTurnOffLowFreq()
*
* @brief	Turn the buzzer off
********************************************************************************************************/
void LEDBuzz_BuzzerTurnOffLowFreq(void) {
	uint8_t bist_result;
	SmokeResult result;
    RTOS_ERR err;
    OS_MSG_SIZE size;

	(void)hal_AFE_Post(setup_Buzzer_2_wire_oFF, &bist_result, &result, true);
    AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	(void)hal_AFE_Post(setup_Buzzer_2_wire_deinit, &bist_result, &result, true);
    afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}




/****************************************************************************************************//**
*                                               LEDBuzz_AmbientLightStatusGet()
*
* @brief	Get the current status of the ambient light
*
* @return	bright, not dark (daytime)
* 			dark, not bright (night time)
*
* @note		(1) This is a dummy function
*  			(2) Move this function to the appropriate module
********************************************************************************************************/
static LEDBuzz_AmbientLightStatus LEDBuzz_AmbientLightStatusGet(void)
{
  uint8_t ambient_light_level_status = (uint8_t)ambient_light_get_status();
  LEDBuzz_AmbientLightStatus light_status;
  if(ambient_light_level_status == (uint8_t)AMBIENT_LEVEL_BRIGHTNESS)
  {
        light_status = bright;
  }
  else if (ambient_light_level_status == (uint8_t)AMBIENT_LEVEL_DARKNESS)
  {
        light_status = dark;
  }
  else
  {
        light_status = bright;  /* default to bright status in case of light sensor hw corrupt or error in read val*/
  }

  return light_status;
}

/****************************************************************************************************//**
*                                               LEDBuzz_AssistanceLEDSet()
*
* @brief	Turn Assistance LED On or Off
*
* @param	led_on true = turn on, false = turn off
********************************************************************************************************/
static void LEDBuzz_AssistanceLEDSet(bool led_on) {

	if (led_on == true) {
	    GPIO_TurnAssistanceLEDon();
	}
	else {
	    GPIO_TurnAssistanceLEDoff();
	}
}

/****************************************************************************************************//**
*                                               LEBBuzz_IsGAP()
*
* @brief	Check if there is gap
*
* @return	true = keep thread running, false = otherwise
********************************************************************************************************/
static bool LEBBuzz_IsGAP(const LEDBuzz_Sequence *active_sequence)
{
	bool ret = true; 
	/*Check if HB LED or Buzzer are on*/
	if((powerLEDOn == active_sequence->led) || ((buzzerOff != active_sequence->buzzerState) && (LEDBuzz_Silence == false)))
	{
		ret = false;
	}
	else 
	{
		/*Do nothing*/
	}
	return ret;
}

/****************************************************************************************************//**
*                                               LEDBuzz_ShutDown()
*
* @brief	Shutdown LED & Buzzer task
*
********************************************************************************************************/
void LEDBuzz_ShutDown(void)
{
	RTOS_ERR err;
	OSTaskSuspend(&LEDBuzz_TaskTCB, &err);
}

/****************************************************************************************************//**
*                                               LEDBuzz_checkForGap()
*
* @brief	Check if thread should be forced to keep running
*
* @return	true = HB LED or Buzzer are on, false = otherwise
********************************************************************************************************/
bool LEDBuzz_checkForGap(void)
{
	return LEDBuzz_GAP;
}

uint32_t LEDBuzz_GetHeartBeatPeriodInMS(void)
{
    return DEF_HEARTBEAT_PERIOD_LE_TIMER_ms;
}

void LEDBuzz_AcquireBuzzer(void)
{
    RTOS_ERR err;

    OSMutexPend( &buzzer_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err );
    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

void LEDBuzz_ReleaseBuzzer(void)
{
    RTOS_ERR err;

    OSMutexPost( &buzzer_mutex, OS_OPT_POST_NONE, &err );
    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

void LEDBuzz_PoweroffAssitLED(void)
{
  GPIO_TurnAssistanceLEDoff();
}

void LEDBuzz_SetFTMPulseTone(bool status)
{
  FTMTestPulseTone = status;
}

bool LEDBuzz_GetFTMPulseTone(void)
{
  return FTMTestPulseTone;
}

/****************************************************************************************************//**
*                                               Buzz_init()
*
* @brief  This initialise the buzz bist period in EEPROM
*
* @return n/a
********************************************************************************************************/
void Buzz_init(void)
{
  dl_buzzer_cfg_data_t buz;
  DataLogging_GetBuzzerConfig(&buz);
  if((buz.BuzzerThreshold == 0U) || (buz.BuzzerThreshold == 0xFFFFU))
  {
      /* The buzzer bist period 604800 is written to entire
      field of buzzer config */
      buz.BuzzerThreshold = 0x93A;
      buz.BuzzerBistPeriod = 0x80;
      DataLogging_SetBuzzerConfig(&buz);
  }
}

/****************************************************************************************************//**
*                                               setHeartBeatFlag
*
* @brief  This sets the heart flag status
*
* @return n/a
********************************************************************************************************/
void setHeartBeatFlag(bool status)
{
   heartBeatOn = status;
}

/****************************************************************************************************//**
*                                               setHeartBeatFlag
*
* @brief  This returns the heart flag status
*
* @return flag status
********************************************************************************************************/
bool getHeartBeatFlag(void)
{
  return heartBeatOn;
}
