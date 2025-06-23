
/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef LED_BUZZER_H_
#define LED_BUZZER_H_


/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "debug.h"
#include "stdbool.h"
#include "stdint.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef enum
{
	PatternIdle = 0,                                                              //0
	PatternGap,                                                                   //1
	PatternAlarmSmoke,                                                            //2
	PatternAlarmSmokeStop,//3
	PatternAlarmHeat,//4
	PatternAlarmHeatStop,//5
	PatternAlarmCO,//6
	PatternAlarmCOStop,//7
	PatternAlarmRemoteSmoke,//8
	PatternAlarmRemoteSmokeStop,//9
	PatternAlarmRemoteHeat,//10
	PatternAlarmRemoteHeatStop,//11
	PatternAlarmRemoteCO,//12
	PatternAlarmRemoteCOStop,//13
	PatternMajorFault,//14
	PatternMinorFault,//15
	PatternStopMajorFault,//16
	PatternStopMinorFault,//17
	PatternUserTestPass,//18
	PatternExtendedUserTestPass,//19
	PatternPhase2ExtendedUserTestPass,//20
	PatternExtendedUserTestPassStop,//21
	PatternFullSelfTestRunning,//22
	PatternFullSelfTestRunningStop,//23
	PatternFullSelfTestPassCommissioning,//24
	PatternFullSelfTestPassCommissioningStop,//25
	PatternDeviceGroupTestRunning,//26
	PatternDeviceGroupTestStop,//27
	PatternTransportActiveMounted,//28
	PatternTransportActiveMountedStop,//29
	PatternTransportModeCheck,//30
	PatternStandbyModeActivate,//31
	PatternStandbyModeCheck,//32
	PatternCommissioningFail,//33
	PatternCommissioningFailStop,//34
	PatternLowBatt,//35
	PatternLowBattStop,//36
	PatternRadioPvtDataActive,//37
	PatternRadioPvtDataActiveStop,//38
	PatternRadioPvtDataInactive,//39
	PatternRadioPvtDataInactiveStop,//40
	PatternTOMTestAlarm,//41
	PatternHeartbeat,//42
	PatternAlarmSilence,//43
	PatternStopAll,//44
	LEDBuzz_LETimerTimeout,//45
	LEDBuzz_BURTCTimerTimeout,//46
	PatternFaultSilenceTimeout,//47
	PatternAlarmSmokeSilcenceTimeout,//48
	PatternAlarmHeatSilcenceTimeout,//49
	PatternAlarmCOSilenceTimeout,//50
	PatternCommissionFailSilenceTimeout,//51
	PatternMounted, //52
	PatternNull, // 53 /*This is used to keep the LED & Buzzer thread running aimlessly*/
	TotalPatterns, // total 54 patterns
} LEDBUZZ_PATTERN_ID;

/********************************************************************************************************
*********************************************************************************************************
*                                               EXTERNS
*********************************************************************************************************
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

void LEDBuzz_Init(void);
void LEDBuzz_Post(LEDBUZZ_PATTERN_ID flag);
void LEDBuzz_ShutDown(void);
bool LEDBuzz_checkForGap(void);
uint32_t LEDBuzz_GetHeartBeatPeriodInMS(void);
void LEDBuzz_AcquireBuzzer(void);
void LEDBuzz_ReleaseBuzzer(void);
void LEDBuzz_PoweroffAssitLED(void);
void LEDBuzz_BuzzerTurnOnHighFreq(void);
void LEDBuzz_BuzzerTurnOffHighFreq(void);
void LEDBuzz_BuzzerTurnOnLowFreq(void);
void LEDBuzz_BuzzerTurnOffLowFreq(void);
void LEDBuzz_SetFTMPulseTone(bool status);
bool LEDBuzz_GetFTMPulseTone(void);
void Buzz_init(void);
void LEDBuzz_HeartbeatRequestProcess(void);
bool getHeartBeatFlag(void);
void setHeartBeatFlag(bool status);
/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* LED_BUZZER_H_ */
