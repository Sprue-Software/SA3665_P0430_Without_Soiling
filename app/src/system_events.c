/*********************************************************************************
 * @file  event_system.c
 * @brief Handle the behaviour of the alarm. Event handler.
 * @project SA3345 P200 Firmware
 * @date  29 Mar 2022
 * @author  NDI

 *******************************************************************************/

/* Includes required by this module */
#include "acquisition_Smoke.h"
#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/source/os_priv.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include "em_core.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "em_letimer.h"
#include "hal_BURTCTimer.h"
#include "events.h"
#include "app.h"
#include "system_events.h"
#include "diagnostics.h"
#include "acquisition_Co.h"
#include "acquisition_Heat.h"
#include "ambient_light.h"
#include "led_buzzer.h"
#include "sl_power_manager.h"
#include "spi_comms.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "telegram.h"
#include "assistance_light.h"
#include "hal_LETimer.h"
#include "hal_switches.h"
#include "production.h"
#include "P0200_FTM.h"
#include "timeHandler.h"
#include "hal_AFE.h"
/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#define AIRING_FLAG_BIT_POS       (2U)
#define ACT_DAVT_AIRING_STATE     (1U)
#define TOGGEL_AIRING             (2U)
#define RADIO_FLAG_BIT_POS        (4U)
#define ACT_DAVT_RADIO_PRI        (1U)
#define TOGGLE_RADIO_PRI          (2U)
#define LASER_MAX_RETRY           (2U)
#define USER_BIST_MAX_TRY         (3U)

#define EXTENDED_LASER_BIST_WAIT_PERIOD (10000u) /* extended BIST laser wait time 10sec */


/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
/* default System Mode & System Operational State & device is de-mounted*/
static behaviour_state_enum_System_modes behavioural_system_mode = Standby_Mode;
static behaviour_state_enum_operational_States behavioural_operational_State = state_Idle;
static Ads_state_t ADS_operateState = Ads_offBase; /*testing*/
static co_state_enum coState = co_none;
static Smoke_state_enum SmokeState = Smoke_none;
static heat_state_enum HeatState = Heat_none;
/* This variable is for alarm silence instances */

static Button_state_t switch_state = Button_Released;

static bool laserbistfail = false;
uint8_t laser_retry_count = 0U;
uint8_t  logbook_data_Co[8u] = {0};

static bool isSmokeMutePressed = false;
static bool isHeatMutePressed = false;
static bool isCoMutePressed = false;
static bool isRemoteAlarmStateActive = false;
static uint8_t remote_alarm_last = REM_ALM_MAX;

/* When high priority alarm overtakes when system is in low priority alarm mode*/
static bool isHeatAlarmNotServiced =  false;
static bool isCoAlarmNotServiced = false;

/* when system is in smoke alarm state and heat alarm trigger
 * then need to serve heat when smoke alarms end
*/
static bool isHeatAlarmtoServeAfterSmoke = false;
static bool isCoAlarmtoServeafterSmokeHeat = false;
static bool isFaultSilenceActive = false; 
static bool isStanbyModeCheckButton = false;

static bool isLaserExtndBistRunning = false;
/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
/*Function definitions for ADS Switch*/
static void handleDeviceEnable(void);
static void handleDeviceDisable(void);

void Stop_Diagnostic_BIST(void);
/*Function definitions for System Mode handling*/
static void handle_System_Standby_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_System_Commisioning_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_System_Operational_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_System_Transport_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_System_ShutDown_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1);
/*Function definitions for Operational System Mode different State handling*/
static void handle_State_idle(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_Smoke_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_Smoke_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_Heat_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_Heat_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_CO_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_CO_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1);
//static void handle_state_Remote_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_Domestic_Test(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_state_domestic_extended_test (OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_State_Airing_Configuration(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void handle_Radio_PriData_Configuration(OS_FLAGS flags_0, OS_FLAGS flags_1);
static void jumpToSmokeAlarm( bool newAlarm );
static void jumpToHeatAlarm( bool newAlarm, bool lowPriorityAlarm );
static void jumpToCOAlarm(bool newAlarm, bool lowPriorityAlarm, OS_FLAGS flags);
static void jumpToRemoteAlarm(void);
static void stopRemoteAlarmPattern(void);
static void log_system_mode_change( const behaviour_state_enum_System_modes system_mode );

/*****************************************************************************************/

/**
 * @brief This function handles control of the operate state machine
 * @details The operate state machine is used to control the behaviour of the
 * @param Event Flag 0: Critical System Events, flags_1 : In case we need more than 32 Flags
 * To Support More Than 32 Events flags_1 shall be used .More than 32 events Not tested
 * @return nothing to return.
 * @req PTR-1452,
 *@req PTR-1331 User Interface > BIST Features > ADS > Mounting
 *
 */
void runOperateModule(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;

	switch (ADS_operateState)
	{
	  case Ads_onBase:
		/*
		 * In active on-base/shipping tag removed state
		 * From here can either enter off-base state or shutdown(very low battery) state
		 */
		if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Device_disable_event_0)) != 0u)
		{
		    DEBUG_EVENTS("\nEnter DISABLED", false, 0u);
		    handleDeviceDisable();
		    ADS_operateState = Ads_offBase;
		}
		break;

	  case Ads_offBase:
		/*
		 * In Disable/Off-base/shipping tag inserted state
		 * From here can either enter on base state
		 */
		if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Device_enable_event_0)) != 0u)
		{
			DEBUG_EVENTS("Enter ACTIVE", false, 0u);
			ADS_operateState = Ads_onBase;/* Device has been placed on base so switch to operate_active State */
			handleDeviceEnable();
		}
		break;

	  default:
		/*Shouldn't reach here*/
	  break;
	}

	/* regardless of the device ADS state, battery shutdown event shall be handled */
	if ((flags_0 & FLAGS_BIT_INDEX(TMR_Device_Shutdown_event_0)) != 0u)
  {
		DEBUG_EVENTS("Enter SHUTDOWN", false, 0u);
  	behavioural_system_mode = Shutdown_Mode; /* change the system state to shudown mode */
    /* Log change to event logbook and send to MCU2 as required */
    log_system_mode_change( behavioural_system_mode );
	}
}

/**
 * @brief This function handles control of the behaviour state machine
 * @details The behavioural state machine is used to control all aspects of
 *  the device behaviour
 * @param Event Flag 0: Critical System Events, flags_1 : In case we need more than 32 Flags
 * To Support More Than 32 Events flags_1 shall be used .More than 32 events Not tested
 * @return nothing to return
 */
void runBehaviouralModule(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	switch (behavioural_system_mode)
	{
		case Standby_Mode:/* Standby_Mode:  The mode when we received a device from Factory*/
			handle_System_Standby_Mode(flags_0, flags_1);
			break;
		case Commisioning_Mode:/* Commisioning_Mode : The mode for when Mounted event received in standby mode*/
			handle_System_Commisioning_Mode(flags_0, flags_1);
			break;
		case Operational_Mode:/* Operational_Mode : The mode for when device is in operational mode*/
			handle_System_Operational_Mode(flags_0, flags_1);
			break;
		case Functional_Test_Mode: /*Functional_Test_Mode: The mode for validation testing*/
			handle_System_Operational_Mode(flags_0, flags_1);
			break;
		case Transport_Mode:/* Transport_Mode:  The mode for returned devices*/
			handle_System_Transport_Mode(flags_0, flags_1);
			break;
			/* No need to Have Shutdown mode: kept as backup*/
		case Shutdown_Mode:/*Shutdown_Mode   The mode for battery critically low and safe shutdown*/
			handle_System_ShutDown_Mode(flags_0, flags_1);
			break;
		default:
			/*Shouldn't reach here*/
			break;
	}
	/* End of Switch*/
}

/**
 * @brief Handle the Standby Mode & events
 *  @param Event Flag 0: Critical System Events, flags_1 : In case we need more than 32 Flags
 * To Support More Than 32 Events flags_1 shall be used .More than 32 events Not tested
 * @return none
 * @req PTR-1269 Device > Operation state > Standby mode Activities
 * @req PTR-1249 Device > Operation State > Standby mode Features
 */
static void handle_System_Standby_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
  (void) flags_1;

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
		DEBUG_EVENTS("\nDisplay Standby Mode\n", false, 0u);
		if (hal_switches_get_pattern() == Button_SingleShortPress)
		{ /* Get the Type of Button press */
		  SetStandByModeCheckButton(true);
			LEDBuzz_Post(PatternStandbyModeCheck); /*LED Post for Standby Mode */
		}
	}
	/* Mode Change By service tool : SPI handler function will set the mode & will set Mode change event*/
}

/**
 * @brief Handle the Transport Mode & events
 * @param flags_0 & flags_1
 * @return none
 * @req PTR-1266 Device > Operation state > Transport mode activities
 * @req PTR-1252 Device > Operation State > Transport mode (EOL)
 *
 */
static void handle_System_Transport_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
		DEBUG_EVENTS("\nDisplay Transport Mode\n", false, 0u);
		if (hal_switches_get_pattern() == Button_SingleShortPress)
		{ /* Get the Type of Button press */
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			LEDBuzz_Post(PatternTransportModeCheck); /*LED Post for transport Mode */
		}
	}
	/* Mode Change By service tool : SPI handler function will set the mode & will set Mode change event accondingly*/
}
/**
 * @brief Handle the Commissioning Mode & events
 * @param flags_0 & flags_1
 * @return none
 * @req PTR-1262 Device > Operation state > Commissioning mode duration
 * @req PTR-1263 Device > Operation State > Commissioning mode  supported activities
 */
static void handle_System_Commisioning_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;

	DEBUG_EVENTS("\nStuck in Commissioning mode if fails\n", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
	    DEBUG_EVENTS("\nCommissioing mode Button press", false, 0u);
	    if (hal_switches_get_pattern() == Button_SingleShortPress)
	    { /* Get the Type of Button press */
	      hal_switches_set_pattern(Button_Released); /* reset the pattern */
	      LEDBuzz_Post(PatternAlarmSilence); /*LED Post for alarm silence */
	      BURTCTimer_Start(TMR_State_Timeout_event_0, one_shot, COMMISSIONMODE_FAIL_SILENCE_PRERIOD);
	    }
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u)
	{
	    DEBUG_EVENTS("\nCommissioing mode Button Timeout", false, 0u);
	    LEDBuzz_Post(PatternCommissionFailSilenceTimeout);
	}

}
/**
 * @brief Handle the Operational Mode & events
 * @param flags_0 & flags_1
 * @return none
 * @req PTR-1464,
 *@req PTR-1245 Device > Operational Modes
 *@req PTR -1253 Device > Operation state > Operating mode features
 */
static void handle_System_Operational_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	switch (behavioural_operational_State)
	{
	  case state_Idle:/* Idle State: This is where the product will spend most of its life.*/
	    handle_State_idle(flags_0, flags_1);
		break;
	  case state_Smoke_Alarm:/* Smoke Alarm State:When device detect smoke */
	    handle_state_Smoke_Alarm(flags_0, flags_1);
		break;
	  case state_Smoke_Alarm_Silence:/* state_Smoke_Alarm_Silence: if device is not in super smoke & button press*/
	    handle_state_Smoke_Silence(flags_0, flags_1);
		break;
	  case state_Heat_Alarm: /*state_Heat_Alarm: When device detect Heat*/
	    handle_state_Heat_Alarm(flags_0, flags_1);
		break;
	  case state_Heat_Alarm_Silence:/* state_Heat_Alarm_Silence: if device is not in super Heat & button press*/
	    handle_state_Heat_Silence(flags_0, flags_1);
		break;
	  case state_CO_Alarm:/* state_CO_Alarm: when device detect Co**/
	    handle_state_CO_Alarm(flags_0, flags_1);
		break;
	  case state_CO_Alarm_Silence:/* CO_LocalAlarmSilence: if device is not in super Co & button press*/
	    handle_state_CO_Silence(flags_0, flags_1);
		break;
//	  case state_Remote_Alarm:/* state_Remote_Alarm: When MCU-2 request for Remote Alarm*/
//	    handle_state_Remote_Alarm(flags_0, flags_1);
//		break;
	  case State_Airing_Configuration:/* State_Airing_Configuration: To enable & disable airing configuration*/
	    handle_State_Airing_Configuration(flags_0, flags_1);
		break;
	  case State_Radio_PriData_Configuration:
	    handle_Radio_PriData_Configuration(flags_0, flags_1);
	  break;

	  default:
		/*Shouldn't reach here*/
		break;
	}
	/* End of Switch*/

	/*Here, add events that need to be handled regardless of the operational state
	 Fault Silence timeout should be handled in any operational state */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_faultSilence_event_0)) != 0u)
	{
		/* Re-enable fault buzzer Sound again */
		DEBUG_EVENTS("Fault Silent Timeout", false, 0u);
		isFaultSilenceActive = false;
		LEDBuzz_Post(PatternFaultSilenceTimeout);
		DataLogging_SetEventLogbookRecord( DEF_LBE_FAULT_MUTED_END, NULL ); /* Log fault mute end. PTR-873 */
	}
}

/**
 * @brief Handle the Shutdown Mode & events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1374, 
 */
static void handle_System_ShutDown_Mode(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	RTOS_ERR err;

	(void) flags_1;
	(void) flags_0;

	DEBUG_EVENTS("\nhandle_System_ShutDown_Mode", false, 0u);

	/* TODO: Log the status into eeprom */
	BURTCTimer_StopFrom(0); /* stop all the backup timer based events */
	BURTC_Enable(false); /* disable the BURTC timer */

	LETIMER_Enable(LETIMER0, false);

	/*Disable Watchdog*/
	WDOGn_Enable(WDOG0, false);
	/*stop all tasks*/
#if defined FTM_BUILD || defined DEBUG_BUILD
	OSTaskSuspend(&CommsTaskTCB, &err);
#endif
	OSTaskSuspend(&eventsTasktcb, &err);
	OSTaskSuspend(&wdogTimerTasktcb, &err);
	LEDBuzz_ShutDown();
	OSTaskSuspend(&InterMCUSPIComms_TaskTCB, &err);

	/*Go to sleep in lowest possible mode*/
	sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
	sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM3);
	sl_power_manager_sleep();
}

/**
 * @brief Function definitions for Operational System Mode different State handling
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1209, PTR-1464, PTR-1488, PTR-1465, PTR-1268, PTR-1166, PTR-1147, PTR-1389, PTR-1483, PTR-1430, PTR-1429, PTR-1385, PTR-2201, PTR-2202
 */
static void handle_State_idle(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;

	behaviour_state_enum_System_modes sys_mode = getBehavioural_System_Modes(false);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
	  /* check if device was last in radio alarm state*/
	  if(isRemoteAlarmStateActive == true)
	  {
	      DEBUG_EVENTS("\nRemote Smoke End", false, 0u);
	      isRemoteAlarmStateActive = false;
	      stopRemoteAlarmPattern();
	  }
		DEBUG_EVENTS("Smoke Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else if(smoke_detected() == true)
		{
			SmokeState = Smoke_high;
		}
		/* Smoke Alarm Post + Logging */
		jumpToSmokeAlarm(true);

	}
	else
  {
		/*LDRA */
	}

	if (((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u) /*|| (getHeatState() != Heat_none)*/)
	{
	    /* check if device was last in radio alarm state*/
	   if(isRemoteAlarmStateActive == true)
	   {
	       DEBUG_EVENTS("\nRemote Heat End", false, 0u);
         isRemoteAlarmStateActive = false;
         stopRemoteAlarmPattern();
	   }
		/*check the heat level set the variable Heat_super*/
		DEBUG_EVENTS("Heat Alarm", false, 0u);
		/*check the  Heat level and set the variables accordingly
		 *  HeatState=Heat_high or Heat_super
		 */
		if (getHeatAfterCompensation() >= SUPER_HEAT)
		{
			HeatState = Heat_super;
		}
		else
		{
			HeatState = Heat_high;
		}
		/* Heat Alarm Post + Logging */
		jumpToHeatAlarm(true, false);

	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
	{
	   /* check if device was last in radio alarm state*/
	   if(isRemoteAlarmStateActive == true)
	   {
	       DEBUG_EVENTS("\nRemote Co End", false, 0u);
         isRemoteAlarmStateActive = false;
         stopRemoteAlarmPattern();
	   }
		/*check the  Co level and set the variables accordingly
		  coState=co_high or co_super */
		if (get_CO_Level() >= SUPER_CO)
		{
			coState = co_super;
		}
		else
	  {
			coState = co_high;
		}

		DEBUG_EVENTS("Co Alarm", false, 0u);
		/* CO Alarm Post + Logging */
		jumpToCOAlarm(true, false, flags_0);
	}

	/* check low priority alarm still exits after exiting high priority alarm */
	if((isHeatAlarmNotServiced == true) && (getHeatState() != Heat_none))
	{
	    jumpToHeatAlarm(false, true);
	}
	else if((isHeatAlarmNotServiced == true) && (getHeatState() == Heat_none))
	{
	    SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
	    BURTCTimer_Stop(TMR_State_Timeout_event_0);
	    /* Stop Heat Alarm */
      if(GetAssistanceLightStatus() == true)
      {
          (void)BURTCTimer_Stop(Assistance_Light_event_1);
          GPIO_TurnAssistanceLEDoff();
      }
      LEDBuzz_Post(PatternAlarmHeatStop);
      isHeatAlarmNotServiced = false;
	}
	else if((isHeatAlarmtoServeAfterSmoke == true) && (getHeatState() != Heat_none))
	{
	    /* Enter as a new heat alarm*/
	    jumpToHeatAlarm(true, false);
	}
	else if((isCoAlarmNotServiced == true) && (get_CO_Level() != NO_CO))
	{
	    jumpToCOAlarm(false, true, flags_0);
	}
	else if((isCoAlarmNotServiced == true) && (get_CO_Level() == NO_CO))
	{
	    SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
      BURTCTimer_Stop(TMR_State_Timeout_event_0);
      if(GetAssistanceLightStatus() == true)
      {
            (void)BURTCTimer_Stop(Assistance_Light_event_1);
            GPIO_TurnAssistanceLEDoff();
      }
      /* Log The Alarm Stop*/
      LEDBuzz_Post(PatternAlarmCOStop); /*Stop alarm pattern */
      isCoAlarmNotServiced = false;
	}
	else if((isCoAlarmtoServeafterSmokeHeat == true) && (get_CO_Level() != NO_CO))
  {
	    /* Enter as a new co alarm*/
	    jumpToCOAlarm(true, false, flags_0);
	}
	else
	{
	   /* LDRA */
	}


	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_CO_IntelligentsampleRate_event_0)) != 0u)
	{
		/*check the heat level set the variable Smoke_super*/
		DEBUG_EVENTS("Co Increased Sample ", false, 0u);
		/* Call Only Co  acquisition */
		acquisitionCO(false);
	}

  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_RADIO_Alarm_event_0)) != 0u)
  {
    DEBUG_EVENTS("Remote Alarm", false, 0u);
    /* Remote Alarm Post + Logging */
    jumpToRemoteAlarm();
  }

	if (((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u) && (sys_mode != Functional_Test_Mode))
	{
		switch_state = hal_switches_get_pattern();
		if (hal_get_ads_state() == Ads_onBase)
		{
			if (switch_state == Button_SingleShortPress)
			{
				hal_switches_set_pattern(Button_Released); /* reset the pattern */
				DEBUG_EVENTS("\nShort single Button Press", false, 0u);

				if(isRemoteAlarmStateActive == true)
				{
				    DEBUG_EVENTS("\nButton Pressed", false, 0u);
				    isRemoteAlarmStateActive = false;
		            stopRemoteAlarmPattern();
		            set_rem_alarm_silence_to_mcu2(true);
		            SPIComms_Send_Data_to_MCU2( SPI_CMD_Alarm );    /* MCU2 wants to konw about button press */
				}
				else
				{
				    /* Perform Any Action on it */
				    uint32_t fault_val = FaultHandler_GetFaultFlags ();

					  /* New fault silent is requested */
				    if((fault_val != DEF_NO_FAULT) && (isFaultSilenceActive == false))
				    {
				        DEBUG_EVENTS("fault Silence requested", false, 0u);
				        /* Fault Silence */
				        LEDBuzz_Post(PatternAlarmSilence);
				        DataLogging_SetEventLogbookRecord( DEF_LBE_FAULT_MUTED_START, NULL ); /* Log fault mute start. PTR-873 */
				        BURTCTimer_Start(TMR_faultSilence_event_0, false, FAULT_SILENCE_TIMEOUT); /*Start silence timeout*/
				        isFaultSilenceActive = true;
				    }
				    else
				    {
				        DEBUG_EVENTS("fault Silencing req ignored", false, 0u);
				    }
				}
			}
			else if (switch_state == Button_LongPress)
			{
				hal_switches_set_pattern(Button_Released); /* reset the pattern */
				/* Perform user/BIST testing */
				/* Call the Function directly no need for State machine*/
				/* This will reduce response time */
				DEBUG_EVENTS("Perform user BIST", false, 0u);
				/* This will improve the button press & LED timings */
				handle_state_Domestic_Test(flags_0,flags_1);
				//setBehavioural_Operational_State(state_Domestic_Test);
			}
			else if (switch_state == Button_LongPressHold)
			{
				/* Perform extended user/BIST testing */
			  	DEBUG_EVENTS("Perform extended user BIST clear", false, 0u);
			  	hal_switches_set_pattern(Button_Released); /* reset the pattern */
				/* Call the Function directly no need for State machine*/
				/* This will improve the button press & LED timings */
			}
			else if (switch_state == Button_userExtTest)
			{
				if (isLaserExtndBistRunning == false)
				{
				   handle_state_domestic_extended_test(flags_0,flags_1);
				}
			}
			else if (switch_state == Button_userExtndTestLaser)
			{
			    if ((getBistResult() == true) && (isLaserExtndBistRunning == false))
			    {
			      isLaserExtndBistRunning = true;
			      DEBUG_EVENTS("Laser BIST 10sec timer start", false, 0u); /* Laser BIST wait timer */
			      LETimer_start(LETIMER_EXT_USER_LASER_TEST, EXTENDED_LASER_BIST_WAIT_PERIOD);
			    }
			}
			else
			{
				/* to avoid LDRA violation  */
			}
		}
		else if(hal_get_ads_state() == Ads_offBase)
		{
			if ((switch_state == Button_FiveShortPress) && (getDemountOneMinFlag() == true))
			{
				/* go back to standby mode from operating mode */
				DEBUG_EVENTS("go to standby mode", false, 0u);
				LEDBuzz_Post(PatternStandbyModeActivate); /* user indication with yellow LED: ON: 0.5sec */
				setBehavioural_System_Modes(Standby_Mode);
				(void)BURTCTimer_Stop(TMR_Demount_too_long_event_0);
				if((FaultHandler_GetFaultFlags() & DEF_DEM_TOO_LONG_FAULT) != 0U)
				{
				    DEBUG_EVENTS("Demount_Flag_Cleared", false, 0u);
				    FaultHandler_FaultClear(DemountedTooLongFault);
				    DataLogging_SetMinorFault(FaultDemountedTooLong, false);
				}
			}
			else
			{
				/*LDRA */
			  DEBUG_EVENTS("event button", false, 0u);
			}
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
		}
		else
		{
			/*LDRA */
		  DEBUG_EVENTS("event button else", false, 0u);
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
		}
		if ((switch_state == Button_DoubleShortPress) 
			&& (getBehavioural_System_Modes(false) == Operational_Mode))
		{
				hal_switches_set_pattern(Button_Released); /* reset the pattern */
				DEBUG_EVENTS(" Radio Private Data config mode activate", false, 0u);
				LEDBuzz_Post(PatternStopAll);
				setBehavioural_Operational_State(State_Radio_PriData_Configuration);			
				setRadioPriDataStatus(ACT_DAVT_RADIO_PRI);
				/* Stop All Diagnostic Timers*/
				Stop_Diagnostic_BIST();	
				set_AiringConfig((uint8_t)RADIOPRIV_DATA_CONFIGMODE);
				SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
				BURTCTimer_Start(TMR_State_Timeout_event_0, false, RADIOAIRING_CONFIGURATION_TIMEOUT); /*Start config timeout*/
		}
	}
}

/**
 * @brief This Function support Smoke Alarm events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1488, PTR-1465, PTR-1166
 */
static void handle_state_Smoke_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	DEBUG_EVENTS("handle_state_Smoke_Alarm", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("Smoke Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else
		{
			DEBUG_EVENTS("Do nothing in Smoke Alarm", false, 0u);
		}
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_NONE_event_0)) != 0u)
	{
		/*check the Smoke level set the variable Smoke_super*/
		DEBUG_EVENTS("Smoke LOW", false, 0u);
		/* Clear all events*/
		SmokeState = Smoke_none; /*Update local Smoke state*/
		/* Log The Alarm Stop*/
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		setBehavioural_Operational_State(state_Idle);
		if(GetAssistanceLightStatus() == true)
		{
		    (void)BURTCTimer_Stop(Assistance_Light_event_1);
		    GPIO_TurnAssistanceLEDoff();
		}
		LEDBuzz_Post(PatternAlarmSmokeStop); 
	}
	/* Any Button Press Event will consider As Silence in Alarm */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
		/*check the Smoke level & set the smoke variable again*/
		if (hal_switches_get_pattern() == Button_SingleShortPress)
		{
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			DEBUG_EVENTS("Smoke Alarm mute request", false, 0u);
			if (SmokeState != Smoke_super)
			{ /*Check if silence is permitted*/
				DEBUG_EVENTS("Enter Local Smoke Alarm Silence Mode", false, 0u);
				BURTCTimer_Start(TMR_State_Timeout_event_0, false,
				ALARM_SILENCE_TIMEOUT); /*Start silence timeout*/
				isSmokeMutePressed = true;
				SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
				setBehavioural_Operational_State(state_Smoke_Alarm_Silence);
				DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_START, NULL ); /* Log Alarm mute start. PTR-490 */
				/*log_advancedEvent(eventType_localAlarmSilenceActivated, NULL);Log event to EEPROM*/		//SK: Why Disabled
				LEDBuzz_Post(PatternAlarmSilence); /*Start silence pattern*/
			}
			else
			{
				DEBUG_EVENTS(" Alarm cannot be silenced", false, 0u);
			}
		}
	}

	/* serve heat alarm while system is in smoke alarm state */
  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
  {
      isHeatAlarmtoServeAfterSmoke = true;
  }
  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
  {
      isCoAlarmtoServeafterSmokeHeat = true;
  }

	/* Clear the Flags*/
	flags_0 = 0;
}

/**
 * @brief This Function support when Smoke Alarm is in silence and system events
 * @param flags_0 & flags_1
 * @return nothing to return
 */
static void handle_state_Smoke_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	RTOS_ERR err;

	DEBUG_EVENTS("handle_state_Smoke_Silence", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
			//P0200-6541 commented out
			//LEDBuzz_Post(PatternAlarmSmokeSilcenceTimeout);
			DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */
			//P0200-6541 if in remote alarm
			if(isRemoteAlarmStateActive == true)
			{
	        isRemoteAlarmStateActive = false;
	        stopRemoteAlarmPattern();
			    LEDBuzz_Post(PatternAlarmSmoke);
			}
			else
			{
			    LEDBuzz_Post(PatternAlarmSmokeSilcenceTimeout);
			}
			jumpToSmokeAlarm(false);/*Enter alarm state, but not as a new alarm*/
		}
		else
		{
			DEBUG_EVENTS(" SMK Silence Do nothing in Smoke Alarm", false, 0u);
		}
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_NONE_event_0)) != 0u)
	{
		/*check the Smoke level set the variable */
		DEBUG_EVENTS("Smoke LOW", false, 0u);
		/* Clear all events*/
		SmokeState = Smoke_none; /*Update local Smoke state*/
		if(isSmokeMutePressed == true)
	  {
		    DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */
		    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
		}
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		setBehavioural_Operational_State(state_Idle);
		BURTCTimer_Stop(TMR_State_Timeout_event_0);

		//P0200-6541 if no remote alarm, stop everything, otherwise do nothing and keep remote alarm pattern
		if(isRemoteAlarmStateActive == false)
		{
		    if(GetAssistanceLightStatus() == true)
		    {
		        (void)BURTCTimer_Stop(Assistance_Light_event_1);
		        GPIO_TurnAssistanceLEDoff();
		    }
		    /* Log The Alarm Stop*/
		    LEDBuzz_Post(PatternAlarmSmokeStop);
		    //clear the flag for NONE
		}


	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u)
	{
		/*Silence timeout so back to full alarm state*/
		DEBUG_EVENTS("Local Smoke Alarm Silence Timeout", false, 0u);
		//P0200-6541 commented out
		//LEDBuzz_Post(PatternAlarmSmokeSilcenceTimeout);
		DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */
		//P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
        isRemoteAlarmStateActive = false;
        stopRemoteAlarmPattern();
        LEDBuzz_Post(PatternAlarmSmoke);
    }
    else
    {
        LEDBuzz_Post(PatternAlarmSmokeSilcenceTimeout);
    }
		jumpToSmokeAlarm(false); /*Enter alarm state, but not as a new alarm*/
	}

	/* serve heat alarm while system is in smoke alarm state */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
	{
	      isHeatAlarmtoServeAfterSmoke = true;
	}
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
	{
	      isCoAlarmtoServeafterSmokeHeat = true;
	}

	//P0200-6541 interconnect issue fix
  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_RADIO_Alarm_event_0)) != 0u)
  {
    DEBUG_EVENTS("Remote Alarm", false, 0u);
    /* Remote Alarm Post + Logging */
    jumpToRemoteAlarm();
  }

}

/**
 * @brief This Function support Heat Alarm events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1488, PTR-1465, PTR-1166
 */
static void handle_state_Heat_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	DEBUG_EVENTS("handle_state_heat_alarm", false, 0u);
	/* If Smoke Event Occurs in Heat Alarm*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else if(smoke_detected() == true)
		{
			SmokeState = Smoke_high;
		}
		else
		{
			/* do nothing */
		}
		/* Smoke Alarm Post + Logging */
		jumpToSmokeAlarm(true);/*Enter alarm state, as a new alarm*/
	}

	/* If Super Heat is reported by Heat detection Module*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("Heat Alarm", false, 0u);
		/*check the  Heat level and set the variables accordingly*/
		if (getHeatAfterCompensation() >= SUPER_HEAT)
		{
			HeatState = Heat_super;
		}
		else
		{
			DEBUG_EVENTS("Do nothing in Heat Alarm", false, 0u);
		}
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_NONE_event_0)) != 0u)
	{
		DEBUG_EVENTS("Heat LOW", false, 0u);
		/* Clear all events*/
		HeatState = Heat_none; /*Update local Heat state*/
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		setBehavioural_Operational_State(state_Idle); /*Change state*/
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		/* Stop Heat Alarm */
	  if(GetAssistanceLightStatus() == true)
	  {
	      (void)BURTCTimer_Stop(Assistance_Light_event_1);
	      GPIO_TurnAssistanceLEDoff();
	  }
		LEDBuzz_Post(PatternAlarmHeatStop);
		isHeatAlarmNotServiced = false;
	}
	/* Any Button Press Event will consider As Silence */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
		if (hal_switches_get_pattern() == Button_SingleShortPress)
		{
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			if (HeatState != Heat_super)
			{ /*Check if silence is permitted*/
				DEBUG_EVENTS("\nLocal Heat Alarm Silence", false, 0u);
				BURTCTimer_Start(TMR_State_Timeout_event_0, false,
				ALARM_SILENCE_TIMEOUT); /*Start silence timeout*/
				isHeatMutePressed = true;
				SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);  //SK: Need to confirm the place to send ALR Silence
				setBehavioural_Operational_State(state_Heat_Alarm_Silence);
				DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_START, NULL ); /* Log Alarm mute start. PTR-490 */
				LEDBuzz_Post(PatternAlarmSilence);
			}
			else
			{
				DEBUG_EVENTS("\nSilence not allowed", false, 0u);
			}
		}
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
  {
	        isCoAlarmtoServeafterSmokeHeat = true;
	}
}

/**
 * @brief Function definitions for Heat Silence state & events
 * @param flags_0 & flags_1
 * @return nothing to return
 */
static void handle_state_Heat_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	RTOS_ERR err;

	DEBUG_EVENTS("\nhandle_state_Heat_Silence", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("\nSmoke Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else if(smoke_detected() == true)
		{
			SmokeState = Smoke_high;
		}
		else
		{
			/* do nothing */
		}

    //P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
        isRemoteAlarmStateActive = false;
        stopRemoteAlarmPattern();
    }
    else
    {
        LEDBuzz_Post(PatternAlarmHeatSilcenceTimeout);
    }

		/* Smoke Alarm Post + Logging */
		jumpToSmokeAlarm(true);/*Enter alarm state, as a new alarm*/
	}

	/* If Super Heat is reported by Heat detection Module*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("\nHeat Alarm", false, 0u);
		/*check the  Heat level and set the variables accordingly*/
		if (getHeatAfterCompensation() >= SUPER_HEAT)
		{
			HeatState = Heat_super;
			/* Stop Silence timeout & put the heat alarm buzzer to "Loud" mode PTR-1227 */
			BURTCTimer_Stop(TMR_State_Timeout_event_0);
			//LEDBuzz_Post(PatternAlarmHeatSilcenceTimeout);
			OSTimeDly(1, OS_OPT_TIME_DLY, &err);
			DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */

	    //P0200-6541 if in remote alarm
	    if(isRemoteAlarmStateActive == true)
	    {
	        isRemoteAlarmStateActive = false;
	        stopRemoteAlarmPattern();
	        LEDBuzz_Post(PatternAlarmHeat);
	    }
	    else
	    {
	        LEDBuzz_Post(PatternAlarmHeatSilcenceTimeout);
	    }

			jumpToHeatAlarm(false, false);
		}
		else
		{
			DEBUG_EVENTS("\nDo nothing in Heat Alarm", false, 0u);
		}
	}
	/* No Heat*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_NONE_event_0)) != 0u)
	{
		DEBUG_EVENTS("\nHeat LOW", false, 0u);
		/* Clear all events*/
		HeatState = Heat_none; /*Update local Smoke state*/
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		/* Log The Alarm Stop*/
		if(isHeatMutePressed == true)
		{
		    DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */
		    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
		}
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		setBehavioural_Operational_State(state_Idle);
    //P0200-6541 if no remote alarm, stop everything, otherwise do nothing and keep remote alarm pattern
    if(isRemoteAlarmStateActive == false)
    {
        if(GetAssistanceLightStatus() == true)
          {
            (void)BURTCTimer_Stop(Assistance_Light_event_1);
            GPIO_TurnAssistanceLEDoff();
          }
        LEDBuzz_Post(PatternAlarmHeatStop);
    }
		isHeatAlarmNotServiced = false;
	}
	/* Heat Silence Timeout*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u)
	{
		/*Silence timeout so back to full alarm state*/
		DEBUG_EVENTS("\nLocal Heat Alarm Silence Timeout", false, 0u);
		//LEDBuzz_Post(PatternAlarmHeatSilcenceTimeout);
		DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute end. PTR-490 */

    //P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
      isRemoteAlarmStateActive = false;
      stopRemoteAlarmPattern();
      LEDBuzz_Post(PatternAlarmHeat);
    }
    else
    {
      LEDBuzz_Post(PatternAlarmHeatSilcenceTimeout);
    }

		jumpToHeatAlarm(false, false); /*Enter alarm state, but not as a new alarm*/
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
	{
	    isCoAlarmtoServeafterSmokeHeat = true;
	}

	//P0200-6541 interconnect issue fix
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_RADIO_Alarm_event_0)) != 0u)
	{
	  DEBUG_EVENTS("Remote Alarm", false, 0u);
	  /* Remote Alarm Post + Logging */
	  jumpToRemoteAlarm();
	}

}

/**
 * @brief Function definitions for CO Alarms and events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1488, PTR-1465, PTR-1166
 */
static void handle_state_CO_Alarm(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	DEBUG_EVENTS("\nhandle_state_CO_Alarm", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("\nCO Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else if(smoke_detected() == true)
		{
			SmokeState = Smoke_high;
		}
		else
		{
			/* do nothing */
		}
		/* Smoke Alarm Post + Logging */
		jumpToSmokeAlarm(true);
	}

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
	{
		/*check the  Heat level and set the variables accordingly
		  HeatState=Heat_high or Heat_super */
		if (getHeatAfterCompensation() >= SUPER_HEAT)
		{
			HeatState = Heat_super;
		}
		else
	  {
			HeatState = Heat_high;
		}
		/* Heat Alarm Post + Logging */
		jumpToHeatAlarm(true, false); /*Enter alarm state, as a new alarm*/
	}

	logbook_data_Co[0]=(uint8_t)getCoAfterCompensation();
	/* Co detection*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
	{
		/*check the Co level set the variable */
		DEBUG_EVENTS("\nCO Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (get_CO_Level() >= SUPER_CO)
		{
			coState = co_super;
		}
	  else
	  {
			coState = co_high;
		}
	}
	/* No CO*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_NONE_event_0)) != 0u)
	{
		/*check the Co level set the variable */
		DEBUG_EVENTS("\nCO LOW", false, 0u);
		coState = co_none; /*Update local co state*/
		/* validate & acknowledge  if overload flag is set after alarm if co >500 PPM  */
		    /*Alarm on -> Alarm off  one event of High CO (i.e co >500) */
	    if (get_CO_Overload_Status() == true){
		      Ack_CO_Overload_events();
		}

		setBehavioural_Operational_State(state_Idle);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
	  if(GetAssistanceLightStatus() == true)
	  {
	        (void)BURTCTimer_Stop(Assistance_Light_event_1);
	        GPIO_TurnAssistanceLEDoff();
	  }
		/* Log The Alarm Stop*/
		LEDBuzz_Post(PatternAlarmCOStop); /*Stop alarm pattern */
		isCoAlarmNotServiced = false;
	}
	/* Co Silence */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{
		if (hal_switches_get_pattern() == Button_SingleShortPress)
		{
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			if (coState != co_super)
			{ /*Check if silence is permitted*/
			  RTOS_ERR err;
				DEBUG_EVENTS("\nEnter Local CO Alarm Silence Mode", false, 0u);
				BURTCTimer_Start(TMR_State_Timeout_event_0, false,
				ALARM_SILENCE_TIMEOUT); /*Start silence timeout*/
				isCoMutePressed = true;
				SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);  //SK: Need to confirm the place to send ALR Silence
				setBehavioural_Operational_State(state_CO_Alarm_Silence);
				OSTimeDly(1, OS_OPT_TIME_DLY, &err);
				DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_START, NULL ); /* Log Alarm mute start. PTR-490 */
				/*log_advancedEvent(eventType_localAlarmSilenceActivated, NULL);Log event to EEPROM*/
				LEDBuzz_Post(PatternAlarmSilence); /*Start silence pattern*/
			}
			else
			{
				DEBUG_EVENTS("\nSilence not allowed when in SUPER CO", false, 0u);
			}
		}
	}
}

/**
 * @brief Function definitions for Co silence & evnts
 * @param flags_0 & flags_1
 * @return nothing to return
 */
static void handle_state_CO_Silence(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	DEBUG_EVENTS("\nhandle_state_CO_Silence", false, 0u);
	(void) flags_1;
	RTOS_ERR err;

	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_HIGH_SUPER_event_0)) != 0u)
	{
		DEBUG_EVENTS("\nSmoke Alarm", false, 0u);
		/*check the  Smoke level and set the variables accordingly*/
		if (super_smoke_detected() == true)
		{
			SmokeState = Smoke_super;
		}
		else if(smoke_detected() == true)
		{
			SmokeState = Smoke_high;
		}
		else
	  {
			/* do nothing */
		}

    //P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
        isRemoteAlarmStateActive = false;
        stopRemoteAlarmPattern();
    }
    else
    {
        LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
    }

		/* Smoke Alarm Post + Logging */
		jumpToSmokeAlarm(true); /*Enter alarm state, as a new alarm*/
	}

	/*Heat detection */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_HIGH_SUPER_event_0)) != 0u)
	{
		/*check the  Heat level and set the variables accordingly
		 HeatState=Heat_high or Heat_supe */
		if (getHeatAfterCompensation() >= SUPER_HEAT)
		{
			HeatState = Heat_super;
		}
		else
		{
			HeatState = Heat_high;
		}

    //P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
        isRemoteAlarmStateActive = false;
        stopRemoteAlarmPattern();
    }
    else
    {
        LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
    }
		/* Heat Alarm Post + Logging */
		jumpToHeatAlarm(true, false); /*Enter alarm state, as a new alarm*/
	}
	/*CO detection */
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0)) != 0u)
	{
		if (get_CO_Level() >= SUPER_CO)
		{
			coState = co_super;
			BURTCTimer_Stop(TMR_State_Timeout_event_0);
			//LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
			DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL );
			OSTimeDly(1, OS_OPT_TIME_DLY, &err);

      //P0200-6541 if in remote alarm
      if(isRemoteAlarmStateActive == true)
      {
          isRemoteAlarmStateActive = false;
          stopRemoteAlarmPattern();
          LEDBuzz_Post(PatternAlarmCO);
      }
      else
      {
          LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
      }

			jumpToCOAlarm(false, false, flags_0);
		}
		else
	    {
			coState = co_high;
		}
	}
	else
	{
	    DEBUG_EVENTS("\nDo nothing in CO Alarm", false, 0u);
	}

	/* No CO*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_COHB_NONE_event_0)) != 0u)
	{
		/*check the Co level set the variable */
		DEBUG_EVENTS("\nCO LOW", false, 0u);
		/* Clear all events*/
		coState = co_none; /*Update local co state*/
	  /* validate & acknowledge  if overload flag is set after alarm if co >500 PPM  */
	        /*Alarm on -> Alarm off  one event of High CO (i.e co >500) */
	  if (get_CO_Overload_Status() == true)
	  {
          Ack_CO_Overload_events();
	  }

	  if(isCoMutePressed == true)
	  {
	      DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute start. PTR-490 */
	      OSTimeDly(1, OS_OPT_TIME_DLY, &err);
	  }
		setBehavioural_Operational_State(state_Idle);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		//P0200-6541 interconnect issue fix.
		//After carrying tests, see if need to add to check if in remote alarm. If not no code change otherwise do nothing
		//ie put the following statements in if statement checking remote alarm state

    //P0200-6541 if no remote alarm, stop everything, otherwise do nothing and keep remote alarm pattern
    if(isRemoteAlarmStateActive == false)
    {
        if(GetAssistanceLightStatus() == true)
          {
            (void)BURTCTimer_Stop(Assistance_Light_event_1);
            GPIO_TurnAssistanceLEDoff();
          }
        LEDBuzz_Post(PatternAlarmCOStop);
    }
	  isCoAlarmNotServiced = false;
	}
	/* Co silence Timeouts*/
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u)
	{
		/*Silence timeout so back to full alarm state*/
		DEBUG_EVENTS("\nLocal CO Alarm Silence Timeout", false, 0u);
		//P0200-6541 commented out
		//LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
		DataLogging_SetEventLogbookRecord( DEF_LBE_ALARM_MUTED_END, NULL ); /* Log Alarm mute start. PTR-490 */

    //P0200-6541 if in remote alarm
    if(isRemoteAlarmStateActive == true)
    {
      isRemoteAlarmStateActive = false;
      stopRemoteAlarmPattern();
      LEDBuzz_Post(PatternAlarmCO);
    }
    else
    {
      LEDBuzz_Post(PatternAlarmCOSilenceTimeout);
    }

		/* log_advancedEvent(eventType_localAlarmSilenceExit, NULL); Log to EEPROM*/
		jumpToCOAlarm(false, false, flags_0); /*Enter alarm state, but not as a new alarm*/
	}

	//P0200-6541 interconnect issue fix
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_RADIO_Alarm_event_0)) != 0u)
	{
	  DEBUG_EVENTS("Remote Alarm", false, 0u);
	  /* Remote Alarm Post + Logging */
	  jumpToRemoteAlarm();
	}

}

/**
 * @brief Function definitions for Domestic test & events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1464, PTR-1483
 */
static void handle_state_Domestic_Test(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_0;
	(void) flags_1;
	bool user_test_result = true;
	bool retVal = false;
	uint8_t counter = 0U;
	uint32_t fault_val = 0U;
	 RTOS_ERR err;

	DataLogging_LogUserTest(); /* Log Test In EEPROM */
	DEBUG_EVENTS("\nDomestic user test #", true, DataLogging_GetUserTestCount());

	const uint32_t userBistTime = get_currentTime();
	set_userBistTest_time(userBistTime);

	if(getIsDeviceSmokeEnable())
	{
	    while(counter < USER_BIST_MAX_TRY)
	    {
	        retVal = smoke_measurement_and_bist();
	        if(retVal || get_cham_minor_fault())
	        {
	            user_test_result = false;
	        }
	        else
	        {
	           user_test_result = true;
	           break;
	        }

	        counter++;
	    }
	}

	counter = 0U;
	while(counter <  USER_BIST_MAX_TRY)
	{
	    retVal = heat_measurement(true);
	    if(retVal == false)
	    {
	        user_test_result = false;
	    }
	    else
	    {
	        user_test_result = true;
	        break;
	    }

	    counter++;
	}

	DataLogging_SetEventLogbookRecord(DEF_LBE_USER_BIST, NULL);
	OSTimeDly(1, OS_OPT_TIME_DLY, &err);

	fault_val = FaultHandler_GetFaultFlags ();

	if((fault_val & DEF_MAJOR_FAULT) != 0U)
	{
	      user_test_result = false;
	}

	if((fault_val & DEF_MINOR_FAULT) != 0U)
	{
	    user_test_result = false;
	}

	if (user_test_result == true)
	{
		LEDBuzz_Post(PatternUserTestPass); /* user test pass status indicate to the user */
	}
	else
	{
	   if((fault_val & DEF_SMOKE_CHAMBER_HW_FAULT) != 0U)
	   {
	       /* Do nothing, smoke fault will be set implicitly */
	   }
	   else if((fault_val & DEF_HEAT_SENSOR_HW_FAULT) != 0U)
	   {
	       /* Do nothing, Heat fault will be set implicitly */
	   }
	   else
	   {
	       LEDBuzz_Post(PatternStopAll);
	       /* Restart Major and Minor fault*/
	       FaultHandler_Activate_Pattern();
	   }


	   if(isFaultSilenceActive == true)
	   {
	       DataLogging_SetEventLogbookRecord( DEF_LBE_FAULT_MUTED_END, NULL ); /* Log fault mute end. PTR-873 */
	       isFaultSilenceActive = false;
	       (void)BURTCTimer_Stop(TMR_faultSilence_event_0);
	   }
	}

	setBehavioural_Operational_State(state_Idle); /* go back to the idle state after finishing the test */
}


/**
 * @brief Function definitions for Domestic extended test & events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1431
 */
void handle_state_domestic_extended_test (OS_FLAGS flags_0, OS_FLAGS flags_1)
{
  (void)flags_0;
  (void)flags_1;
  bool user_test_result = false;
  DataLogging_LogUserExtTest(); /* Log extended Test In EEPROM */

  DEBUG_EVENTS("\nDomestic user extended test #", true, DataLogging_GetUserExtTestCount());

  const uint32_t userBistTime = get_currentTime();
  set_userBistTest_time(userBistTime);

  user_test_result = diagnostics(behavioural_system_mode, hal_get_ads_state(), 0);
  if (user_test_result == true)
  {
    DataLogging_SetEventLogbookRecord(DEF_LBE_USER_BIST, NULL);
    LEDBuzz_Post(PatternExtendedUserTestPass); /* user test pass status indicate to the user */
  }
  else
  {
    /* Do nothing, Minor or Major faults will be set implicitly*/
  }
  setBehavioural_Operational_State(state_Idle); /* go back to the idle state after finishing the test */
}


/**
 * @brief Function definitions for Domestic extended test & events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1431
 */
void handle_state_laser_extended_test(void)
{
  /* Call the laser BIST and take actions */
  DEBUG_EVENTS("Start Laser BIST", false, 0u);

  /* Disable the extended user BIST before starting the laser detection  */
  LEDBuzz_Post(PatternStopAll);
  GPIO_TurnAssistanceLEDoff();
  setEndExtUsrTest(false);
  hal_switches_set_pattern(Button_Released); /* reset the pattern */
  
  BURTCTimer_Start(Start_Laser_BIST_1, one_shot, EVENT_START_LASER_BIST_PERIOD);
  BURTCTimer_Start(Check_BIST_Results_1, one_shot, EVENT_CHECK_BIST_RESULT_PERIOD);
  setBehavioural_Operational_State(state_Idle); /* go back to the idle state after finishing the test */
}



/**
 * @brief Function definitions for Airing Configurations &events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req PTR-1209,
 */
static void handle_State_Airing_Configuration(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
	(void) flags_1;
	DEBUG_EVENTS("\nhandle_State_Airing_Configuration", false, 0u);
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
	{

		if (hal_switches_get_pattern() == Button_LongPress)
		{
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			DEBUG_EVENTS("\nToggle airing config in eeprom", false, 0u);
			 set_AiringConfig((uint8_t)AIRING_FLAG_TOGGLE);
			 BURTCTimer_Stop(TMR_State_Timeout_event_0);
			 BURTCTimer_Start(TMR_State_Timeout_event_0, false, RADIOAIRING_CONFIGURATION_TIMEOUT);
			 SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
		}
		else if (hal_switches_get_pattern() == Button_SingleShortPress)
		{
			hal_switches_set_pattern(Button_Released); /* reset the pattern */
			DEBUG_EVENTS("\nEnd of Airing Config ", false, 0u);
			set_AiringConfig((uint8_t)AIRING_FLAG_OFF);
			SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
			setBehavioural_Operational_State(state_Idle);
			FaultHandler_Activate_Pattern();
			Start_Diagnostic_BIST();
			BURTCTimer_Stop(TMR_State_Timeout_event_0);
		}
		else if (hal_switches_get_pattern() == Button_DoubleShortPress)
		{
	        hal_switches_set_pattern(Button_Released); /* reset the pattern */
	        DEBUG_EVENTS("\nHandle Radio Private Data Config ", false, 0u);
	        set_AiringConfig((uint8_t)RADIOPRIV_DATA_CONFIGMODE);
	        SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
	        setRadioPriDataStatus(ACT_DAVT_AIRING_STATE);
	        setBehavioural_Operational_State(State_Radio_PriData_Configuration);
	        BURTCTimer_Stop(TMR_State_Timeout_event_0);
	        BURTCTimer_Start(TMR_State_Timeout_event_0, false, RADIOAIRING_CONFIGURATION_TIMEOUT); 
		}
		else
	  {
			/* to avoid MISRA violations */
		}
	}
	if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u) {
		DEBUG_EVENTS("\nEnd of Airing Configuration", false, 0u);
		set_AiringConfig((uint8_t)AIRING_FLAG_OFF);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
		setBehavioural_Operational_State(state_Idle);
		FaultHandler_Activate_Pattern();
		Start_Diagnostic_BIST();
	}
}

/**
 * @brief Function definitions for Radio Private Configurations &events
 * @param flags_0 & flags_1
 * @return nothing to return
 * @req DCR119,
 */

static void handle_Radio_PriData_Configuration(OS_FLAGS flags_0, OS_FLAGS flags_1)
{
  (void) flags_1;
  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Button_Press_0)) != 0u)
  {
    if (hal_switches_get_pattern() == Button_LongPress)
    {
      hal_switches_set_pattern(Button_Released); /* reset the pattern */
      /*check the heat level set the variable Smoke_super*/
      DEBUG_EVENTS("\nToggle Radio config in eeprom", false, 0u);
      setRadioPriDataStatus(TOGGLE_RADIO_PRI);
      BURTCTimer_Stop(TMR_State_Timeout_event_0);
      BURTCTimer_Start(TMR_State_Timeout_event_0, false, RADIOAIRING_CONFIGURATION_TIMEOUT); /*Start silence timeout*/
    }
    else if (hal_switches_get_pattern() == Button_SingleShortPress)
    {
      hal_switches_set_pattern(Button_Released); /* reset the pattern */
      /*check the heat level set the variable Smoke_super*/
      DEBUG_EVENTS("\nEnd of Radio PriData Config ", false, 0u);
	  GPIO_TurnFaultLEDOff();
      LEDBuzz_Post(PatternRadioPvtDataInactiveStop);
      SPIComms_Send_Data_to_MCU2(SPI_CMD_Radio_Private_Data);
      setBehavioural_Operational_State(state_Idle);
	  FaultHandler_Activate_Pattern();
      Start_Diagnostic_BIST();
      BURTCTimer_Stop(TMR_State_Timeout_event_0);

    }
    else if (hal_switches_get_pattern() == Button_DoubleShortPress)
    {
        hal_switches_set_pattern(Button_Released); /* reset the pattern */
        /*check the heat level set the variable Smoke_super*/
        DEBUG_EVENTS("\nHandle state Airing Config ", false, 0u);
		GPIO_TurnFaultLEDOff();
        LEDBuzz_Post(PatternRadioPvtDataInactiveStop);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Radio_Private_Data);
        set_AiringConfig((uint8_t)AIRING_FLAG_ON);
        SPIComms_Send_Data_to_MCU2(SPI_CMD_Toggle_Config_Air);
        setBehavioural_Operational_State(State_Airing_Configuration);
        BURTCTimer_Stop(TMR_State_Timeout_event_0);
        BURTCTimer_Start(TMR_State_Timeout_event_0, false, RADIOAIRING_CONFIGURATION_TIMEOUT); /*Start silence timeout*/
    }
    else
    {
      /* to avoid MISRA violations */
    }
  }

  if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_State_Timeout_event_0)) != 0u)
  {
		GPIO_TurnFaultLEDOff();	
		LEDBuzz_Post(PatternRadioPvtDataInactiveStop);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Radio_Private_Data);
		DEBUG_EVENTS("\nEnd of Radio PriData Configuration", false, 0u);
		setBehavioural_Operational_State(state_Idle);
		FaultHandler_Activate_Pattern();
		Start_Diagnostic_BIST();
  }
}

/**
 * @brief Handle the common transition actions for entering the CO alarm state
 * @param newAlarm - true if function called when new alarm started ie not from silence to alarm state transition
 *  @param Flags :Event Flag 0
 */
static void jumpToCOAlarm(bool newAlarm, bool lowPriorityAlarm, OS_FLAGS flags)
{
	(void)(flags);
	BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
	GPIO_TurnAssistanceLEDon();
	isCoMutePressed = false;
	isCoAlarmNotServiced = true;
	isCoAlarmtoServeafterSmokeHeat = false;

	/* if low priority alarm resume from idle state, then send CMD to inform Tom Group*/
	if(lowPriorityAlarm == true)
	{
	    SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
	}

	if (newAlarm == true)
  {
		/*If newly triggered alarm reset variables*/
		SetIntelligentSampleRate(false);
		/*log_localAlarmEvent (alarmType_CO);*/
		DEBUG_EVENTS("\nEnter LocalCOAlarm state", false, 0u);
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		LEDBuzz_Post(PatternAlarmCO);
	}

	setBehavioural_Operational_State(state_CO_Alarm);
}

/**
 * @brief Handle the common transition actions for entering the Smoke alarm state
 * @param newAlarm - true if function called when new alarm started ie not from silence to alarm state transition
 * @param Flags :Event Flag 0
 * @req PTR-1147
 */
static void jumpToSmokeAlarm(bool newAlarm)
{

  BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
  GPIO_TurnAssistanceLEDon();
  isSmokeMutePressed = false;

	if (newAlarm == true)
	{
		/*If newly triggered alarm reset variables*/
		// Clear_the Increased sample Rate  ;
		DEBUG_EVENTS("Enter LocalSmokeAlarm state", false, 0u);
		/* Stop Any State_Timeouts*/
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		LEDBuzz_Post(PatternAlarmSmoke);
	}
	setBehavioural_Operational_State(state_Smoke_Alarm);
}

/**
 * @brief Handle the common transition actions for entering the Heat alarm state
 * @param newAlarm - true if function called when new alarm started ie not from silence to alarm state transition
 * @param Flags :Event Flag 0
 */
static void jumpToHeatAlarm(bool newAlarm, bool lowPriorityAlarm)
{

	 BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
	 GPIO_TurnAssistanceLEDon();
	 isHeatMutePressed = false;
	 isHeatAlarmNotServiced = true;
	 isHeatAlarmtoServeAfterSmoke = false;

	 /* if low priority alarm resume from idle state, then send CMD to inform Tom Group */
	 if(lowPriorityAlarm == true)
	 {
	     SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
	 }

	 if (newAlarm == true)
	 {
		/*If newly triggered alarm reset variables*/
		/*if required  Acknowledge or reset any Module variable*/
		DEBUG_EVENTS("Enter LocalHeatAlarm state", false, 0u);
		/* Stop Any State_Timeouts*/
		BURTCTimer_Stop(TMR_State_Timeout_event_0);
		SPIComms_Send_Data_to_MCU2(SPI_CMD_Alarm);
		LEDBuzz_Post(PatternAlarmHeat);
	}
	setBehavioural_Operational_State(state_Heat_Alarm);
}

/**
 * @brief Handle device disable
 * @details LED/Buzzer actions should be stopped and pins put into lowest possible current state.
 * The switch task should be set to only look at the ADS.
 * The event should be logged to the EEPROM
 * All  Diagnostic events should be , except timestamp and ambient light,
 *  @note This should be called at powerup to ensure all modules in correct state
 *  @req PTR-1452, PTR-1075
 * */

static void handleDeviceDisable(void)
{
  (void)BURTCTimer_Stop(Start_Laser_BIST_1);
  (void)BURTCTimer_Stop(Check_BIST_Results_1);
  set_laser_status_validated(false);
  laser_retry_count = 0U;
  /* Stop all timers except timestamp, variance, Heart Beat. */
  Stop_Diagnostic_BIST();

  /* Get production complete status */
  const uint32_t prod_comp = prod_get_value( );

	if (getBehavioural_System_Modes(false) == Standby_Mode)
	{

	}
	/* due to fault if device in commissioning mode , then de-mount will put it back to standby mode */
	else if (getBehavioural_System_Modes(false) == Commisioning_Mode)
	{
	    (void)BURTCTimer_Stop(TMR_State_Timeout_event_0);
	    setBehavioural_System_Modes(Standby_Mode);
	}
	else if (getBehavioural_System_Modes(false) == Operational_Mode)
	{
		setBehavioural_Operational_State(state_Idle);
		if(prod_comp == PROD_COMP_BB)
		{
       // Stay in operational mode
		}
		else
		{
		    setBehavioural_System_Modes(Standby_Mode);
		    //if prod comp == AA then Start the 120hrs timer
		    if(prod_comp == PROD_COMP_AA)
		    {
		        uint32_t prodTimer =  (uint32_t)get_prod_comp_timer();
		        if(prodTimer == 0U)
		        {
		            RTOS_ERR err;
		            DEBUG_EVENTS("\nDemount FTM Value is 0 Written BB", true, prodTimer);
		            prod_set_value(PROD_COMP_BB);
		            OSTimeDly(1, OS_OPT_TIME_DLY, &err);
		            Reset_EEPROM_Production();
		        }
		        else
		        {
		            //Convert time from hr to seconds
		            prodTimer = (prodTimer*3600U)/BURTC_PERIOD;
		            BURTCTimer_Start(TMR_Production_complete_1, one_shot, prodTimer);
		            DEBUG_EVENTS("\nDemount FTM Value is ", true, prodTimer);
		        }
		    }
		}
	}
	else if (getBehavioural_System_Modes(false) == Transport_Mode)
	{
		/* No Handling required only Stop any indications*/
	}
	/* This Need Special Handling if we dismount in Functional test then timeout should happen ?
	 * or  After switch testing switch back to previous mode*/
	else if (getBehavioural_System_Modes(false) == Functional_Test_Mode)
	{
		/* Switch back to  Functional test and after timeout back to operational mode */
		setBehavioural_System_Modes(get_previous_mode());
	}
	else
	{

	}

	/* Default to smoke init so when re-mount and if smoke is present, signals the alarms*/
	smoke_demount_init();
	/* Default the heat state to Heat_none so when re-mount and if heat present, signals the alarms */
	setHeatState(0);
	/* re-initialise low priority serviced alarm */
	isHeatAlarmNotServiced = false;
	isCoAlarmNotServiced = false;
	isHeatAlarmtoServeAfterSmoke = false;
	isCoAlarmtoServeafterSmokeHeat  = false;

	/* disable the extnd user BIST running status */
	isLaserExtndBistRunning = false; 

	co_demount_init();

	if(GetAssistanceLightStatus() == true)
	{
	    (void)BURTCTimer_Stop(Assistance_Light_event_1);
	    GPIO_TurnAssistanceLEDoff();
	}
	/* Stop LED/Buzz and put into ADS only low power mode */
	LEDBuzz_Post(PatternStopAll);

	/* reset the AFE registers on every demount */
	hal_AFE_POR();

}

/**
 * @brief   Starts laser bist test
 * @details this module send laser hw test command to MCU2
 * @parem n/a
 * @return n/a
 */
void startLaserBIST(void)
{
  DEBUG_EVENTS("\n Start laser Bist", false, 0u);
  Set_OC_Parameter(OC_BIST,0u,0u,0u,0u);
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Trig_Detection);
}

/**
 * @brief Checks Bist results
 * @details this module checks all HW bist result and if passes then system mode
 *          transition from commissioning to operating mode
 *
 * @parem n/a
 * @return n/a
 */
void checkBISTResults(void)
{

  LASER_TEST laserBistResult = END_OF_LASER_TEST;
  DEBUG_EVENTS("\nCheck BIST Result Enter", false, 0u);
	laserBistResult = get_laser_status();
	uint32_t fault_val = FaultHandler_GetFaultFlags ();


  if((getBistResult() == true) && (get_laser_status_validated() == true) &&
    (laserBistResult == LASER_NO_DETETCION) && (!(fault_val & DEF_MAJOR_FAULT)))
  {
      DEBUG_EVENTS("\nBIST Passed", false, 0u);

      if(laserbistfail == true)
      {
          FaultHandler_FaultClear(ObstacleDetectionHwFault);
          DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_HW_ERR_END, NULL );
          laserbistfail = false;
      }

      if(getBehavioural_System_Modes(false) == Operational_Mode)
      {
          FaultHandler_Activate_Pattern();
          LEDBuzz_Post(PatternFullSelfTestPassCommissioning);
      }
      else
      {
          SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
          LEDBuzz_Post(PatternFullSelfTestPassCommissioning);
          setBehavioural_System_Modes(Operational_Mode);
          setBehavioural_Operational_State(state_Idle);
      }

      Start_Diagnostic_BIST();
  }
  else
  {
      if((laserBistResult == LASER_SENSOR_FAILUARE) && (laser_retry_count < LASER_MAX_RETRY))
      {
          BURTCTimer_Start(Start_Laser_BIST_1, one_shot, EVENT_START_LASER_BIST_PERIOD);
          BURTCTimer_Start(Check_BIST_Results_1, one_shot, EVENT_CHECK_BIST_RESULT_PERIOD);
          laser_retry_count++;
      }
      else
      {
          /* laser HW error is previously not set */
          if (((fault_val & DEF_OBSTACLE_DET_HW_FAUT) == 0u) &&(laserBistResult == LASER_SENSOR_FAILUARE))
          {
              if(laserbistfail == false)
              {
                FaultHandler_FaultSet(ObstacleDetectionHwFault);
                DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_HW_ERR_START, NULL );
                laserbistfail = true;
              }
          }

          if(getBehavioural_System_Modes(false) == Operational_Mode)
          {
              setBehavioural_Operational_State(state_Idle);
              Start_Diagnostic_BIST();
              FaultHandler_Activate_Pattern();
          }
          else if(getBehavioural_System_Modes(false) == Commisioning_Mode)
          {
              /*Stop The Diagnostic Timers & Stay in commissioning mode */
              DEBUG_EVENTS("\nBIST failed", false, 0u);
              SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
              LEDBuzz_Post(PatternCommissioningFail);
          }
          else
          {

          }
      }
  }

  isLaserExtndBistRunning = false; 
}

/**
 * @brief Handle the device enable
 * @details Log the event.
 *  Resume the LED/BUZZ task
 * Start the diagnostics and timers depend on system modes
 *
 */
static void handleDeviceEnable(void)
{

	defaultVariableCO(); /* reset the default State */
	isRemoteAlarmStateActive = false;/* Set remote alarm state to false for init */

	/* Get production complete status */
	const uint32_t prod_comp_bb = prod_get_value( );
  //Stop 120hrs timer everytime mount happen
	(void)BURTCTimer_Stop(TMR_Production_complete_1);

	//LETimer_delay_ms(500u); /* delay 500ms */
	if (hal_get_ads_state() == Ads_onBase) /* read the ADS state */
	{
	    LEDBuzz_Post(PatternMounted); /* display the mounted pattern */
	}

	if (getBehavioural_System_Modes(false) == Standby_Mode)
	{
	    /* Charge the IRCAP before smoke BIST/measurement */
	    hal_IRCAP_Charge(20u);

		/* Run BIST only run when production is complete */
		if( prod_comp_bb == PROD_COMP_BB )
		{
		    DEBUG_EVENTS("\nProd BB completed - StandBy", false, 0u);
		    setBehavioural_System_Modes(Commisioning_Mode);
		    LEDBuzz_Post(PatternFullSelfTestRunning);
			  BURTCTimer_Start(Start_Laser_BIST_1, one_shot, EVENT_START_LASER_BIST_PERIOD);
			  BURTCTimer_Start(Check_BIST_Results_1, one_shot, EVENT_CHECK_BIST_RESULT_PERIOD);
			  (void)diagnostics(Standby_Mode, Ads_onBase, 0);
			  LEDBuzz_Post(PatternFullSelfTestRunningStop);
			  const uint32_t comTime = get_currentTime();
			  set_commissioning_time(comTime);

		}
		else
		{
		    DEBUG_EVENTS("\nProd BB NOT completed - StandBy", false, 0u);
		    setBehavioural_System_Modes(Operational_Mode);
		    setBehavioural_Operational_State(state_Idle);
		    FaultHandler_Activate_Pattern();
		    Start_Diagnostic_BIST();
		    // Giver user indication of Power LED on for 500ms and 5 beeps that system is now in operational mode
		    LEDBuzz_Post(PatternFullSelfTestPassCommissioning);
		}

	}
	else if (getBehavioural_System_Modes(false) == Operational_Mode)
	{
      /* Charge the IRCAP before smoke BIST/measurement */
      hal_IRCAP_Charge(20u);

	    if( prod_comp_bb == PROD_COMP_BB )
	    {
	        DEBUG_EVENTS("\nProd BB completed - Operation", false, 0u);
	        BURTCTimer_Start(Start_Laser_BIST_1, one_shot, EVENT_START_LASER_BIST_PERIOD);
	        BURTCTimer_Start(Check_BIST_Results_1, one_shot, EVENT_CHECK_BIST_RESULT_PERIOD);
	        (void)diagnostics(Operational_Mode, Ads_onBase, 0);
	    }
	    else
	    {
	        DEBUG_EVENTS("\nProd BB NOT completed - Operation", false, 0u);
	        FaultHandler_Activate_Pattern();
	        Start_Diagnostic_BIST();
	    }
	}
	else
	{

	}


	if (getBehavioural_System_Modes(false) == Transport_Mode)
	{
		/*Stop The Diagnostic  Timers */
		Stop_Diagnostic_BIST();
	}
}

/**
 * @brief get the behavioural Operational state
 * return  system state
 */
behaviour_state_enum_operational_States getBehavioural_Operational_State(void)
{
	return behavioural_operational_State;
}
/**
 * @brief Set  the behavioural Operational state
 * @param  New system state
 * @return none
 */
void setBehavioural_Operational_State(behaviour_state_enum_operational_States state)
{
	behavioural_operational_State = state;
}

/**
 * @brief get  the current behavioural state mode
 * @param  true= read from eeprom after power cycle
 * @return behavioural state mode
 */
behaviour_state_enum_System_modes getBehavioural_System_Modes( bool read_From_eeprom)
{
	behaviour_state_enum_System_modes system_mode;
	if (read_From_eeprom == true)
	{
		uint8_t op_state;
		/* read form EEROM */
		/* If the System Mode returned from EEPROM is invalid, then set the system mode to Standby_Mode */
		op_state = DataLogging_GetOperatingState();
		switch(op_state)
		{
			case (uint8_t)(Standby_Mode):
			{
				system_mode = Standby_Mode;
				break;
			}

			case (uint8_t)(Commisioning_Mode):
			{
				system_mode = Commisioning_Mode;
				break;
			}

			case (uint8_t)(Operational_Mode):
			{
				system_mode = Operational_Mode;
				break;
			}
	
			case (uint8_t)(Functional_Test_Mode):
			{
				system_mode = Functional_Test_Mode;
				break;
			}

			case (uint8_t)(Transport_Mode):
			{
				system_mode = Transport_Mode;
				break;
			}

			case (uint8_t)(Shutdown_Mode):
			{
				system_mode = Shutdown_Mode;
				break;
			}

			default:
			{
				system_mode = Standby_Mode;	
			}
		}
	}
	else 
	{
		system_mode = behavioural_system_mode;
	}
	return system_mode;
}

/**
 * @brief Stop All Timers except Timestamp
 * @param  none
 * @return none
 */
void Stop_All_timers_except_timestamp(void)
{
	/* Stop All Diagnostic Timers */
	Stop_Diagnostic_BIST();
	/* Stop general Timers */
	(void)BURTCTimer_Stop(TMR_Battery_Measurement_BIST_event_0); /* For Testing */
	(void)BURTCTimer_Stop(TMR_TempHum_measure_BIST_event_0);
	(void)BURTCTimer_Stop(FTM_Test_Btn_event_1);
	(void)BURTCTimer_Stop(FTM_Timeout_event_1);

}

/**
 * @brief Stop The Diagnostic BIST & Sensor Detection Timers
 * @param  none
 * @return none
 */
void Stop_Diagnostic_BIST(void)
{
	/* Stop All Diagnostic Timers */
  BURTCTimerEvent0_StopFrom(TMR_Smoke_measure_event_0);
  BURTCTimer_Start(TMR_heartbeat_event_0, periodical, HEARTBEAT_EVENT );
}
/**
 * @brief Start The Diagnostic BIST & Sensor Detection Timers
 * @param  None
 * @return none
 */
void Start_Diagnostic_BIST(void)
{
	uint32_t period; 
	/* Start All Diagnostic Timers */
	/* Check if device is configured with smoke detection activated PTR-846 */
	if (true == getIsDeviceSmokeEnable())
	{
		period = SMOKE_BIST_MEASUREMENT;  /*Default Value*/
		BURTCTimer_Start(TMR_Smoke_BIST_event_0, periodical, period);

		/* Smoke detection*/
		period = SMOKE_MEASUREMENT;		  /*Default Value*/
		BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, period);
	}
	else 
	{
		/*Device not configured with smoke detection deactivated*/
	}

	period = HEAT_MEASURMENT_BIST_PERIOD; /*Default Value*/
	BURTCTimer_Start(TMR_Heat_measure_BIST_event_0, periodical, period);

	period = CO_MEASUREMENT_PERIOD; /*Default Value*/
	BURTCTimer_Start(TMR_CO_measure_event_0, periodical, period);

    period = CO_BIST_PERIOD * 5 ; 

	BURTCTimer_Start(TMR_CO_BIST_event_0, periodical, period);

	period = BUZZER_BIST_PERIOD;  /* Default Value */
	BURTCTimer_Start(TMR_BUZZER_BIST_event_0, periodical, period);

	/* Obstacle/Laser  BIST*/
	BURTCTimer_Start(TMR_Obstacle_Coverage_BIST_event_0, periodical,
	OBSTACLE_COVARAGE_PERIOD); /* test*/

	/* Temp Humid */
	BURTCTimer_Start(TMR_TempHum_measure_BIST_event_0, periodical,
	TEMP_HUMIDITY_PERIOD);

	/* Stop off base Variance */
	BURTCTimer_Stop(TMR_CO_Variance_Acquisition_event_0);
}

static void log_system_mode_change( const behaviour_state_enum_System_modes system_mode )
{
  static behaviour_state_enum_System_modes system_mode_last = NUM_Modes;

  if( system_mode_last != system_mode )
  {
    switch( system_mode_last )
    {
	    case Standby_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_STANDBY_MODE_END, NULL );
        break;
      }
	    case Commisioning_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_COMM_MODE_END, NULL );
        break;
      }
	    case Operational_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_OPERATING_MODE_END, NULL );
        break;
      }
	    case Functional_Test_Mode:
      {
        break;
      }
	    case Transport_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_TRANSPORT_MODE_END, NULL );
        break;
      }
	    case Shutdown_Mode:
      {
        break;
      }
      default:
      {
        break;
      }
    }

    switch( system_mode )
    {
	    case Standby_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_STANDBY_MODE_START, NULL );
        break;
      }
	    case Commisioning_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_COMM_MODE_START, NULL );
        break;
      }
	    case Operational_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_OPERATING_MODE_START, NULL );
        break;
      }
	    case Functional_Test_Mode:
      {
        break;
      }
	    case Transport_Mode:
      {
        DataLogging_SetEventLogbookRecord( DEF_LBE_TRANSPORT_MODE_START, NULL );
        break;
      }
	    case Shutdown_Mode:
      {
        break;
      }
      default:
      {
        break;
      }
    }

    system_mode_last = system_mode;
  }
}

/**
 * @brief Set the mode of the behavioural state mode
 * @param  New system mode
 * @return none
 */
void setBehavioural_System_Modes( behaviour_state_enum_System_modes system_mode )
{
  RTOS_ERR err;

	DataLogging_SetOperatingState( ( uint8_t )( system_mode ) );

	behavioural_system_mode = system_mode;
 
  /* Log change to event logbook and send to MCU2 as required */
  log_system_mode_change( behavioural_system_mode );

	/* This will change the system mode by calling event system */
	/* This Function is not mandatory */
  OSFlagPost( &Event_Flags_SubGroup[ 0 ], EVENT_MODE_CHANGE_0, OS_OPT_POST_FLAG_SET, &err );
}


/**
 * @brief OPERATE_EVENTS
 * @param  none
 * @return  ADS events
 */

uint32_t OPERATE_EVENTS(void)
{
	uint32_t data_bytes_val = 0u;
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Device_enable_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Device_disable_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Device_Shutdown_event_0);
	return data_bytes_val;
}
/**
 * @brief DIAGNOSTIC_EVENTS
 * @param  none
 * @return  Diagnostics events
 */

uint32_t DIAGNOSTIC_EVENTS(void)
{
	uint32_t data_bytes_val = 0u;
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_CO_measure_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_CO_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Smoke_measure_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Heat_measure_BIST_event_0);
	//data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Soling_Measurement_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_TempHum_measure_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_Obstacle_Coverage_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_BUZZER_BIST_event_0);
	data_bytes_val |= (uint32_t) FLAGS_BIT_INDEX(TMR_heartbeat_event_0);
	return data_bytes_val;
}

/**
 * @brief Send Radio Private Data message to MCU2
 * @param  uint32:status
 * @return none
 */
void setRadioPriDataStatus(uint32_t status)
{
   uint32_t radioPriData_status = 0u;
   radioPriData_status = DataLogging_GetFeatureconfigurationFlags();
   switch(status)
   {
     case ACT_DAVT_RADIO_PRI:
       if ((radioPriData_status & (1u << RADIO_FLAG_BIT_POS)) != 0u)
       {

           DEBUG_EVENTS("\nPri Data Active ", false, 0u);
           LEDBuzz_Post(PatternRadioPvtDataInactiveStop);
           //LEDBuzz_Post(PatternRadioPvtDataActive);
           GPIO_TurnFaultLEDOn(); 
       }
       else
       {
           DEBUG_EVENTS("\nPri Data Inactive ", false, 0u);
		   GPIO_TurnFaultLEDOff(); 
           LEDBuzz_Post(PatternRadioPvtDataInactive);
       }
       break;
     case TOGGLE_RADIO_PRI:
       if ((radioPriData_status & (1u << RADIO_FLAG_BIT_POS)) != 0u)
       {
           DEBUG_EVENTS("\nToggle Pri Data Active ", false, 0u);
           radioPriData_status = (radioPriData_status & (~(1u << RADIO_FLAG_BIT_POS))); /* PrivateRadioData status off   */
		   GPIO_TurnFaultLEDOff();		   
           LEDBuzz_Post(PatternRadioPvtDataInactive);
       }
       else
       {
           DEBUG_EVENTS("\nToggle Pri Data Inactive ", false, 0u);
           radioPriData_status = (radioPriData_status | (1u << RADIO_FLAG_BIT_POS));   /* PrivateRadioData status on   */
           LEDBuzz_Post(PatternRadioPvtDataInactiveStop);
           //LEDBuzz_Post(PatternRadioPvtDataActive);
           GPIO_TurnFaultLEDOn();
       }
       DataLogging_SetFeatureConfigurationFlags(radioPriData_status);
       break;
     default:
       break;
   }
}

/**
 * @brief Handle the common transition actions for entering the Remote alarm state
 * Remote Alarm is only available when System is not in Local alarm
 * @param n/a
 * @return n/a
 */
static void jumpToRemoteAlarm(void)
{
  uint8_t remoteAlarm = 0U;
  RTOS_ERR err;

  //P0200-6541 interconnect issue fix
  if( (behavioural_operational_State == state_Idle) || (behavioural_operational_State == state_Smoke_Alarm_Silence) ||
      (behavioural_operational_State == state_Heat_Alarm_Silence) || (behavioural_operational_State == state_CO_Alarm_Silence) )
  {
      DEBUG_EVENTS("\nEnter Remote Alarm state", false, 0u);
      remoteAlarm = get_remote_alarm_status_MCU_2();
      if(remoteAlarm != remote_alarm_last)
        {
          //P0200-6541 interconnect issue fix
          if(remoteAlarm > REM_ALM_END)
          {
              switch(behavioural_operational_State)
              {
                case state_Smoke_Alarm_Silence:
                  DEBUG_EVENTS("\nSmoke Silenced State pattern stopped!", false, 0u);
                  LEDBuzz_Post(PatternAlarmSmokeStop);
                  break;
                case state_Heat_Alarm_Silence:
                  DEBUG_EVENTS("\Heat Silenced State pattern stopped!", false, 0u);
                  LEDBuzz_Post(PatternAlarmHeatStop);
                  break;
                case state_CO_Alarm_Silence:
                  DEBUG_EVENTS("\CO Silenced State pattern stopped!", false, 0u);
                  LEDBuzz_Post(PatternAlarmCOStop);
                  break;
                default:
                  /* Should not come here */
                  break;
              }
          }

          switch(remoteAlarm)
          {
            case REM_ALM_SMOKE:
              isRemoteAlarmStateActive = true;
              DEBUG_EVENTS("\nRemote Smoke", false, 0u);
              DataLogging_SetEventLogbookRecord(DEF_LBE_SMOKE_REMOTE_ALARM, NULL);
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              DataLogging_SetSmokeEvent(EVENT_TYPE_REMOTE);
              BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
              GPIO_TurnAssistanceLEDon();
              LEDBuzz_Post( PatternAlarmRemoteSmoke );
              break;
            case REM_ALM_HEAT:
              isRemoteAlarmStateActive = true;
              DEBUG_EVENTS("\nRemote Heat", false, 0u);
              DataLogging_SetEventLogbookRecord(DEF_LBE_HEAT_REMOTE_ALARM, NULL);
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              DataLogging_SetHeatEvent(EVENT_TYPE_REMOTE);
              BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
              GPIO_TurnAssistanceLEDon();
              LEDBuzz_Post( PatternAlarmRemoteHeat );
              break;
            case REM_ALM_CO:
              isRemoteAlarmStateActive = true;
              DEBUG_EVENTS("\nRemote Co", false, 0u);
              DataLogging_SetEventLogbookRecord(DEF_LBE_CO_REMOTE_ALARM, NULL);
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              DataLogging_SetCOEvent(EVENT_TYPE_REMOTE);
              BURTCTimer_Start(Assistance_Light_event_1, one_shot, ASSISTANCE_LIGHT_PERIOD); /* For Testing */
              GPIO_TurnAssistanceLEDon();
              LEDBuzz_Post( PatternAlarmRemoteCO );
              break;
            case REM_ALM_TEST:
              isRemoteAlarmStateActive = true;
              DEBUG_EVENTS("\nRemote Test Alarm", false, 0u);
              DataLogging_SetEventLogbookRecord(DEF_LBE_REMOTE_ALARM_TEST, NULL);
              LEDBuzz_Post( PatternDeviceGroupTestRunning );
              break;
            case REM_ALM_END:
              OSTimeDly(2, OS_OPT_TIME_DLY, &err);
              DEBUG_EVENTS("\nRemote Alarm End", false, 0u);
              isRemoteAlarmStateActive = false;
              DataLogging_SetEventLogbookRecord(DEF_LBE_REMOTE_ALARM_SILENCE, NULL);
              stopRemoteAlarmPattern();
              //P0200-6541 interconnect issue fix
              if(behavioural_operational_State > state_Idle)
              {
                  switch(behavioural_operational_State)
                  {
                    case state_Smoke_Alarm_Silence:
                      LEDBuzz_Post(PatternAlarmSmoke);
                      break;
                    case state_Heat_Alarm_Silence:
                      LEDBuzz_Post(PatternAlarmHeat);
                      break;
                    case state_CO_Alarm_Silence:
                      LEDBuzz_Post(PatternAlarmCO);
                      break;
                    default:
                      /* Should not come here */
                      break;
                  }
                  LEDBuzz_Post(PatternAlarmSilence);
              }
              break;
            default:
              /* Should not come here */
              break;
          }
        }

      remote_alarm_last = remoteAlarm; /* Save the last state so not to repeat */
  }

}


void stopRemoteAlarmPattern(void)
{
  LEDBuzz_Post(PatternAlarmRemoteSmokeStop);
  LEDBuzz_Post(PatternAlarmRemoteHeatStop);
  LEDBuzz_Post(PatternAlarmRemoteCOStop);
  LEDBuzz_Post(PatternDeviceGroupTestStop);
  (void)BURTCTimer_Stop(Assistance_Light_event_1);
  GPIO_TurnAssistanceLEDoff();
  remote_alarm_last = REM_ALM_MAX;
}

co_state_enum getCoState(void)
{
  return coState;
}

void SetStandByModeCheckButton(bool status)
{
  isStanbyModeCheckButton = status;
}

bool GetStandByModeCheckButton(void)
{
  return isStanbyModeCheckButton;
}

