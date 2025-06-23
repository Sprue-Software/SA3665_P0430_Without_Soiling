/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "acquisition_Smoke.h"
#include "acquisition_Co.h"
#include "acquisition_Heat.h"
#include <time.h>
#include "em_device.h"
#include "em_common.h"
#include "em_core.h"
#include "sl_sleeptimer.h"
#include "sl_power_manager.h"
#include "sl_atomic.h"
#include "board.h"
#include "os.h"
#include "app.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_wdog.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_chip.h"
#include "em_system.h"
#include "hal_BURTCTimer.h"
#include "hal_LETimer.h"
#include "hal_Timer0.h"
#include "hal_switches.h"
#include "timeHandler.h"
#include "system_events.h"
#include "comms_handler.h"
#include "events.h"
#include "diagnostics.h"
#include "hal_gpio.h"
#include "led_buzzer.h"
#include "spi_comms.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "switches_handler.h"
#include "crc.h"
#include "hal_AFE.h"
#include "battery_measurement.h"
#include "spi_driver.h"
#include "Variance.h"
#include "eeprom_handler.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "data_logging.h"
#include "ambient_light.h"
#include "ustimer.h"
#include "v3sctrl.h"
#include "P0200_FTM.h"
#include "uartCLI.h"
#include "assistance_light.h"
#include "user_info_page.h"
#include "system_events.h"
#include "ctune.h"
#include "production.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
//ABR Techem. Each of the following needs to be moved to relevant module.
OS_TCB eventsTasktcb;
OS_TCB wdogTimerTasktcb;
static OS_TCB uartCommsTasktcb;
static OS_TCB switchTasktcb;
static OS_TCB uartCLITasktcb;

static CPU_STK eventsTaskstack[EVENTS_TASK_STK_SIZE];
static CPU_STK wdogTimerTaskstack[WDOG_TIMER_TASK_STK_SIZE];
static CPU_STK uartCommsTaskstack[UART_COMMS_TASK_STK_SIZE];
static CPU_STK uartCLITaskstack[UART_CLI_TASK_STK_SIZE];
static CPU_STK switchTaskstack[SWITCH_TASK_STK_SIZE];
static CPU_STK AFETaskStack[AFE_TASK_STK_SIZE];

static AFERspMessage_t EventTaskResponse;

static uint32_t wdog_task_flags = 0;
static uint32_t wdogtimer_bit_mask = 0;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
static void app_bootup(void);
static void WDOG_init(void);

static void events_task(void *arg);
static void wdogTimer_task(void *arg);
/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
OS_FLAG_GRP Event_Flags_SubGroup[2];
OS_FLAGS flags_0;
OS_FLAGS flags_1;

void OSIdleEnterHook(void);
void OSIdleExitHook(void);

extern void AFE_Task(void *arg);

void timestamp_callback(sl_sleeptimer_timer_handle_t *handle,void *data);
static sl_sleeptimer_timer_handle_t timestamp_timer;

/**************************************************************************//**
 * @brief Watchdog initialization
 *****************************************************************************/
static void WDOG_init(void) {
	/* Enable clock for the WDOG module */
	CMU_ClockEnable(cmuClock_WDOG0, true);

	/* Watchdog Initialise settings */
	WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
	CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO as clock source */
	wdogInit.debugRun = false;
	wdogInit.em2Run = true;
	wdogInit.perSel = wdogPeriod_128k; // 8193 clock periods of 1KHz clock ~8 seconds timeout // // 2049 clock cycles of a 1kHz clock  ~2 seconds period

	/* Initialising watchdog with chosen settings*/
	WDOG_Init(&wdogInit);
}

void timestamp_callback(sl_sleeptimer_timer_handle_t *handle,void *data)
{
  (void)&handle;
  (void)&data;

  time_updateTimestamp();
}

/***************************************************************************//**
 * Initialise application.
 ******************************************************************************/
void app_init(void) {
	RTOS_ERR err;
	/*Use LPM 2*/
	sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
	SPIDriver_Init();
	uip_init( );          /* User page programming */
	//ct_init();
	Timer0_init();
	BURTC_init();
	CRC_Init();
	InterMCU_SPIComms_Init();
	LEDBuzz_Init();
	WDOG_init();
	hal_i2c_init(); /* i2c power pin configuration */
	hal_switches_init();
	LETimer_init();
	EFM_ASSERT(USTIMER_Init() == ECODE_OK);
	SetDBGRegPinStatus(2U, 0U); /*Disable JTAG test output pin*/
	SetDBGRegPinStatus(3U, 0U); /*Disable JTAG test input pin*/

#ifdef EEPROM_CALI
	DataLogging_CheckDataIntergrity( );
#endif

	/*   Check error code Need to change assert and trigger watchdog     */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSFlagCreate(&Event_Flags_SubGroup[0], /*   Pointer to user-allocated event flag.         */
	"EventSubGroup0Flags", /*   Name used for debugging.                  */
	0, /*   Initial flags, all cleared.                   */
	&err);
	/*   Check error code Need to change assert and trigger watchdog     */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSFlagCreate(&Event_Flags_SubGroup[1], /*   Pointer to user-allocated event flag.         */
	"EventSubGroup1Flags", /*   Name used for debugging.                  */
	0, /*   Initial flags, all cleared.                   */
	&err);
	/*   Check error code Need to change assert and trigger watchdog     */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSFlagCreate(&Event_Switches, /*   Pointer to user-allocated event flag.         */
	"EventSwitchesFlags", /*   Name used for debugging.                  */
	0, /*   Initial flags, all cleared.                   */
	&err);
	/*   Check error code Need to change assert and trigger watchdog     */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	/* Create Events Task for behavioural model & event system */
	/* This is starting point of the FW */
	OSTaskCreate(&eventsTasktcb, "Events Task", events_task,
	&eventsTasktcb,
	EVENTS_TASK_PRIO, &eventsTaskstack[0], (EVENTS_TASK_STK_SIZE / 10u),
	EVENTS_TASK_STK_SIZE, 40u, 0u,
	DEF_NULL, (OS_OPT_TASK_STK_CLR ), &err);
	EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/**
 * @brief Run the bootup procedures, initialising all hardware, RTOS tasks and variables
 */

static void app_bootup(void)
{
	const uint32_t reset_cause = RMU_ResetCauseGet();
	/*Init Co variables */
	defaultVariableCO();
	smoke_init(); /* smoke module initialisation */
	initHeat();
	initVariance(); /* init Variance */
	ambient_light_init(); //init ambient light threshold values
	battery_init();
	temp_humid_init();
	Buzz_init();
	DataLogging_SetLogMsgToMCU(true);  // Enable log msg send to mcu2
	prod_value_init();
	
	DataLogging_SetFWBuild();

	const bool eeprom_ok = data_logging_is_eeprom_ok( );

   /* Do not re-validate CRC if there is exisiting corruption */
	if( eeprom_ok )
	{
	   DataLogging_SetCRC( );
	}
	
	if(reset_cause & EMU_RSTCAUSE_POR)
	{
		DataLogging_SetResetReason(RstPOR);
	}

	if(reset_cause & EMU_RSTCAUSE_PIN)
	{
		DataLogging_SetResetReason(RstPIN);
	}

	if(reset_cause & EMU_RSTCAUSE_EM4)
	{
		DataLogging_SetResetReason(RstEM4);
	}

	if(reset_cause & EMU_RSTCAUSE_WDOG0)
	{
		DataLogging_SetResetReason(RstWDOG0);
	}

	if(reset_cause & EMU_RSTCAUSE_WDOG1)
	{
		DataLogging_SetResetReason(RstWDOG1);
	}

	if(reset_cause & EMU_RSTCAUSE_LOCKUP)
	{
		DataLogging_SetResetReason(RstLOCKUP);
	}

	if(reset_cause & EMU_RSTCAUSE_SYSREQ)
	{
		DataLogging_SetResetReason(RstSYSREQ);
	}

	if(reset_cause & EMU_RSTCAUSE_DVDDBOD)
	{
		DataLogging_SetResetReason(RstDVDDBOD);
	}

	if(reset_cause & EMU_RSTCAUSE_DVDDLEBOD)
	{
		DataLogging_SetResetReason(RstDVDDLEBOD);
	}

	if(reset_cause & EMU_RSTCAUSE_DECBOD)
	{
		DataLogging_SetResetReason(RstDECBOD);
	}

	if(reset_cause & EMU_RSTCAUSE_AVDDBOD)
	{
		DataLogging_SetResetReason(RstAVDDBOD);
	}

	if(reset_cause & EMU_RSTCAUSE_IOVDD0BOD)
	{
		DataLogging_SetResetReason(RstIOVDDBOD);
	}
	
	DEBUG_APP("\nReset reason:", true, reset_cause);
	
	DEBUG_APP("\nPIN Rst Cnt:", true, DataLogging_GetResetReasonCount(RstPIN));
	DEBUG_APP("\nPOR Rst Cnt:", true, DataLogging_GetResetReasonCount(RstPOR));
	DEBUG_APP("\nWDT Rst Cnt:", true, DataLogging_GetResetReasonCount(RstWDOG0));

	RMU_ResetCauseClear();

}

/**
 * @brief    The system task is responsible for starting all other tasks in the system, and then
 *           monitoring the state of the system to ensure all tasks are behaving as expected.
 * @param    p_arg   Pointer to an optional data area which can pass parameters to the task when the
 *                   task executes. Unused here.
 * @req PTR-1401,
 */
static void events_task(void *arg) {
	RTOS_ERR err;
    const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)arg;

     OSTaskRegSet(NULL, 0, ptrToMyTCB, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    
    bool result = hal_AFE_RegisterResponseVar(&EventTaskResponse);

#ifdef EEPROM_CALI
	dl_fw_rev_data_t fwRev;
#endif

#if defined FTM_BUILD || defined DEBUG_BUILD

	/* init Debug Message */
	commsAppInit(true);
#endif

#if defined FTM_BUILD || defined DEBUG_BUILD
    //SK: Release the uart Comms Task early as it is used by debug module to display debug messages
	OSTaskCreate(&uartCommsTasktcb, "UartComms Task", uartCommsTask,
				 &uartCommsTasktcb,
				 UART_COMMS_TASK_PRIO, &uartCommsTaskstack[0], (UART_COMMS_TASK_STK_SIZE / 10u),
				 UART_COMMS_TASK_STK_SIZE, 40u, 0u,
				 DEF_NULL, (OS_OPT_TASK_STK_CLR), &err);
#endif

	EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

	/*Initialise sensors*/
	app_bootup();

	OSTaskCreate(&AFETasktcb, "AFE Task", AFE_Task,
	DEF_NULL,
	AFE_TASK_PRIO, &AFETaskStack[0], (AFE_TASK_STK_SIZE / 10u),
	AFE_TASK_STK_SIZE, 40u, 0u,
	DEF_NULL, (OS_OPT_TASK_STK_CLR ), &err);


	//ABR. Create WDOG Timer Task.
	OSTaskCreate(&wdogTimerTasktcb, "WDOGTimer Task", wdogTimer_task,
	&wdogTimerTasktcb,
	WDOG_TIMER_TASK_PRIO, &wdogTimerTaskstack[0], (WDOG_TIMER_TASK_STK_SIZE / 10u),
	WDOG_TIMER_TASK_STK_SIZE, 0u, 0u,
	DEF_NULL, (OS_OPT_TASK_STK_CLR ), &err);

	/* ads and button task initialisation */
	OSTaskCreate(&switchTasktcb, "Switch Task", switches_handler_task, &switchTasktcb,
	SWITCH_TASK_PRIO, &switchTaskstack[0], (SWITCH_TASK_STK_SIZE / 10u),
	SWITCH_TASK_STK_SIZE, 0u, 0u,
	DEF_NULL, (OS_OPT_TASK_STK_CLR ), &err);
	EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


	/*Start-up: Start the three general timers */
	//BURTCTimer_Start(TMR_timestamp_event_0, periodical, TIMESTAMP_PERIOD);
	BURTCTimer_Start(WdogTimer_event_1, periodical, WDOGTIMER_EVENT_PERIOD_TEN_SEC);
  BURTCTimer_Start(SpiComms_event_1, periodical, SPI_COMMUMICATION_TX_RX);
	BURTCTimer_Start(TMR_AmbientLight_measure_event_0, periodical, AMBIENT_MEASUREMENT_PERIOD);
	BURTCTimer_Start(TMR_heartbeat_event_0, periodical, HEARTBEAT_EVENT );
	BURTCTimer_Start(TMR_Disable_DBG_Port_1, one_shot, DBG_PORT_EVENT );
	if (getBehavioural_System_Modes(false) != Functional_Test_Mode)
	{
	    BURTCTimer_Start(TMR_Battery_Measurement_BIST_event_0, periodical, BATTERY_MEAS_BIST_PERIOD); /* Default battery periodicity is 24hours */
	    BURTCTimer_Start(TMR_CO_Variance_Acquisition_event_0, periodical, VARIANCE_PERIOD_NON_OPERATIONAL);
	}
	/* Send event counter information to MCU2 on a daily basis */
	BURTCTimer_Start( TMR_EVENT_COUNTERS_SPI_SEND_1, periodical, EVENT_COUNTERS_PERIOD );

	//TDI and TDO pins 0011
	uint8_t dbgStatus = GetDBGRegPinStatus(3U);
	DEBUG_APP("\nDBG TDI Pin Status ", true, dbgStatus);
	dbgStatus = GetDBGRegPinStatus(2U);
	DEBUG_APP("\nDBG TDO Pin Status ", true, dbgStatus);

	DEBUG_APP("\nFwRevMajor ", true, FW_MAJOR_REV);
	DEBUG_APP("\nFwRevMinor ", true, FW_MINOR_REV);
	DEBUG_APP("\nFwRevBuild ", true, FW_BUILD_REV);

	SPIComms_Send_Data_to_MCU2(SPI_CMD_Firmware_version);



#ifdef DEBUG_BUILD
	OSTaskCreate(&uartCLITasktcb, "UART CLI Task", uartCLITask,
				 &uartCLITasktcb,
				 UART_CLI_TASK_PRIO, &uartCLITaskstack[0], (UART_CLI_TASK_STK_SIZE / 10u),
				 UART_CLI_TASK_STK_SIZE, 0u, 0u,
				 DEF_NULL, (OS_OPT_TASK_STK_CLR), &err);
	EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
#endif

	/* Show ctuning */
  uint32_t ctune;

  const bool ok = ct_get_tuning_value( &ctune );

  if( ok )
  {
		DEBUG_APP( "\nctune:", true, ctune );
  }

	/*Init the time handler*/
	time_initTime();

	/* Initilaise demounting logbook */
	data_logging_logbook_init( );

	if(Get_Reset_Cause_MCU() == true)
	{
	    DEBUG_APP("\nReset-Cause ", false, 0u);
	    DataLogging_SetEventLogbookRecord(DEF_LBE_RESET, NULL ); /* PTR-1288 */
	}

  sl_sleeptimer_start_periodic_timer_ms(&timestamp_timer, 10000u, timestamp_callback,
                                         NULL, 0u, SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);

  OSTimeDly(200, OS_OPT_TIME_DLY, &err);

  /* Start 120hrs timer if unintentional reset happens */
   const uint32_t isCompleteProductonSet = prod_get_value();

   if( isCompleteProductonSet == 0xFFFFFFFFU)
   {

   }
   else if (isCompleteProductonSet == PROD_COMP_BB)
   {
      const uint8_t dev_config = DataLogging_GetDeviceConfig();

      if( dev_config == DEVICE_CONFIG_SMOKE )
      {
          setIsDeviceSmokeEnable(true);
      }
      else
      {
          setIsDeviceSmokeEnable(false);
      }

   }
   else if(isCompleteProductonSet == PROD_COMP_AA)
   {
      if(hal_get_ads_state() == Ads_offBase)
      {
          uint32_t prodCommenceTime = dl_get_prod_commence_time();
          uint32_t CurrTime = get_currentTime();
          uint32_t timeElapsedInHrs = 0U;
          const uint32_t prodTimer = dl_get_production_timer();

          if(prodCommenceTime != 0xFFFFFFFFU)
          {
              if(CurrTime >= prodCommenceTime)
              {
                  timeElapsedInHrs = (CurrTime - prodCommenceTime)/3600U;
              }

              if(timeElapsedInHrs == 0U)
              {
                  BURTCTimer_Start(TMR_Production_complete_1, one_shot, ((prodTimer*3600U)/BURTC_PERIOD));
              }
              else
              {
                  BURTCTimer_Start(TMR_Production_complete_1, one_shot, (((prodTimer-timeElapsedInHrs)*3600U)/BURTC_PERIOD));
              }

          }
      }
   }
   else
   {

   }

	/* This  is blocking function for Event Task, This function will unblock once Event received */
	while (true) {
		//ABR. Will add TopGroup checking later.
		flags_0 = OSFlagPend(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
		0xffffffffu, /* Flag bitmask to match. */
		0, /* Wait indefinitely. */
		OS_OPT_PEND_FLAG_SET_ANY | /* Wait until ANY flags are set and */
		OS_OPT_PEND_BLOCKING | /* task will block and */
		OS_OPT_PEND_FLAG_CONSUME, /* consume flags */
		DEF_NULL, /* Timestamp is not used. */
		&err);
		/* Check error code. */
		if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)
		{
			/* every 1 minute */
//			if ((flags_0 & FLAGS_BIT_INDEX(TMR_timestamp_event_0)) != 0u)
//			{
//			    time_handleTimestamp();
//			    DEBUG_APP("\nTime Stamp ", true, get_currentTime());
//			}
      /* every 6 minute */
      if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_AmbientLight_measure_event_0)) != 0u)
      {
          DEBUG_APP("\nAmbient Light ", false, 0u);
          /* Suggested steps */
          /* Step:1 Read the Ambient level from sensor and set the variable
           * Step:2 Night detected: which will be useful for all alarms
           */
          ambient_light_measure(AMBIENT_LIGHT_SETLING_TIME, AMBIENT_LIGHT_SAMPLE_COUNT);
      }
			/* ADS switch Conditions*/
			if ((flags_0 & (uint32_t) OPERATE_EVENTS()) != 0u)
			{
			    /*Evaluate the operational state*/
			    runOperateModule(flags_0, flags_1);
			}

			if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_MODE_Change_event_0)) != 0u)
			{
			      //setBehavioural_System_Modes(Transport_Mode);
			      /* This Function will be call if the System Need changes & event Task Need Calling + Run Behavioural
				   SPI-MCU-2 App layer will check the mode and set the Event
			     * (Supported Mode)
				   * standby <-> Operational , Standby <-> Transport, Operational > Transport
				   * FTM mode is allowed from Operational, Transport, Commissioning mode only
				   * Record the Time Stamp with who requested mode change
				   record_System_Mode_with_Time_Stamp(getBehavioural_System_Modes());*/
			    SPIComms_Send_Data_to_MCU2(SPI_CMD_Operating_modes);
			    SPIComms_Send_Data_to_MCU2(SPI_CMD_Logbook_value);
			}

			if (((flags_0 & (uint32_t) DIAGNOSTIC_EVENTS())
					|| (flags_0 & FLAGS_BIT_INDEX(TMR_Battery_Measurement_BIST_event_0))) != 0u)
			{
			    /* Suggested steps */
			    /* Step:1 check the ADS switch Status ? Mounted or demounted (i.e operate_active, operate_disabled);
			     * Step 3 System mode =getBehavioural_System_Modes ();
			     *diagnostics(System mode ,Mounted,flags_0);*/
			    /* Flag_0 will be use to decide which mode need to handle */
			    diagnostics(getBehavioural_System_Modes(false), hal_get_ads_state(), flags_0);
			}

			if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_CO_Variance_Acquisition_event_0)) != 0u)
			{
			    if(getBehavioural_System_Modes(false) != Functional_Test_Mode)
			    {
			        DEBUG_APP("\n off base variance  ", false, false);
			        /* Suggested steps */
			        /* below function will be use to get off Base variance (ADS- OFF & Diagnostic is stoped)
				     offbase_variance();*/
			        acquisitionCO(false);
			        DoVariance();
          }
			}

			if((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_Demount_too_long_event_0)) != 0u)
			{
			    DEBUG_APP("\n Demount Too Long Fault Set  ", false, false);
			    FaultHandler_FaultSet(DemountedTooLongFault);
			    DataLogging_SetEventLogbookRecord(DEF_LBE_DEMOUNTED_TOO_LONG_START, NULL);
			    setDemountTooLongFlag(true);
			    DataLogging_SetMinorFault(FaultDemountedTooLong, true);
			}
			/* Pass the events to the State machine */
			runBehaviouralModule(flags_0, flags_1);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

static void wdogTimer_task(void *arg)
{
	RTOS_ERR err;
	OS_FLAGS flags;
  const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)arg;
	wdogtimer_bit_mask = (1u << (uint32_t) WDOG_TASK_FLAGS_SIZE) - 1u;
    
  OSTaskRegSet(DEF_NULL, 0, ptrToMyTCB, &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	while (true)
	{
		//ABR. Will be changed to pend the corresponding semaphore or mutex as it is not periodical.
		flags = OSFlagPend(&Event_Flags_SubGroup[1], 0xffffffffu, 0,
		OS_OPT_PEND_FLAG_SET_ANY |
		OS_OPT_PEND_BLOCKING |
		OS_OPT_PEND_FLAG_CONSUME,
		DEF_NULL, &err);
		// Check error code.
		if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)
		{
			if ((flags & FLAGS_BIT_INDEX(WdogTimer_event_1)) != 0u)
			{
				//ABR Next 5 statement to be removed. Here to prevent the WDOG reset
				//Need to be placed where needed to monitor functions or tasks being executed
				/////////////////////////////////////////////////////////////////////////////
				wdogTimer_flagSet(WDOG_task0_flag);
				wdogTimer_flagSet(WDOG_task1_flag);
				wdogTimer_flagSet(WDOG_task2_flag);
				wdogTimer_flagSet(WDOG_task3_flag);
				wdogTimer_flagSet(WDOG_task4_flag);
				//////////////////////////////////////////////////////////////////////////////
				//ABR. Add functionality code
				if ((wdog_task_flags & wdogtimer_bit_mask) == wdogtimer_bit_mask)
				{
					WDOG_Feed();
					wdog_task_flags = 0;
				}
			}

			if ((flags & FLAGS_BIT_INDEX(LedBuzz_event_1)) != 0u)
		  {
				LEDBuzz_Post(LEDBuzz_BURTCTimerTimeout);
			}

			if ((flags & FLAGS_BIT_INDEX(SpiComms_event_1)) != 0u)
			{

			    SPIComms_Send_Data_to_MCU2(SPI_CMD_Current_value);
			}

	    if ((flags & FLAGS_BIT_INDEX(CO_OVERLOAD_event_1)) != 0u)
	    {
	        if(getBehavioural_System_Modes(false) != Functional_Test_Mode)
	        {
	           SetOverLoadFlag();
	        }
	    }

	    if ((flags & FLAGS_BIT_INDEX(FTM_Test_Btn_event_1)) != 0u)
	    {
	          set_ftm_btn_event(true);
	    }
#ifdef FTM_BUILD
	    if ((flags & FLAGS_BIT_INDEX(FTM_Timeout_event_1)) != 0u)
	    {
	        exitFTMProtocol();
	    }
#endif
	    if((flags & FLAGS_BIT_INDEX(Assistance_Light_event_1)) != 0u)
	    {
	        GPIO_TurnAssistanceLEDoff();
	        DataLogging_SetAssiatnceLightPeriod(GetAssistancelogPeriod());
	        const bool eeprom_fine = data_logging_is_eeprom_ok( );
	        if( eeprom_fine )
	        {
	            DataLogging_SetCRC( );
	        }

	    }

			if((flags & FLAGS_BIT_INDEX( TMR_EVENT_COUNTERS_SPI_SEND_1)) != 0u )
			{
			    SPIComms_Send_Data_to_MCU2( SPI_CMD_Countersdates );
			}

			if ((flags & FLAGS_BIT_INDEX(Start_Laser_BIST_1)) != 0u)
			{
			    startLaserBIST();
			}

			if ((flags & FLAGS_BIT_INDEX(Check_BIST_Results_1)) != 0u)
			{
			    checkBISTResults();
			}

			/* Have we been in darkness for 7 days? */
		  if( ( flags & FLAGS_BIT_INDEX( TMR_AmbientLight_7days_darkness_1 ) ) != 0u )
			{
				/* Yes, create logbook event */
		    set_SevenDays_Darkness_Status(true);
		    OSTimeDly(1, OS_OPT_TIME_DLY, &err);
        DataLogging_SetEventLogbookRecord( DEF_LBE_AMB_LIGHT_7_DAYS_DARK, NULL );
        if((FaultHandler_GetFaultFlags() & DEF_MINOR_FAULT) != 0u)
        {
            DEBUG_APP("\n7 days darkness ended", false, 0u);
            LEDBuzz_Post(PatternMinorFault);
        }
			}

		  /* Have demount been one min long ? */
		  if ((flags & FLAGS_BIT_INDEX(TMR_Demount_One_min_period_event_1)) != 0u)
		  {
		      setDemountOneMinFlag(true);
		  }

		  if((flags & FLAGS_BIT_INDEX(TMR_Production_complete_1)) != 0u)
		  {
		      DEBUG_APP("\nTimer Expires written BB ", false, 0u);
		      prod_set_value(PROD_COMP_BB);
		      OSTimeDly(1, OS_OPT_TIME_DLY, &err);
		      Reset_EEPROM_Production();
		  }

		  if((flags & FLAGS_BIT_INDEX(TMR_FAILED_SPI_COMMS_TIMEOUT_1)) != 0u)
		  {
		        SPIComms_HandleLongTimeout();
		  }

		  if((flags & FLAGS_BIT_INDEX(TMR_Disable_DBG_Port_1)) != 0u)
		  {
		      SetDBGRegPinStatus(0U, 0U); /*Disable JTAG serial wire clk*/
		      SetDBGRegPinStatus(1U, 0U); /*Disable JTAG serial wire data */
		      uint8_t dbgStatus = GetDBGRegPinStatus(3U);
		      DEBUG_APP("\nDBG TDI Pin Status ", true, dbgStatus);
		      dbgStatus = GetDBGRegPinStatus(2U);
		      DEBUG_APP("\nDBG TDO Pin Status ", true, dbgStatus);
		      dbgStatus = GetDBGRegPinStatus(1U);
		      DEBUG_APP("\nDBG SWDIO Pin Status ", true, dbgStatus);
		      dbgStatus = GetDBGRegPinStatus(0U);
		      DEBUG_APP("\nDBG SWCLK Pin Status ", true, dbgStatus);


		  }

		  if((flags & FLAGS_BIT_INDEX(TMR_Smoke_Fast_Flame_Disable_1)) != 0u)
		  {
		      smoke_fast_flame_mode_st = false;
		      DEBUG_APP("\nFast flame mode OFF ", false, 0u);
		  }

		  if ((flags & FLAGS_BIT_INDEX(TMR_Extended_Laser_BIST_1)) != 0u)
		  {
		  	  DEBUG_APP("Laser BIST started", false, 0u);
		  	  handle_state_laser_extended_test();
		  }
		}
	}
}

void wdogTimer_flagSet(WDOGTimer_Flags_TypeDef flag) {
	wdog_task_flags |= (1u << (uint32_t) flag);
}

//*******************************************

//*******************************************
/**
 * @brief    This will be called by the Micrium OS idle task when there is no other task ready to run.
 *           We enter the lowest possible energy mode; usually EM2.
 */
#ifdef SL_CATALOG_POWER_MANAGER_PRESENT

#else
  void OSIdleHook(void)
  {
    while(1)
      {
        EMU_EnterEM2(true);
      }
    //(void) SLEEP_Sleep();
  }
#endif

void OSIdleEnterHook(void) {
	//ABR. To add any functionality that might be needed in this project
#ifdef DEBUG_MODE /*MUA TODO: Check if XTAL Tuning is needed in Development Build*/
	//GPIO_PinOutClear(DEF_LXTAL_TUNE_PORT, DEF_LXTAL_TUNE_PIN);
#endif
}

void OSIdleExitHook(void) {
	//ABR. To add any functionality that might be needed in this project
}

#if 0
void OSIdleContextPowerManagerHook(void)
{
#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
  RTOS_ERR err;

  if(hal_get_ads_state() == Ads_offBase)
    {
      GPIO_PinOutClear( MCU1_SOILA_ENABLE_PORT, MCU1_SOILA_ENABLE_PIN );
      GPIO_PinOutClear( MCU1_SOILB_ENABLE_PORT, MCU1_SOILB_ENABLE_PIN );
      GPIO_PinOutClear(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN);
      GPIO_PinOutClear(DEF_HEAT_POWER_PORT, DEF_HEAT_POWER_PIN);
      GPIO_PinOutClear(DEF_LIGHT_SRC_PORT, DEF_LIGHT_SRC_PIN);
      GPIO_PinOutClear( DEF_ASSIST_LIGHT_PORT, DEF_ASSIST_LIGHT_RESET_PIN );
      GPIO_PinOutClear( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );
    }

  OSSchedLock(&err);
  // OS_ASSERT_DBG_NO_ERR(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE, RTOS_ERR_ASSERT_CRITICAL_FAIL,; );
  sl_power_manager_sleep();
  OSSchedUnlock(&err);
  //OS_ASSERT_DBG_NO_ERR(RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE, RTOS_ERR_ASSERT_CRITICAL_FAIL,; );

#endif
}
#endif
