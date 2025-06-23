/***************************************************************************//**
 * @file  hal_LETimer.c
 * @brief handler for the low power timer
 * @project P0200 Techem Core Firmware
 * @date    21 March 2022
 * @author  abenrashed
 *******************************************************************************/
#include "acquisition_Smoke.h"
#include  <cpu/include/cpu.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>
#include  <common/include/common.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include  <common/include/rtos_prio.h>

#include "em_cmu.h"
#include "em_letimer.h"
#include "em_core.h"

#include "hal_LETimer.h"
#include "hal_BURTCTimer.h"
#include "events.h"
#include "app.h"
#include "led_buzzer.h"
#include "hal_switches.h"
#include "spi_comms.h"
#include "diagnostics.h"
#include "assistance_light.h"

#define ONE_MS_FREQ 1000u

OS_MUTEX leTimerMutex;
//OS_SEM leTimerSem;

typedef struct {
	bool enabled;
	uint32_t period;
} LETimer_TypeDef;

volatile LETimer_TypeDef LETimer_events[(uint8_t) LETIMER_EVENTS_SIZE];

static bool timerActive = false; /*To identify if timer is already enabled*/

/*Local functions*/
static void LETimer_checkAndStop(void);
static void LETimer_callback(uint32_t eventsComplete);

/**************************************************************************//**
 * @brief Blocking delay of specified ms
 * @param ms_period
 * @note should NOT be used from an interrupt

 *****************************************************************************/
void LETimer_delay_ms(uint32_t ms_period) {
	RTOS_ERR err;
	bool complete = false;
	uint32_t failTimeout = 5000000;

	OSSchedLock(&err); /* Lock the scheduler, do no other tasks can take over mid process  */
	LETimer_start(LETIMER_BLOCKING, ms_period);
	while ((complete == false) && (failTimeout > 0u)) {
		failTimeout--;

		if (LETimer_events[LETIMER_BLOCKING].enabled == false) {
			complete = true;
		}
	}
	OSSchedUnlock(&err); /* Unlock the scheduler.*/

	if (failTimeout == 0u) {
		/*if reaching here then, error has occurred with blocking timer*/
	}
}

/**************************************************************************//**
 * Start event LE Timer
 * @param event
 * @param period
 *****************************************************************************/
extern void LETimer_start(LETimer_Events_TypeDef event, uint32_t period) {
	if (LETIMER_EVENTS_SIZE > event) {
		RTOS_ERR err;

		OSMutexPend(&leTimerMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    NVIC_DisableIRQ(LETIMER0_IRQn);
    OSSchedLock(&err);

		if (timerActive == false) {
			//LETIMER0->CMD = LETIMER_CMD_START;
			LETIMER_Enable(LETIMER0, true);
			timerActive = true;
		}
		LETimer_events[event].period = period+1;
		LETimer_events[event].enabled = true;

		OSSchedUnlock(&err);
		NVIC_EnableIRQ(LETIMER0_IRQn);


		OSMutexPost(&leTimerMutex, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	} else {
		/*Unknown event*/
	}
}

/**************************************************************************//**
 * @brief stops event LE timer
 * @param event Id of the timer object to be stopped
 * @return Remaining period left on timer before stopped.
 *****************************************************************************/
extern uint32_t LETimer_stop(LETimer_Events_TypeDef event) {
	uint32_t remaining = 0u;

	if (LETIMER_EVENTS_SIZE > event) {
		RTOS_ERR err;

		OSMutexPend(&leTimerMutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

		NVIC_DisableIRQ(LETIMER0_IRQn);
		OSSchedLock(&err);

		remaining = LETimer_events[event].period; /*get value before its stopped*/
		LETimer_events[event].period = 0u;
		LETimer_events[event].enabled = false;
		LETimer_checkAndStop();

	  OSSchedUnlock(&err);
	  NVIC_EnableIRQ(LETIMER0_IRQn);

		OSMutexPost(&leTimerMutex, OS_OPT_POST_NONE, &err);
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	} else {
		/*Unknown event*/
	}
	return remaining;
}

/**************************************************************************//**
 * @brief checks if all timers objects have ended and stops the timer IRQ
 *****************************************************************************/
static void LETimer_checkAndStop(void) {
	bool allStopped = true;
	LETimer_Events_TypeDef i;

	for (i = 0; i < LETIMER_EVENTS_SIZE; i++) {
		if (LETimer_events[i].enabled == true) {
			allStopped = false;
			break;
		}
	}
	if (allStopped == true) {
		//LETIMER0->CMD = LETIMER_CMD_STOP;
		LETIMER_Enable(LETIMER0, false);
		timerActive = false;
	}
}

/**************************************************************************//**
 * @brief LETimer IRQ callback function
 * @param eventsComplete
 *****************************************************************************/
static void LETimer_callback(uint32_t eventsComplete)
{
	RTOS_ERR err;

	//ABR. Add all events needed as per the project requirements
	if ((eventsComplete & (1u << (uint32_t) LETIMER_BLOCKING)) != 0u)
	{
		/*Don't need to do anything with this as blocking functions watches for expire*/
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_DIAG_DELAY)) != 0u)
	{
		OS_FLAGS delayedFlags = diagnostics_GetDelayedFlags();
		if (delayedFlags != 0u)
		{
			/* post all delayed flags for the expired diagnostics tests */
			OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
					   delayedFlags,
					   OS_OPT_POST_FLAG_SET,        /*   Set the flag.                        */
					   &err);
			/*   Check error code.                                  */
			APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		}
		else
		{
			/*outdated callback, ignore*/
		}
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_LEDBUZZ)) != 0u)
	{
		LEDBuzz_Post(LEDBuzz_LETimerTimeout);
	}

	//PowRed
	/*
	if ((eventsComplete & (1u << (uint32_t) LETIMER_HEARTBEAT)) != 0u) {
		LEDBuzz_Post(PatternHeartbeat);
	}
	*/
	if ((eventsComplete & (1u << (uint32_t) LETIMER_SELFTEST_PRESSED)) != 0u)
	{
		OSFlagPost(&Event_Switches,    /*   Pointer to user-allocated event flag.*/
		SWITCH_SELFTEST_PRESS_TIMEOUT, /*   event bit-mask.                      */
		OS_OPT_POST_FLAG_SET,          /*   Set the flag.                        */
		&err);
		                               /*   Check error code.                    */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_SELFTEST_RELEASED)) != 0u)
	{
		OSFlagPost(&Event_Switches,      /*   Pointer to user-allocated event flag.         */
		SWITCH_SELFTEST_RELEASE_TIMEOUT, /*   event bit-mask.                                */
		OS_OPT_POST_FLAG_SET,            /*   Set the flag.                                 */
		&err);
		                                /* Check error code. */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_PRESS_MONITOR_TIMER)) != 0u)
	{
		OSFlagPost(&Event_Switches, /*   Pointer to user-allocated event flag.         */
		SWITCH_PATTERN_TIMEOUT, /*   event bit-mask.                                */
		OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
		&err);
		/* Check error code. */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

//	if ((eventsComplete & (1u << (uint32_t) LETIMER_PRESS_MONITOR_TIMER))
//			!= 0u) {
//		OSFlagPost(&Event_Switches, /*   Pointer to user-allocated event flag.         */
//		SWITCH_PATTERN_TIMEOUT, /*   event bit-mask.                                */
//		OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
//		&err);
//		/* Check error code. */
//		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
//	}

//	if ((eventsComplete & (1u << (uint32_t) LETIMER_FAST_FLAME_MODE)) != 0u)
//	{
//		smoke_fast_flame_mode_st = false; /* fast flame mode timeout */
//	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_SMOKE_FASTRATE)) != 0u)
	{
		OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
		EVENT_SMOKE_MEASUREMENT_0, /*   event bit-mask.				*/
		OS_OPT_POST_FLAG_SET, /*   Set the flag.								 */
		&err);
		/*	 Check error code */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_LIGHTSENSOR)) != 0u)
	{
#if 0
    OSFlagPost(&Event_Flags_SubGroup[FLAGS_SUBGROUP_INDEX(AmbientLight_measure_event)], /* Pointer to user-allocated event flag. */
               FLAGS_BIT_INDEX(AmbientLight_measure_event),
               OS_OPT_POST_FLAG_SET,  /* Set the flag. */
               &err);
    /* Check error code. */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#endif
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_SPI_TIMEOUT)) != 0u)
	{
        SPIComms_HandleShortTimeout();
	}
/*
	if ((eventsComplete & (1u << (uint32_t) LETIMER_FAILED_SPI_COMMS_TIMEOUT)) != 0u)
	{
        SPIComms_HandleLongTimeout();
	}
*/

	if ((eventsComplete & (1u << (uint32_t) LETIMER_FMT_Smoke)) != 0u)
	{
	    OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
	    EVENT_SMOKE_MEASUREMENT_0, /*   event bit-mask.       */
	    OS_OPT_POST_FLAG_SET, /*   Set the flag.                 */
	    &err);
	    /*   Check error code */
	    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

	if ((eventsComplete & (1u << (uint32_t) LETIMER_FMT_BIST_Smoke)) != 0u)
	{
	      OSFlagPost(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
	      EVENT_SMOKE_BIST_0, /*   event bit-mask.       */
	      OS_OPT_POST_FLAG_SET, /*   Set the flag.                 */
	      &err);
	      /*   Check error code */
	      APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}

  if ((eventsComplete & (1u << (uint32_t) LETIMER_EXT_USER_TEST)) != 0u)
  {
    OSFlagPost(&Event_Switches, /*   Pointer to user-allocated event flag.         */
    SWITCH_EXT_USER_TEST, /*   event bit-mask.                                */
    OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
    &err);
    /* Check error code. */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
  }

  if ((eventsComplete & (1u << (uint32_t) LETIMER_EXT_USER_LASER_TEST)) != 0u)
  {
    OSFlagPost(&Event_Flags_SubGroup[1], /*   Pointer to user-allocated event flag.         */
    EVENT_USER_EXTND_LASER_BIST_1, /*   event bit-mask.                                */
    OS_OPT_POST_FLAG_SET, /*   Set the flag.                                 */
    &err);
    /* Check error code. */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
  }

	/*ABR.  Add top group set flags if any subgroup was set*/
}

/**************************************************************************//**
 * @brief  LETimer Interrupt Handler
 *****************************************************************************/
void LETIMER0_IRQHandler(void) {
	//RTOS_ERR err;
	bool allStopped = true;
	uint32_t eventsComplete = 0u;
	LETimer_Events_TypeDef i = 0u;
	CORE_irqState_t irqState;

	OSIntEnter();
	irqState = CORE_EnterAtomic();

	for (i = 0u; i < LETIMER_EVENTS_SIZE; i++) {
		if (LETimer_events[i].enabled == true) {
			if (LETimer_events[i].period > 0u) {
				LETimer_events[i].period--;
				if (LETimer_events[i].period <= 0u) {
					LETimer_events[i].enabled = false;
					eventsComplete |= (1u << i); /*Set detected event index*/
				} else {
					allStopped = false;
				}
			}
		}
	}
	if (allStopped == true) {
		LETIMER_Enable(LETIMER0, false);
		timerActive = false;
	}

	if (eventsComplete > 0u) {
		LETimer_callback(eventsComplete);
	}

	LETIMER0->IF_CLR = LETIMER_IF_UF; // Clear interrupt flag */

	CORE_ExitAtomic(irqState);
	OSIntExit();
}

/**
 * @brief the initialise the hardware low power timer
 */
extern void LETimer_init(void) {
	RTOS_ERR err;
	LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

	OSMutexCreate(&leTimerMutex, "leTimer mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFXO);
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	uint32_t topValue = (CMU_ClockFreqGet(cmuClock_LETIMER0) / ONE_MS_FREQ);/* for 32768KHz, 32 gives 0.976 msec per tick,
	 while rounded up to 33 (by adding 1) gives 1.007 msec tick*/

	/*Initialise the timer but don't enable yet*/
	letimerInit.enable = false;
	letimerInit.comp0Top = true;
	letimerInit.topValue = topValue;
	LETIMER_Init(LETIMER0, &letimerInit);

	/*Enable interrupts*/
	LETIMER0->IEN = LETIMER_IEN_UF;

	for (uint8_t i = 0u; i < LETIMER_EVENTS_SIZE; i++) {
		LETimer_events[i].enabled = false;
	}

	/*Finally Enable the timer*/
	LETIMER_Enable(LETIMER0, true);
	NVIC_EnableIRQ(LETIMER0_IRQn);
	//PowRed
  //LETimer_start(LETIMER_HEARTBEAT, LEDBuzz_GetHeartBeatPeriodInMS());	/* start timer for heartbeat LED 			*/
}

