/***************************************************************************//**
 * @file  hal_BURTCTimer.c
 * @brief This project uses the BURTC (Backup Real Time Counter) to tick every
 *        10 seconds.
 *
 * @project P0200 Techem Core Firmware
 * @date    16 March 2022
 * @author  abenrashed
 *******************************************************************************/
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
#include "em_wdog.h"
#include "hal_BURTCTimer.h"
#include "hal_gpio.h"
#include "events.h"
#include "app.h"

#include "sl_device_init_lfxo.h"
#include "sl_device_init_lfxo_config.h"

//PowRed
#include "hal_switches.h"
#include "led_buzzer.h"

#define DEF_BURTCTIMER_SEC_COUNTS_OnBase    1u
#define DEF_BURTCTIMER_SEC_COUNTS_OffBase   5u  //10u
#define DEF_HEARTBEAT_PERIOD_SEC_OnBase     6u  //PowRed testing(58u)
#define DEF_HEARTBEAT_PERIOD_SEC_OffBase    30  //60u

volatile BURTCTimer_TypeDef Event_Timer[(uint8_t) NO_OF_EVENTS];
static Ads_state_t current_ads_state = Ads_onBase;

static uint8_t BURTCTIMER_period = DEF_BURTCTIMER_SEC_COUNTS_OnBase;
static uint8_t HEARTBEAT_period = DEF_HEARTBEAT_PERIOD_SEC_OnBase;

/**
 * BURTCTimer Callback
 * @req
 * @brief   Sets relevant event flags.
 * @details Each event arising is flagged by the relevant flag, semaphore, etc.
 *          When that timer period passes, the event is flagged in a callback function.
 */
static void BURTCTimer_CallBack(uint32_t *event) {
	RTOS_ERR err;
	uint32_t subgroub[2];
#ifdef DEBUG_MODE /*MUA TODO: Check if XTAL Tuning is needed in Development Build*/
	//GPIO_PinOutSet(DEF_LXTAL_TUNE_PORT, DEF_LXTAL_TUNE_PIN);
#endif
	subgroub[0] = event[0];
	subgroub[1] = event[1];

	OSFlagPost(&Event_Flags_SubGroup[0], /*Pointer to user-allocated event flag.*/
	subgroub[0],
	OS_OPT_POST_FLAG_SET,    //Set the flag
			&err);
	/*   Check error code.                                  */
	if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
		RTOS_ERR_SET(err, RTOS_ERR_FAIL)
	;
	}
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSFlagPost(&Event_Flags_SubGroup[1], /*Pointer to user-allocated event flag.*/
	subgroub[1],
	OS_OPT_POST_FLAG_SET,    //Set the flag
			&err);
	/*   Check error code.                                  */
	if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
		RTOS_ERR_SET(err, RTOS_ERR_FAIL)
	;
	}
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/**************************************************************************//**
 * @brief  BURTC Handler
 *****************************************************************************/
void BURTC_IRQHandler(void) {
CORE_DECLARE_IRQ_STATE;
uint8_t index;
uint32_t events_detected[2] = { 0U }; //Initialise detected events to none
RTOS_ERR err;

//PowRed
static uint8_t BURTCTIMER_sec_counter = 0u;
static uint8_t HEARTBEAT_sec_counter = 0u;


OSIntEnter();
CORE_ENTER_ATOMIC();

//PowRed
//poll ADS switch every second
if(current_ads_state == Ads_offBase)
  {
    OSFlagPost(&Event_Switches, EVENT_ADS_INTERRUPT, OS_OPT_POST_FLAG_SET,
    &err); /* post the ADS interrupt status */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); /*   Check the OS error code. */
  }

HEARTBEAT_sec_counter++;

if(HEARTBEAT_sec_counter >= HEARTBEAT_period)
  {
    HEARTBEAT_sec_counter = 0;
    setHeartBeatFlag(true);
    OSFlagPost(&Event_Switches, EVENT_ADS_INTERRUPT, OS_OPT_POST_FLAG_SET,
    &err); /* post the ADS interrupt status */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); /*   Check the OS error code. */
  }


BURTCTIMER_sec_counter++;

if(BURTCTIMER_sec_counter >= BURTCTIMER_period)
{
    BURTCTIMER_sec_counter = 0u;

    for (index = 0U; index < (uint8_t) NO_OF_EVENTS; index++) //Check all events timers if expired
      {
        if (Event_Timer[index].enabled) //Only active timers checked
          {
            if (Event_Timer[index].period > 0U) //When timer elapsed
              {
                Event_Timer[index].period--;
                if (Event_Timer[index].period == 0U)
                  {
                    if (index < 32U)
                      {
                        events_detected[0] |= (1U << index); //Set detected event index
                      }
                    else
                      {
                        events_detected[1] |= (1U << (index - 32U));
                      }

                    if (Event_Timer[index].periodic)
                      {
                        Event_Timer[index].period = Event_Timer[index].value; //Restart the periodic timers
                      }
                    else
                      {
                        Event_Timer[index].enabled = false; //Disabled the one shot timers when expired
                      }
                  }
              }
          }
      }

    if (events_detected[0] > 0U || events_detected[1] > 0U) //Some events triggered
      {
        BURTCTimer_CallBack(events_detected);
      }

    //BURTC_IntClear(BURTC_IF_COMP); //compare match
}

BURTC_IntClear(BURTC_IF_COMP); //compare match

CORE_EXIT_ATOMIC();
OSIntExit();
}

/**
 * BURTCTimer Start
 * @req
 * @brief Starts an event timer.
 * @param event: the event of concern
 * @param periodic: True, periodic, False one shot
 * @param period: The timer period in counts
 */

void BURTCTimer_Start(BURTCTimer_Events_TypeDef event, bool periodic, uint32_t period) {
RTOS_ERR err;

OSSchedLock(&err);
Event_Timer[event].periodic = periodic; /*Set to periodic or one shot*/
Event_Timer[event].period = period; /*Set the period counts*/
Event_Timer[event].value = period; /*Keep the period value*/
Event_Timer[event].enabled = true; /*Enable the timer*/
OSSchedUnlock(&err);
}

/**
 * BURTCTimer Stop
 * @req
 * @brief Stops an event timer.
 * @param event: the event of concern
 */
uint32_t BURTCTimer_Stop(BURTCTimer_Events_TypeDef event) {
RTOS_ERR err;
uint32_t stoppedValue;

OSSchedLock(&err);
Event_Timer[(uint8_t) event].enabled = false; /*Stop the timer*/
stoppedValue = Event_Timer[(uint8_t) event].period;
Event_Timer[(uint8_t) event].period = 0u;
OSSchedUnlock(&err);

return stoppedValue; /*Return value at moment stopped*/
}

/**
 * BURTCTimer Stop a range of timers
 * @req
 * @brief Stops all events timers after a certain event.
 * @param event: the event index after which all events timers to be stopped
 */

void BURTCTimer_StopFrom(BURTCTimer_Events_TypeDef event) {
RTOS_ERR err;
uint8_t index;

OSSchedLock(&err);
for (index = (uint8_t) event; index < (uint8_t) NO_OF_EVENTS; index++) /*All specified timers after the event index*/
{
Event_Timer[index].enabled = false; /*Stop the timer*/
Event_Timer[(uint8_t) event].period = 0u;
}
OSSchedUnlock(&err);
}

/**
 * BURTCTimer Event 0 Stop a range of timers
 * @req
 * @brief Stops all events timers after a certain event.
 * @param event: the event index after which all events timers to be stopped
 */

void BURTCTimerEvent0_StopFrom(BURTCTimer_Events_TypeDef event) {
RTOS_ERR err;
uint8_t index;

OSSchedLock(&err);
for (index = (uint8_t) event; index < (uint8_t) EVENT0_LIMIT; index++) /*All specified timers after the event index*/
{
Event_Timer[index].enabled = false; /*Stop the timer*/
Event_Timer[(uint8_t) event].period = 0u;
}
OSSchedUnlock(&err);
}

/**************************************************************************//**
 * @brief  Initialise GPIOs for push button and LED
 *****************************************************************************/
/* removed dead code */

/**************************************************************************//**
 * @brief  Configure BURTC to interrupt every BURTC_IRQ_PERIOD
 *
 *****************************************************************************/
void BURTC_init(void) {
CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_LFXO);  //ABR Techem.
CMU_ClockEnable(cmuClock_BURTC, true);

BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
burtcInit.compare0Top = true; // reset counter when counter reaches compare value
burtcInit.clkDiv = burtcClkDiv_128; //ABR Techem.
BURTC_Init(&burtcInit);

BURTC_CounterReset();
BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_10SEC);

BURTC_IntEnable(BURTC_IEN_COMP);    // compare match
NVIC_EnableIRQ(BURTC_IRQn);
BURTC_Enable(true);
}

/**************************************************************************//**
 * @brief  ReConfigure BURTC with stored ctune value, to interrupt every BURTC_IRQ_PERIOD
 *
 *****************************************************************************/
void BURTC_reinit(uint8_t compVal) {
/*
 uint32_t dataBuff = 0u;
 uint8_t ctune_val = 0u;
 bool status = 0u;

 CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

 lfxoInit.mode = SL_DEVICE_INIT_LFXO_MODE;
 lfxoInit.timeout = SL_DEVICE_INIT_LFXO_TIMEOUT;
 */

/*ABR To be added.  Wait until LFXO not busy and is ready for the configuration */
    //while(??);

/*ABR To be added.  Read ctune from EEPROM */
/* Get ctune value */
/*Flash Read ctune*/

/*ABR to be added. Reinitialise with new ctune*/
/*
 lfxoInit.capTune = ctune;
 CMU_LFXOInit(&lfxoInit);
 CMU_LFXOPrecisionSet(SL_DEVICE_INIT_LFXO_PRECISION);
 */

/*ABR to be added*/
/*
 CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
 // wait until the oscillator is ready
 while (??);
 */

BURTC_Enable(false);
CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_LFXO);    //ABR.
CMU_ClockEnable(cmuClock_BURTC, true);

BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
burtcInit.compare0Top = true; // reset counter when counter reaches compare value
burtcInit.clkDiv = burtcClkDiv_128;
BURTC_Init(&burtcInit);

BURTC_CounterReset();
if(compVal == 10U)
{
BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_10SEC);
}
else if (compVal == 8U)
{
    BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_8SEC);
}
else if (compVal == 5U)
{
    BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_5SEC);
}
else if (compVal == 4U)
{
    BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_4SEC);
}
else if( compVal == 2U)
{
    BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_2SEC);
}
else
{
    BURTC_CompareSet(0, BURTC_COMPARE_FOR_IRQ_PERIOD_1SEC);
}

BURTC_IntEnable(BURTC_IEN_COMP);    // compare match
NVIC_EnableIRQ(BURTC_IRQn);
BURTC_Enable(true);
WDOG_Feed();
}
/**************************************************************************//**
 * @brief  check event flag
 * @param none
 * return event set value
 *
 *****************************************************************************/
uint32_t FLAGS_BIT_INDEX (uint8_t data)
{
  uint32_t data_bytes=0;
  data=data % 32;
  data_bytes= (uint32_t)(1u<<data);
  return data_bytes;
}

/**************************************************************************//**
 * @brief  set BURTC timer for ADS checking
 * @param none
 * return none
 *
 *****************************************************************************/
void set_BURTCTimer_ads_polling(uint8_t newState)
{
  BURTC_IntDisable(BURTC_IEN_COMP);

  if(newState == Ads_onBase)
    {
      current_ads_state = Ads_onBase;
      BURTCTIMER_period = DEF_BURTCTIMER_SEC_COUNTS_OnBase;
      HEARTBEAT_period = DEF_HEARTBEAT_PERIOD_SEC_OnBase;
      BURTC_reinit(Ten_Second_Period);
    }
  else if(newState == Ads_offBase)
    {
      current_ads_state = Ads_offBase;
      BURTCTIMER_period = DEF_BURTCTIMER_SEC_COUNTS_OffBase;
      HEARTBEAT_period = DEF_HEARTBEAT_PERIOD_SEC_OffBase;
      BURTC_reinit(Two_Second_Period);
    }
  else
    {
      BURTC_IntEnable(BURTC_IEN_COMP);
    }

}

bool BURTCTimer_Get_Event_Enable(BURTCTimer_Events_TypeDef event)
{
  RTOS_ERR err;
  OSSchedLock(&err);
  bool enable = Event_Timer[(uint8_t)event].enabled;
  OSSchedUnlock(&err);
  return  enable;
}
