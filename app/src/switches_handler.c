/*
 * switches_handler.c
 *
 *  Created on: 15 Jun 2022
 *      Author: uhegde
 */

#include "os.h"
#include "hal_switches.h"
#include "switches_handler.h"
#include "comms_handler.h"
#include "fault_handler.h"
#include "hal_gpio.h"
#include "hal_LETimer.h"
#include "app.h"
#include "system_events.h"
#include "hal_BURTCTimer.h"
#include "data_logging.h"
#include "production.h"


#define DEBOUNCE_PERIOD 20u

typedef enum {
	switch_ads, /* switch type is ADS */
	switch_test_btn, /* switch type is test and mute button */
} switch_type_t;

OS_FLAG_GRP Event_Switches; /* switches status event flag */
static uint8_t
switches_handler_debounce(GPIO_Port_TypeDef port, uint16_t pin,
		switch_type_t type);

static bool btnTimer = false;
static bool btn_shortPress = false;
static bool btn_longPress = false;
static bool btn_longHold = false;
static bool btn_5shortPress = false;
static uint8_t userExtTest = 0u;
static bool first_ads_check = true;

/**
 * @brief button and ADS switch status handler task
 * @req PTR-648, PTR-1386
 */
void switches_handler_task(void *arg) {
	RTOS_ERR err;
	OS_FLAGS flags;
  const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)arg;
	uint8_t newState = 0u;



    OSTaskRegSet(DEF_NULL, 0, ptrToMyTCB, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	/*Kick off ADS Switch event to capture the initial state of the ADS switch and 
	move to the correct mode (StandBy or Commissioning)*/
	OSFlagPost(&Event_Switches, EVENT_ADS_INTERRUPT, OS_OPT_POST_FLAG_SET,
			   &err);													/* post the ADS interrupt status */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); /*   Check the OS error code. */
	while (true) {
		flags = OSFlagPend(&Event_Switches, /* Pointer to user-allocated event flag. */
		0xffffffffu, /* Flag bit mask to match. */
		0, /* Wait indefinitely. */
		OS_OPT_PEND_FLAG_SET_ANY | /* Wait until ANY flags are set and */
		OS_OPT_PEND_BLOCKING | /* task will block and */
		OS_OPT_PEND_FLAG_CONSUME, /* consume flags */
		DEF_NULL, /* Time stamp is not used. */
		&err);
		/* Check error code. */
		if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
			if ((flags & EVENT_BUTTON_INTERRUPT) != 0u) {
				OSSchedLock(&err); /*Lock the scheduler*/
				GPIO_IntDisable(DEF_SELFTEST_BTN_PIN); /* disable the interrupt */
				newState = switches_handler_debounce(DEF_SELFTEST_BTN_PORT,
						(uint16_t) DEF_SELFTEST_BTN_PIN, switch_test_btn); /* switch debounce check */
				GPIO_IntClear(DEF_SELFTEST_BTN_PIN); /* clear any pending interrupt */
				GPIO_IntEnable(DEF_SELFTEST_BTN_PIN); /* re-enable the interrupt */
				OSSchedUnlock(&err); /*Unlock the scheduler*/

				if (newState != hal_switches_get_state()) /* compare the old and new button status */
				{
					hal_switches_set_state(newState); /* set the new ADS state */
				}
			}
			if ((flags & EVENT_ADS_INTERRUPT) != 0u) {
				OSSchedLock(&err); /*Lock the scheduler*/
				GPIO_IntDisable(DEF_ADS_PIN); /* disable the interrupt */

				//PowRed
				GPIO_PinModeSet(DEF_ADS_PORT, DEF_ADS_PIN, gpioModeInputPullFilter, 1u);/*ADS Button*/

				newState = switches_handler_debounce(DEF_ADS_PORT,
						(uint16_t) DEF_ADS_PIN, switch_ads); /* ADS status debounce check */

				//PowRed
        if(newState == Ads_offBase)
        {
          GPIO_PinModeSet(DEF_ADS_PORT, DEF_ADS_PIN, gpioModeInput, 0u);/*ADS Button*/
        }

				GPIO_IntClear(DEF_ADS_PIN); /* clear any pending interrupt */
				GPIO_IntEnable(DEF_ADS_PIN); /* re-enable the interrupt */
				OSSchedUnlock(&err); /*Unlock the scheduler*/
				DEBUG_SWI_HAND("\nADS_PIN new state", true, newState);

				//PowRed
				if(true == first_ads_check)
				{
				    first_ads_check = false;
				    set_BURTCTimer_ads_polling(newState);
				}
				if (newState != hal_get_ads_state()) /* compare the old and new ADS status */
				{
					hal_set_ads_state(newState); /* set the new ADS state */
					set_BURTCTimer_ads_polling(newState);
				}
			}
			if ((flags & SWITCH_SELFTEST_PRESS_TIMEOUT) != 0u) {
        /* Button stuck fault condition identified */
			    DEBUG_SWI_HAND("\nStuck button fault", false, 0u);
        FaultHandler_FaultSet(TestButtonFault);
        /* Do not send user bist log msg to mcu2 for testbutton as requested by techem */
        DataLogging_SetLogMsgToMCU(false);
        DataLogging_SetEventLogbookRecord(DEF_LBE_USER_BIST, NULL);
        DataLogging_SetLogMsgToMCU(true);
        DataLogging_SetMinorFault(FaultTestButton, true);
        hal_switches_set_pattern(Button_Stuck);
			}
			if ((flags & SWITCH_SELFTEST_RELEASE_TIMEOUT) != 0u) {
			  OSSchedLock(&err); /*Lock the scheduler*/
				uint32_t pattern_time = BUTTON_PATTERN_MONITOR
						- LETimer_stop(LETIMER_PRESS_MONITOR_TIMER); /* button press sequence ended */
				pattern_time = pattern_time - RELEASE_GAP_MAX; /* ignoring the 1 second release timeout time */
				switch (press_counter) {
				case single_press:
					if (pattern_time <= SHORT_PRESS_TIME) /* single short press in 1 second */
					{
					    DEBUG_SWI_HAND("1-press", true, pattern_time);
						hal_switches_set_pattern(Button_SingleShortPress); /* single short press detected */
						set_ftm_btn_pattern(true, false, false, false);
					} else if ((pattern_time > SHORT_PRESS_TIME)
							&& (pattern_time <= LONG_PRESS_HOLD_TIME)) {
					    DEBUG_SWI_HAND("long press ", true, press_counter);
						hal_switches_set_pattern(Button_LongPress); /* long press detected */
						set_ftm_btn_pattern(false, true, false, false);
					} else if ((pattern_time > LONG_PRESS_HOLD_TIME)) {
					    DEBUG_SWI_HAND("long press hold", true, press_counter);
						hal_switches_set_pattern(Button_LongPressHold); /* long press hold detected */
						set_ftm_btn_pattern(false, false, true, false);
					} else {
						/* to avoid MISRA violation */
					}
					break;
				case double_press:
					if ((pattern_time) <= SHORT_PRESS_TIME) /* double short press in 1 second */
					{
					    DEBUG_SWI_HAND("2-press", true, pattern_time);
						hal_switches_set_pattern(Button_DoubleShortPress); /* double short press detected */
					}
					break;
				case five_press:
					if ((pattern_time > SHORT_PRESS_TIME)
							&& (short_press_counter == 5u)) {
					    DEBUG_SWI_HAND("5-short-press", true, pattern_time); /* sequence press pattern detected */
						hal_switches_set_pattern(Button_FiveShortPress); /* five short presses detected */
						set_ftm_btn_pattern(false, false, false, true);
					}
					break;
				default:
					/* to avoid MISRA violation */
					break;
				}
				press_counter = 0u; /* reset the pattern counter */
				short_press_counter = 0; /* reset the short press pattern counter */
				long_press_pattern = switch_no_long_press;
				OSSchedUnlock(&err); /*Unlock the scheduler*/
			}
			if ((flags & SWITCH_PATTERN_TIMEOUT) != 0u) {
			    DEBUG_SWI_HAND("pattern monitoring ended ", true, press_counter); /* pattern monitoring has ended */
				OSSchedLock(&err); /*Lock the scheduler*/
				if (press_counter == 1u) {
					if (long_press_pattern == switch_long_press_hold) {
						//hal_switches_set_pattern(Button_LongPressHold); /* five short presses detected */
						//DEBUG_SWI_HAND("long press hold", true, press_counter);
					} else if (long_press_pattern == switch_long_press) {
						//hal_switches_set_pattern(Button_LongPress); /* five short presses detected */
						//DEBUG_SWI_HAND("long press counter", true, press_counter); /* maximum allowed multiple press pattern count is 5 */
					} else {
						/* to avoid MISRA violation */
					}
					press_counter = 0u; /* reset the pattern counter */
					short_press_counter = 0; /* reset the short press pattern counter */
					long_press_pattern = switch_no_long_press;
				}
				else if (press_counter == 0u)
				{
				    DEBUG_SWI_HAND("extra long press counter", true, press_counter);
				    const uint32_t prod_comp = prod_get_value( );

				    if ((getBehavioural_Operational_State() == state_Idle)
				        && (hal_get_ads_state() == Ads_onBase) &&(prod_comp == PROD_COMP_BB))
				    {
				        DEBUG_SWI_HAND("** Button_userExtndTestLaser", true, press_counter);
				        hal_switches_set_pattern(Button_userExtndTestLaser);
				        set_ftm_btn_pattern(false, false, true, false);
				        press_counter = 0u; /* reset the pattern counter */
				        short_press_counter = 0; /* reset the short press pattern counter */
				        long_press_pattern = switch_no_long_press;
				    }
				}
				OSSchedUnlock(&err); /*Unlock the scheduler*/
			}

			if((flags & SWITCH_EXT_USER_TEST) != 0u)
			{
			    OSSchedLock(&err); /*Lock the scheduler*/
			    (void)LETimer_stop(LETIMER_EXT_USER_TEST);
			    userExtTest = switches_handler_debounce(DEF_SELFTEST_BTN_PORT,
			                (uint16_t) DEF_SELFTEST_BTN_PIN, switch_test_btn); /* switch debounce check */
			    if((userExtTest == 0U) && (press_counter == 0U) && (getBehavioural_Operational_State() == state_Idle))
			    {
			        DEBUG_SWI_HAND("long press hold ext user", false, 0u);
			        hal_switches_set_pattern(Button_userExtTest);
			        setEndExtUsrTest(true);
			    }
			    OSSchedUnlock(&err); /*Unlock the scheduler*/
			}
		}
	}
}

/**
 * @brief Debounce a switch
 * @details 1. Read the switch state
 * 			2. Wait for a delay of DEBOUNCE_PERIOD
 * 			3. Read switch state again
 * 			4. If both states the same then return new state
 * 			5. If different then delay again for DEBOUNCE_PERIOD/2 upto 2 times.
 * 			6. If still not same then return last state.
 * @note Function is blocking
 * @param port Port of input to debounce
 * @param pin Pin of input to debounce
 * @param switch type (ads or test button)
 * @return Debounced state of pin
 * @req PTR-648
 */
static uint8_t switches_handler_debounce(GPIO_Port_TypeDef port, uint16_t pin,
		switch_type_t type)
{
    uint8_t firstState, secondState;
    uint8_t retry = 3u; /* strike count to confirm the switches hardware fault */

    uint32_t debounceTimer = DEBOUNCE_PERIOD;
    while (retry > 0u)
    {
        firstState = (uint8_t) GPIO_PinInGet(port, pin); /* read the pin status */
        //PowRed
        #if 0
        if(hal_get_ads_state() == Ads_offBase)
        {
            LETimer_delay_ms(10u); /* debounce time delay */
        }
        else
        {
            LETimer_delay_ms(debounceTimer); /* debounce time delay */
        }
        #endif

        LETimer_delay_ms(debounceTimer); /* debounce time delay */

        secondState = (uint8_t) GPIO_PinInGet(port, pin);
        if (firstState == secondState)
        {
            retry = 0u; /*If states same then exit*/
        }
        else
        {
            retry--; /*Retry with half debounce period*/
            debounceTimer = (DEBOUNCE_PERIOD / 2u);
        }
    }
    if (type == switch_ads)
    {
        if (firstState != secondState)
        {
            return Ads_fault; /* ADS is faulty */
        }
        else
        {
            return secondState; /* No ADS fault identified */
        }
    }
    else
    {
        return secondState; /* in case of switch no setting of debounce error */
    }

}

/*******************************************************************************
 * @brief set_ftm_btn_pattern
 * @details set the pattern status for button press actions
 * @param bool btn_sp, bool btn_lp, bool btn_lh, bool btn_5p
 * @return n/a
 ******************************************************************************/
void set_ftm_btn_pattern(bool btn_sp, bool btn_lp, bool btn_lh, bool btn_5p)
{
   btn_shortPress = btn_sp;
   btn_longPress = btn_lp;
   btn_longHold = btn_lh;
   btn_5shortPress = btn_5p;
}

/*******************************************************************************
 * @brief set_ftm_btn_event
 * @details set the timeout event for pattern read status
 * @param bool bTimerOver
 * @return n/a
 ******************************************************************************/
void set_ftm_btn_event(bool bTimerOver)
{
  btnTimer = bTimerOver;
}

/*******************************************************************************
 * @brief get_ftm_btn_event
 * @details returns the timeout event for pattern read status
 * @param n/a
 * @return button event
 ******************************************************************************/
bool get_ftm_btn_event()
{
  return btnTimer;
}

/*******************************************************************************
 * @brief get_ftm_shortP
 * @details returns the short press status
 * @param n/a
 * @return btn_shortPress
 ******************************************************************************/
bool get_ftm_shortP()
{
  return btn_shortPress;
}

/*******************************************************************************
 * @brief get_ftm_longP
 * @details returns the long press status
 * @param n/a
 * @return btn_longPress
 ******************************************************************************/
bool get_ftm_longP()
{
  return btn_longPress;
}

/*******************************************************************************
 * @brief get_ftm_longHold
 * @details returns the long hold status
 * @param n/a
 * @return btn_longHold
 ******************************************************************************/
bool get_ftm_longHold()
{
  return btn_longHold;
}

/*******************************************************************************
 * @brief get_ftm_5shortPress
 * @details returns the 5short press within 5sec status
 * @param n/a
 * @return btn_5shortPress
 ******************************************************************************/
bool get_ftm_5shortPress()
{
  return btn_5shortPress;
}
