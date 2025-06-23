/*
 * hal_switches.c
 *
 *  Created on: 14 Jun 2022
 *      Author: uhegde
 */
#include "os.h"
#include "em_cmu.h"
#include "hal_gpio.h"
#include "hal_switches.h"
#include "comms_handler.h"
#include "gpiointerrupt.h"
#include "events.h"
#include "hal_LETimer.h"
#include "system_events.h"
#include "hal_BURTCTimer.h"
#include "led_buzzer.h"
#include "fault_handler.h"
#include "data_logging.h"
#include "spi_comms.h"
#include "assistance_light.h"

Button_state_t button_state = Button_Released;
Ads_state_t ads_state = Ads_offBase;
uint8_t press_counter = 0u; /* counter to monitor the number of button presses */
uint8_t short_press_counter = 0u; /* counter to monitor the number of short button presses */
Button_state_t curr_pattern;
switch_long_press_t long_press_pattern = switch_no_long_press; /* 1= long press, 2= long press and hold */
static bool endExtUsrTest = false;
static bool demountLongFlag = false;
static bool demountOneMinFlag = false;

void hal_switches_callback(uint8_t intNo); /* GPIO interrupt application call back function */

/**
 * @brief Initialise the hardware for the buttons and switches
 */
void hal_switches_init(void) {

	GPIO_ExtIntConfig(DEF_ADS_PORT, DEF_ADS_PIN, DEF_ADS_PIN, true, true, true); /* configure the interrupt for ADS pin */
	GPIO_ExtIntConfig(DEF_SELFTEST_BTN_PORT, DEF_SELFTEST_BTN_PIN, DEF_SELFTEST_BTN_PIN,true,true, true); /* configure the interrupt for self test pin */
	/* register callback function */
	GPIOINT_CallbackRegister(DEF_ADS_PIN, hal_switches_callback); /* register the interrupt handler callback for ADS pin */
	GPIOINT_CallbackRegister(DEF_SELFTEST_BTN_PIN, hal_switches_callback); /* register the interrupt handler callback for self test pin */

	button_state = Button_Released;
	ads_state = Ads_offBase;
	curr_pattern = Button_Released;
}

/**
 * @brief GPIO interrupt callback function, this is called every time GPIO status is changed.
 * @param gpio_int_no GPIO PIN number which caused interrupt
 */
void hal_switches_callback(uint8_t gpio_int_no) {
	RTOS_ERR err;
	if (gpio_int_no == DEF_SELFTEST_BTN_PIN) {
		OSFlagPost(&Event_Switches, EVENT_BUTTON_INTERRUPT,
		OS_OPT_POST_FLAG_SET, &err); /* post the switches interrupt status */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); /*   Check the OS error code. */
	}
	if (gpio_int_no == DEF_ADS_PIN) {
		OSFlagPost(&Event_Switches, EVENT_ADS_INTERRUPT, OS_OPT_POST_FLAG_SET,
				&err); /* post the ADS interrupt status */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); /*   Check the OS error code. */
	}
}

/**
 * @brief function to read the self test button state
 * @return current button state
 */
Button_state_t hal_switches_get_state(void) {
	return button_state; /* return the latest button pressed/released state */
}

/**
 * @brief function to read the ads switch status
 * @return current ads state
 */
Ads_state_t hal_get_ads_state(void) {
	return ads_state; /* get the current ads status */
}

/**
 * @brief function to read the latest switches pattern selected
 * @return Button_state_t latest switch pattern confirmed
 */
Button_state_t hal_switches_get_pattern(void) {
	return curr_pattern;
}

/**
 * @brief set the confirmed switches pattern selected
 * @param new pattern to be set
 * @req PTR-1386
 */
void hal_switches_set_pattern(uint8_t pattern) {
	RTOS_ERR err;
	curr_pattern = (Button_state_t) pattern; /* set the new pattern */

	if (curr_pattern > Button_Released) { /* post only when there is a valid pattern */
		OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t) EVENT_BUTTON_PRESS_0,
		OS_OPT_POST_FLAG_SET, &err); /* post the ADS off-base state */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1); /*   Check the OS error code. */
	}
}

/**
 * @brief set the button state
 * @param btn_st button state to be set
 */
void hal_switches_set_state(uint8_t btn_st)
{
    button_state = btn_st;
    if (button_state == Button_Pressed)
    {
        uint32_t button_release_time = 0u;
        /* Button pressed */
        button_release_time = RELEASE_GAP_MAX - LETimer_stop(LETIMER_SELFTEST_RELEASED);

        if (press_counter == 0u) /* not yet started monitoring the button press pattern */
        {
            /* button press pattern monitor */
            LETimer_start(LETIMER_PRESS_MONITOR_TIMER, BUTTON_PATTERN_MONITOR); /* start the button press pattern monitor timer */
            LETimer_start(LETIMER_EXT_USER_TEST, LONG_PRESS_HOLD_TIME); /* start the button press pattern monitor timer */
        }
        if (button_release_time > RELEASE_GAP_MIN)
        {
            LETimer_start(LETIMER_SELFTEST_PRESSED, PRESS_LENGTH_MAX); /* start monitoring the length of the button press */
        }
    }
    else if (button_state == Button_Released)
    {
        /* Button released */
        uint32_t button_press_time = 0u;

        if(getEndUsrTest() == true)
        {

           LEDBuzz_Post(PatternStopAll);
           GPIO_TurnAssistanceLEDoff();
           FaultHandler_Activate_Pattern();

        }
        setEndExtUsrTest(false);

        if (FaultHandler_GetFaultFlags() & FaultHandler_FaultCodeGet(TestButtonFault))
        {
            /* Clear the stuck button fault */
            DEBUG_SWITCHES("\nStuck button fault cleared", false, 0u);
            FaultHandler_FaultClear(TestButtonFault);
            /* Do not send user bist log msg to mcu2 for testbutton as requested by techem */
            DataLogging_SetLogMsgToMCU(false);
            DataLogging_SetEventLogbookRecord(DEF_LBE_USER_BIST, NULL);
            DataLogging_SetLogMsgToMCU(true);
            DataLogging_SetMinorFault(FaultTestButton, false);
        }

        button_press_time = PRESS_LENGTH_MAX - LETimer_stop(LETIMER_SELFTEST_PRESSED); /* calculate the button press duration */
        if ((button_press_time > PRESS_LENGTH_MIN) && (button_press_time < PRESS_LENGTH_MAX)) /* valid button press detected */
				{
            press_counter++;
            if (press_counter > BUTTON_PRESS_COUNTER_MAX)
            {
                /*if more than 5 presses in a row, need to wait 2 seconds before restarting counter */
                LETimer_start(LETIMER_SELFTEST_RELEASED, (RELEASE_GAP_MAX) * 2u);
            }
            else
            {
                if ((button_press_time > SHORT_PRESS_TIME) && (button_press_time <= LONG_PRESS_HOLD_TIME)) /* long press detected */
                {
                    if (press_counter <= BUTTON_PRESS_COUNTER_MAX)
                    {
                        long_press_pattern = switch_long_press;
                        LETimer_start(LETIMER_SELFTEST_RELEASED, RELEASE_GAP_MAX); /* wait until 5 seconds to observe the sequence press selected */
                    }
                    else
                    {
                        /* To void MISRA violations */
                    }
                }
                else if (button_press_time > LONG_PRESS_HOLD_TIME) /* long press and hold detected */
                {
                    if (press_counter <= BUTTON_PRESS_COUNTER_MAX)
                    {
                        long_press_pattern = switch_long_press_hold;
                        LETimer_start(LETIMER_SELFTEST_RELEASED, RELEASE_GAP_MAX); /* pattern ended ?? */

                        //hal_switches_set_pattern(Button_LongPressHold); /* Long press and hold pattern detected */
                    }
                    else
                    {
                        /* To avoid MISRA violation */
                    }
                }
                else if (button_press_time <= SHORT_PRESS_TIME)
                {
                    short_press_counter++; /* short press detected */
                    LETimer_start(LETIMER_SELFTEST_RELEASED, RELEASE_GAP_MAX); /* start the timer to detect pattern ending time */
                }
                else
                {
                      /* To avoid the MISRA violation */
                }
            }
				}
    }
}

/**
 * @brief set the ads mount state
 * @param n/a
 * @return n/a
 */
void recordMountEvent(void)
{
  if(Transport_Mode != getBehavioural_System_Modes(false))
  {
      DataLogging_SetDemountingLogbookRecord( DEF_LBE_DEMOUNTED_END ); /* Log Mounting event into Demounting */
      DataLogging_LogMountingEvent(); /* Increment Mounting event count */
      SPIComms_Send_Data_to_MCU2(SPI_CMD_demoutingLogbook);
  }

  (void)BURTCTimer_Stop(TMR_Demount_One_min_period_event_1);
  (void)BURTCTimer_Stop(TMR_Demount_too_long_event_0);

  if(getDemountOneMinFlag() == true)
  {
       setDemountOneMinFlag(false);
  }

  if(getDemountTooLongFlag() == true)
  {
        FaultHandler_FaultClear(DemountedTooLongFault);
        setDemountTooLongFlag(false);
        DataLogging_SetMinorFault(FaultDemountedTooLong, false);
  }

}

/**
 * @brief set the ads demount state
 * @param n/a
 * @return n/a
 */
void recordDemountEvent(void)
{
  if(Transport_Mode != getBehavioural_System_Modes(false))
  {
      DataLogging_SetDemountingLogbookRecord( DEF_LBE_DEMOUNTED_START ); /* Log Mounting event into Demounting */
      SPIComms_Send_Data_to_MCU2(SPI_CMD_demoutingLogbook);
  }

  if (Operational_Mode == getBehavioural_System_Modes(false))
  {
      DEBUG_SWITCHES("\nDemount too long started", false, 0u);
      BURTCTimer_Start(TMR_Demount_One_min_period_event_1, one_shot,
                       DEMOUNT_ONE_MIN_PERIOD);
      BURTCTimer_Start(TMR_Demount_too_long_event_0, one_shot,
                       DEMOUNTING_LONGTERM_PERIOD);
  }
}

/**
 * @brief set the ads state
 * @param ads_st ads state to be set
 * @req PTR-1452,
 */
void hal_set_ads_state(uint8_t ads_st) {
  RTOS_ERR err;
	ads_state = ads_st; /* set the new ads state */

	if (ads_state == Ads_offBase) {
	  OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t) EVNET_ADS_DISABLE_0, OS_OPT_POST_FLAG_SET, &err); /* post the ADS off-base state */
	  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1); /*   Check the OS error code. */
		recordDemountEvent();
		DEBUG_SWITCHES("\nADS state: OFF_Base", false, 0u);

		/* TODO: clear the ADS fault state */
	} else if (ads_state == Ads_onBase) {
	  OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t) EVNET_ADS_ENABLE_0,
	  OS_OPT_POST_FLAG_SET, &err); /* post the ADS on-base state */
	  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1); /*   Check the OS error code. */
		recordMountEvent();
		DEBUG_SWITCHES("\nADS state: ON_Base #", true, DataLogging_GetMountingEventCount());
		/* TODO: clear the ADS fault state */
	} else {
	    DEBUG_SWITCHES("\nADS fault report", false, 0u);
		/* TODO: report to fault management unit */
		/* TODO: Set the eeprom log */
		/* Indicate the LED and buzzer pattern of Major fault */
		LEDBuzz_Post(PatternMajorFault);
		/* TODO: Inform MCU2 about the change in system state */
	}
}

/**
 * @brief set status flag for end of Extender User Test
 * @param bool status
 * @return n/a
 */
void setEndExtUsrTest(bool status)
{
  endExtUsrTest = status;
}

/**
 * @brief get status flag for end of Extender User Test
 * @param n/a
 * @return bool: endExtUsrTest
 */
bool getEndUsrTest(void)
{
  return endExtUsrTest;
}

/**
 * @brief set status flag for demount too long
 * @param bool status
 * @return n/a
 */
void setDemountTooLongFlag(bool status)
{
  demountLongFlag = status;
}

/**
 * @brief get status flag for demount too long
 * @param n/a
 * @return bool: demountLongFlag
 */
bool getDemountTooLongFlag(void)
{
  return demountLongFlag;
}

/**
 * @brief set status flag for demount one min period
 * @param bool status
 * @return n/a
 */
void setDemountOneMinFlag(bool status)
{
  demountOneMinFlag = status;
}

/**
 * @brief get status flag for demount one min period
 * @param n/a
 * @return bool: demountOneMinFlag
 */
bool getDemountOneMinFlag(void)
{
  return demountOneMinFlag;
}
