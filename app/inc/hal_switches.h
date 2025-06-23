/*
 * hal_switches.h
 *
 *  Created on: 14 Jun 2022
 *      Author: uhegde
 */

#ifndef APP_INC_HAL_SWITCHES_H_
#define APP_INC_HAL_SWITCHES_H_

#include "debug.h"
#include "os.h"
#include "em_gpio.h"

#define EVENT_BUTTON_INTERRUPT	(1u << 0u)   /* Button pressed interrupt triggered */
#define EVENT_BUTTON_PRESSED	  (1u << 1u)   /* Button pressed interrupt triggered */
#define EVENT_BUTTON_RELEASED 	(1u << 2u)   /* Button released interrupt triggered */
#define EVENT_ADS_INTERRUPT		  (1u << 3u)   /* ADS switch interrupt triggered */
#define EVENT_ADS_ONBASE		    (1u << 4u)	/* ADS on base/pressed event triggered */
#define EVENT_ADS_OFFBASE		    (1u << 5u)	/* ADS off base/ released event triggered */

#define PRESS_LENGTH_MIN 		    (50u)
#define PRESS_LENGTH_MAX 		    (60000u)  /* 60 seconds of button press is considered as button stuck condition */
#define RELEASE_GAP_MIN 		    (25u)
#define RELEASE_GAP_MAX 		    (1000u)
#define SHORT_PRESS_TIME		    (1000u) /* short press timeout time is 1 second */
#define LONG_PRESS_HOLD_TIME	  (3000u) /* Long press time */
#define EXTENDED_BIST_LASER_TIME  (5000u) /* Extended BIST with Laser */
#define BUTTON_PATTERN_MONITOR	(5000u) /* maximum time button patterns are monitored */
#define BUTTON_PRESS_COUNTER_MAX (5u)  /* maximum valid button press */

/**
 * ADS switch states
 */
typedef enum {
	Ads_offBase, /* ADS switch OFF base */
	Ads_onBase, /* ADS switch ON base */	
	Ads_fault, /* ADS switch is faulty */
} Ads_state_t;

typedef enum {
	no_press, single_press, /* single button press */
	double_press, /* double button press */
	five_press = 5u, /* five times button presses */
} Button_press_count_t;

/**
 * State options of SelfTest switch
 */
typedef enum {
	Button_Pressed = 0u, /* button pressed */
	Button_Released, /* Self test button not pressed or in released state */
	Button_SingleShortPress, /* self test button single short press pattern */
	Button_DoubleShortPress, /* self test button double short press pattern */
	Button_FiveShortPress, /* self test button five short press pattern */
	Button_LongPress, /* self test button long press pattern */
	Button_LongPressHold, /* self test long press and hold */
	Button_Stuck, /* self test button stuck state */
	Button_userExtTest,
	Button_userExtndTestLaser,
} Button_state_t;

typedef enum {
	switch_no_long_press, /* long press not detected */
	switch_long_press, /* user test pattern */
	switch_long_press_hold, /* extended user BIST */
} switch_long_press_t;

extern OS_FLAG_GRP Event_Switches;
extern uint8_t press_counter; /* counter to monitor the number of button presses */
extern uint8_t short_press_counter; /* number of short press identified */
extern Button_state_t curr_pattern;
extern switch_long_press_t long_press_pattern;

void hal_switches_init(void);
uint8_t hal_switches_get_state(void);
uint8_t hal_get_ads_state(void);
void hal_switches_set_state(uint8_t new_state);
void hal_set_ads_state(uint8_t new_state);
Button_state_t hal_switches_get_pattern(void);
void hal_switches_set_pattern(uint8_t pattern);
void hal_switches_callback(uint8_t intNo);
void recordMountEvent(void);
void recordDemountEvent(void);
void setEndExtUsrTest(bool status);
bool getEndUsrTest(void);
void setDemountTooLongFlag(bool status);
bool getDemountTooLongFlag(void);
void setDemountOneMinFlag(bool status);
bool getDemountOneMinFlag(void);


#endif /* APP_INC_HAL_SWITCHES_H_ */
