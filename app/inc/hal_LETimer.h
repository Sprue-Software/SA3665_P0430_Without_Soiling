/**
 * @file  hal_LETimer.h
 * @brief handler for the low power timer
 * @project P0200 Techem Core Firmware
 * @date    21 March 2022
 * @author  abenrashed
 */
#ifndef HAL_LETIMER_H_
#define HAL_LETIMER_H_

/**
 * LETIMER REQUIREMENTS
 * Tick at 1 ms rate
 * Operate in 2 modes, blocking and non-blocking
 * Blocking needs to be a short function to block for specified ms.
 * Blocking is required by I2C, EEPROM, CO and SWITCH modules
 * Non-blocking needs to allow timer to be set with period, and when it expires trigger an action such as post flag to switch task.
 * In non-blocking mode, should be able to stop the timer before it expires and get its value at this point.
 * Non-blocking is required by SWITCH and LED/BUZZ modules.
 */

typedef enum {
	LETIMER_BLOCKING, /*Blocking timer*/
	LETIMER_DIAG_DELAY, /*delay diag routine until a gap where LED/Buzzer are in an off state, timer mode, non blocking*/
	LETIMER_SELFTEST_PRESSED, /*time button press lengths- timer mode, non blocking 1 sec */
	LETIMER_SELFTEST_RELEASED, /*time button release lengths - timer mode, non blocking 5 sec*/
	LETIMER_SELFTEST_LONG_RELEASED, /*time button release lengths - timer mode, non blocking*/
	LETIMER_LEDBUZZ, /*time pattern phases, timer, non-blocking*/
	LETIMER_LIGHTSENSOR, /*light sensor settle time after powering*/
	LETIMER_HEARTBEAT, /*heart-beat timer, non-blocking*/
	LETIMER_SPI_TIMEOUT, /*spi timeout timer, non-blocking*/
	LETIMER_PRESS_MONITOR_TIMER,/* Button press duration monitor */
	LETIMER_FAST_FLAME_MODE, /* fast flame mode timeout time */
	LETIMER_SMOKE_FASTRATE, /* smoke increased sample rate */
	LETIMER_CO_OVERLOAD_COMPENSATION,/*overload compensation  */
	LETIMER_CO_LowLevel,/* Low Level Co Warning */
	LETIMER_FMT_Smoke, /* FMT smoke timer for less than 10sec */
	LETIMER_FMT_BIST_Smoke,
	LETIMER_SPI_COMMS_TIMEOUT, /*spi timeout timer for faulty SPI for 20 min */
	LETIMER_FAILED_SPI_COMMS_TIMEOUT,
	LETIMER_EXT_USER_TEST,
	LETIMER_EXT_USER_LASER_TEST,
	LETIMER_EVENTS_SIZE /*Enum size*/
} LETimer_Events_TypeDef;

#define DIAG_DELAY_PERIOD 10u
#define OVERLOAD_COMPENSESTION  86400000u  /*Additional compensation, after 5000 PPM test for 24 Hours */
#define LO_LEVEL_CO_TIMEOUT_20_MIN  1200000  /* 20 min */
#define LO_LEVEL_CO_TIMEOUT_35_MIN  2100000u  /* 35 min */
#define LO_LEVEL_CO_TIMEOUT_4_HR    14400000u  /* 4hr */

void LETimer_delay_ms(uint32_t ms_period);
void LETimer_start(LETimer_Events_TypeDef event, uint32_t period);
uint32_t LETimer_stop(LETimer_Events_TypeDef event);
void LETimer_init(void);

#endif /* HAL_LETIMER_H_ */
