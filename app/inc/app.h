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

#ifndef APP_H
#define APP_H

#include "debug.h"
#include "os.h"
#include "comms_handler.h"

#define TOOGLE_DELAY_MS            (60000)

#define FW_SA_NUM_1                (0x0E)
#define FW_SA_NUM_2                (0x51)
#define FW_MAJOR_REV               (0U)
#define FW_MINOR_REV               (1U)
#define FW_BUILD_REV               (0U)

/*
 *********************************************************************************************************
 *                                      APPLICATION TASK PRIORITIES (Round Robin Scheduler is not enabled, so all priorities should be different)
 *********************************************************************************************************
 */
#define AFE_TASK_PRIO               (4u)		
#define LED_BUZZ_TASK_PRIO          (6u)
#define EVENTS_TASK_PRIO            (7u)
#define SPI_COMMS_TASK_PRIO         (12u)
#define SWITCH_TASK_PRIO            (13u)
#define UART_COMMS_TASK_PRIO        (14u)
#define UART_CLI_TASK_PRIO          (15u)
#define WDOG_TIMER_TASK_PRIO        (16u)

/*
 *********************************************************************************************************
 *                                      APPLICATION TASK STACK SIZES
 *********************************************************************************************************
 */
#define LED_BUZZ_TASK_STK_SIZE      (700u)
#define EVENTS_TASK_STK_SIZE        (800u)
#define SPI_COMMS_TASK_STK_SIZE     (400u)  /* re-run  static Analysis*/
#define SWITCH_TASK_STK_SIZE        (800u)
#define UART_COMMS_TASK_STK_SIZE    (800u)
#define UART_CLI_TASK_STK_SIZE      (400u)
#define WDOG_TIMER_TASK_STK_SIZE    (256u)
#define AFE_TASK_STK_SIZE			      (600u)
//ABR. All the following is to be moved to relevant task modules
//BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB

//ABR. To be updated as per the project switch requirements
#define SWITCH_SELFTEST_INTERRUPT       (1u << 1u)
#define SWITCH_ADS_INTERRUPT            (1u << 2u)
#define SWITCH_ADS_POLL                 (1u << 3u)
#define SWITCH_SELFTEST_RELEASE_TIMEOUT (1u << 4u)
#define SWITCH_SELFTEST_PRESS_TIMEOUT   (1u << 5u)
#define SWITCH_LONG_RELEASE_TIMEOUT     (1u << 6u)
#define SWITCH_PATTERN_TIMEOUT			(1u << 7u)
#define SWITCH_EXT_USER_TEST            (1u << 8u)
extern OS_TCB eventsTasktcb;
extern OS_TCB wdogTimerTasktcb;
extern OS_TCB ledBuzzTasktcb;
extern OS_TCB CommsTaskTCB;

typedef enum {
	WDOG_task0_flag, /*task0 running flag*/
	WDOG_task1_flag, /*task1 running flag*/
	WDOG_task2_flag, /*task2 running flag*/
	WDOG_task3_flag, /*task3 running flag*/
	WDOG_task4_flag, /*task4 running flag*/
	WDOG_TASK_FLAGS_SIZE /*Enum size*/
} WDOGTimer_Flags_TypeDef;


/***************************************************************************//**
 * Initialise application.
 ******************************************************************************/
void app_init(void);
void wdogTimer_flagSet(WDOGTimer_Flags_TypeDef flag);

#endif  // APP_H
