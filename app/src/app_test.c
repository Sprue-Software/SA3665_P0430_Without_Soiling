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
#include "P0200_FTM.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
#define DEBUG_LOGGING_EVENT (1u << 0u)

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
//ABR Techem. Each of the following needs to be moved to relevant module.
OS_TCB eventsTasktcb;

static CPU_STK eventsTaskstack[EVENTS_TASK_STK_SIZE];

extern OS_FLAG_GRP CommsEventFlags;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
static void app_bootup(void);
static void events_task(void *arg);

void sendDebugPacket(void);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
OS_FLAG_GRP Event_Flags_TopGroup;
OS_FLAG_GRP Event_Flags_SubGroup[2];
OS_FLAGS flags_0;
OS_FLAGS flags_1;

/***************************************************************************//**
 * Initialise application.
 ******************************************************************************/
void app_init(void)
{
  RTOS_ERR err;

  /* Use LPM 2 */
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);

  /* Hardware Init */
  GPIO_Init();
  SPIDriver_Init();
  Timer0_init();
  BURTC_init();
  LETimer_init();
  InterMCU_SPIComms_Init();
  LEDBuzz_Init();

  /* create flags can be left here or moved to app_bootup */
  OSFlagCreate(&Event_Flags_TopGroup, /* Pointer to user-allocated event flag. */
               "EventTopGroupFlags",  /* Name used for debugging. */
               0,                     /* Initial flags, all cleared. */
               &err);
  /* Check error code Need to change assert and trigger watchdog. */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSFlagCreate(&Event_Flags_SubGroup[0], /* Pointer to user-allocated event flag. */
               "EventSubGroup0Flags",    /* Name used for debugging. */
               0,                        /* Initial flags, all cleared. */
               &err);
  /* Check error code Need to change assert and trigger watchdog. */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSFlagCreate(&Event_Flags_SubGroup[1], /* Pointer to user-allocated event flag. */
               "EventSubGroup1Flags",    /* Name used for debugging. */
               0,                        /* Initial flags, all cleared. */
               &err);
  /* Check error code Need to change assert and trigger watchdog. */
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  /* Create Events Task for behavioural model & event system */
  /* This is starting point of the FW */
  OSTaskCreate(&eventsTasktcb,
               "Events Task",
               events_task,
               DEF_NULL,
               EVENTS_TASK_PRIO,
               &eventsTaskstack[0],
               (EVENTS_TASK_STK_SIZE / 10u),
               EVENTS_TASK_STK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR ),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/**
 * @brief Run the bootup procedures, initialising all hardware, RTOS tasks and variables
 */
static void app_bootup(void)
{
  //uint32_t reset_cause = 0u;
  //bool jmpToStoredMode = true;

  /* Init variables */
  smoke_init(); /* smoke module initialisation */

#if 0 // jmm
  reset_cause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();
  if (reset_cause & EMU_RSTCAUSE_POR)
  {
    jmpToStoredMode = false;
    DataLogging_SetResetReason(RstPOR);
  }

  if (reset_cause & EMU_RSTCAUSE_PIN)
  {
    jmpToStoredMode = false;
    DataLogging_SetResetReason(RstPIN);
  }

  if (reset_cause & EMU_RSTCAUSE_EM4)
  {
    DataLogging_SetResetReason(RstEM4);
  }

  if (reset_cause & EMU_RSTCAUSE_WDOG0)
  {
    DataLogging_SetResetReason(RstWDOG0);
  }

  if (reset_cause & EMU_RSTCAUSE_WDOG1)
  {
    DataLogging_SetResetReason(RstWDOG1);
  }

  if (reset_cause & EMU_RSTCAUSE_LOCKUP)
  {
    DataLogging_SetResetReason(RstLOCKUP);
  }

  if (reset_cause & EMU_RSTCAUSE_SYSREQ)
  {
    DataLogging_SetResetReason(RstSYSREQ);
  }

  if (reset_cause & EMU_RSTCAUSE_DVDDBOD)
  {
    DataLogging_SetResetReason(RstDVDDBOD);
  }

  if (reset_cause & EMU_RSTCAUSE_DVDDLEBOD)
  {
    DataLogging_SetResetReason(RstDVDDLEBOD);
  }

  if (reset_cause & EMU_RSTCAUSE_DECBOD)
  {
    DataLogging_SetResetReason(RstDECBOD);
  }

  if (reset_cause & EMU_RSTCAUSE_AVDDBOD)
  {
    DataLogging_SetResetReason(RstAVDDBOD);
  }

  if (reset_cause & EMU_RSTCAUSE_IOVDD0BOD)
  {
    DataLogging_SetResetReason(RstIOVDDBOD);
  }

  DEBUG_APP("\nReset reason:", true, reset_cause);
  DEBUG_APP("\nPIN Rst Cnt:", true, DataLogging_GetResetReasonCount(RstPIN));
  DEBUG_APP("\nPOR Rst Cnt:", true, DataLogging_GetResetReasonCount(RstPOR));
  DEBUG_APP("\nWDT Rst Cnt:", true, DataLogging_GetResetReasonCount(RstWDOG0));

  setBehavioural_System_Modes(getBehavioural_System_Modes(jmpToStoredMode));
#endif
}

/**
 * @brief    The system task is responsible for starting all other tasks in the system, and then
 *           monitoring the state of the system to ensure all tasks are behaving as expected.
 * @param    p_arg   Pointer to an optional data area which can pass parameters to the task when the
 *                   task executes. Unused here.
 * @req PTR-1401,
 */
static void events_task(void *arg)
{
  (void) &arg; /* Unused parameters */

  RTOS_ERR err;

  commsAppInit();   /* init Debug Messages. */
  hal_AFE_init();

  /* Initialise sensors. */
  app_bootup();

  //DEBUG_APP("\n<<<<< Single Task Smoke demo >>>>>", false, 0);
  DEBUG_APP("\nFwRevMajor ", true, 0);
  DEBUG_APP("\nFwRevMinor ", true, 0);
  DEBUG_APP("\nFwRevBuild ", true, 7);
  DEBUG_APP("\nTest for Major Fault ", false, 0u);

  BURTCTimer_Start(TMR_Smoke_measure_event_0, periodical, SMOKE_MEASUREMENT);

  setBehavioural_System_Modes(Operational_Mode);

  while (true)
  {
    //ABR. Will add TopGroup checking later.
    flags_0 = OSFlagPend(&Event_Flags_SubGroup[0],  /* Pointer to user-allocated event flag. */
                         0xffffffffu,               /* Flag bitmask to match. */
                         100u,                      /* Wait 100 ticks. */
                         OS_OPT_PEND_FLAG_SET_ANY | /* Wait until ANY flags are set and */
                         OS_OPT_PEND_BLOCKING |     /* task will block and */
                         OS_OPT_PEND_FLAG_CONSUME,  /* consume flags */
                         DEF_NULL,                  /* Timestamp is not used. */
                         &err);
    /* Check error code. */
    if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)
    {
      if ((flags_0 & FLAGS_BIT_INDEX(TMR_Smoke_measure_event_0)) != 0u)
      {
        smoke_periodic_measurement();
      }

      if ((flags_0 & (uint32_t) FLAGS_BIT_INDEX(TMR_MODE_Change_event_0)) != 0u) {

      }
    }

    flags_0 = OSFlagPend(&CommsEventFlags,          /* Pointer to user-allocated event flag. */
                         DEBUG_LOGGING_EVENT,       /* Flag bitmask to match. */
                         100u,                      /* Wait 100 ticks. */
                         OS_OPT_PEND_FLAG_SET_ANY | /* Wait until ANY flags are set and */
                         OS_OPT_PEND_BLOCKING |     /* task will block and */
                         OS_OPT_PEND_FLAG_CONSUME,  /* consume flags */
                         NULL,                      /* Timestamp is not used. */
                         &err);
    /* Check error code. */
    if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)
    {
      if ((flags_0 & DEBUG_LOGGING_EVENT) == DEBUG_LOGGING_EVENT)
      {
        sendDebugPacket();
      }
    }
  }

}
