/* em_wdog.c
 *
 *  Created on: 11 Apr 2022
 *      Author: abenrashed
 */
/***************************************************************************//**
 * @file
 * @brief Watchdog (WDOG) peripheral API
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 ******************************************************************************/

#include "em_wdog.h"
#if defined(WDOG_COUNT) && (WDOG_COUNT > 0)

#include "em_bus.h"
#include "em_core.h"

/***************************************************************************//**
 * @addtogroup wdog WDOG - Watchdog
 * @brief Watchdog (WDOG) Peripheral API
 * @details
 *  This module contains functions to control the WDOG peripheral of Silicon
 *  Labs 32-bit MCUs and SoCs. The WDOG resets the system in case of a fault
 *  condition.
 * @{
 ******************************************************************************/

/** In some scenarioes when the watchdog is disabled the synchronization
 * register might be set and not be cleared until the watchdog is enabled
 * again. This will happen when for instance some watchdog register is modified
 * while the watchdog clock is disabled. In these scenarioes we need to make
 * sure that the software does not wait forever. */
#define WDOG_SYNC_TIMEOUT  30000

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Enable/disable the watchdog timer.
 *
 * @note
 *   This function modifies the WDOG CTRL register which requires
 *   synchronisation into the low-frequency domain. If this register is modified
 *   before a previous update to the same register has completed, this function
 *   will stall until the previous synchronisation has completed.
 *
 * @param[in] wdog
 *   A pointer to the WDOG peripheral register block.
 *
 * @param[in] enable
 *   True to enable Watchdog, false to disable. Watchdog cannot be disabled if
 *   it's been locked.
 ******************************************************************************/
void WDOGn_Enable(WDOG_TypeDef *wdog, bool enable) {
	// SYNCBUSY may stall when locked.
#if defined(_WDOG_STATUS_MASK)
	if ((wdog->STATUS & _WDOG_STATUS_LOCK_MASK) == WDOG_STATUS_LOCK_LOCKED) {
		return;
	}
#else
  if (wdog->CTRL & WDOG_CTRL_LOCK) {
    return;
  }
#endif

#if defined(_WDOG_EN_MASK)
	if (!enable) {
		while (wdog->SYNCBUSY & WDOG_SYNCBUSY_CMD) {
		}
		wdog->EN_CLR = WDOG_EN_EN;
#if defined(_WDOG_EN_DISABLING_MASK)
		while (wdog->EN & _WDOG_EN_DISABLING_MASK) {
		}
#endif
	}
	else {
		wdog->EN_SET = WDOG_EN_EN;
	}
#else
  // Wait for previous operations/modifications to complete
  int i = 0;
  while (((wdog->SYNCBUSY & WDOG_SYNCBUSY_CTRL) != 0U)
         && (i < WDOG_SYNC_TIMEOUT)) {
    i++;
  }

  bool wdogState = ((wdog->CTRL & _WDOG_CTRL_EN_MASK) != 0U);

  // Make sure to only write to the CTRL register if we are changing mode
  if (wdogState != enable) {
    BUS_RegBitWrite(&wdog->CTRL, _WDOG_CTRL_EN_SHIFT, enable);
  }
#endif
}

/***************************************************************************//**
 * @brief
 *   Feed WDOG.
 *
 * @details
 *   When WDOG is activated, it must be fed (i.e., clearing the counter)
 *   before it reaches the defined timeout period. Otherwise, WDOG
 *   will generate a reset.
 *
 * @note
 *   Note that WDOG is an asynchronous peripheral and when calling the
 *   WDOGn_Feed() function the hardware starts the process of clearing the
 *   counter. This process takes some time before it completes depending on the
 *   selected oscillator (up to 4 peripheral clock cycles). When using the
 *   ULFRCO for instance as the oscillator the watchdog runs on a 1 kHz clock
 *   and a watchdog clear operation might take up to 4 ms.
 *
 *   If the device enters EM2 or EM3 while a command is in progress then that
 *   command will be aborted. An application can use @ref WDOGn_SyncWait()
 *   to wait for a command to complete.
 *
 * @param[in] wdog
 *   A pointer to the WDOG peripheral register block.
 ******************************************************************************/
void WDOGn_Feed(WDOG_TypeDef *wdog) {
	// Series 2 devices
	CORE_DECLARE_IRQ_STATE;

	// WDOG should not be fed while it is disabled.
	if ((wdog->EN & WDOG_EN_EN) == 0U) {
		return;
	}

	// We need an atomic section around the check for sync and the clear command
	// because sending a clear command while a previous command is being synchronised
	// will cause a BusFault.
	CORE_ENTER_ATOMIC();
	if ((wdog->SYNCBUSY & WDOG_SYNCBUSY_CMD) == 0U) {
		wdog->CMD = WDOG_CMD_CLEAR;
	}
	CORE_EXIT_ATOMIC();
}

/***************************************************************************//**
 * @brief
 *   Initialise WDOG (assuming the WDOG configuration has not been
 *   locked).
 *
 * @note
 *   This function modifies the WDOG CTRL register which requires
 *   synchronisation into the low-frequency domain. If this register is modified
 *   before a previous update to the same register has completed, this function
 *   will stall until the previous synchronisation has completed.
 *
 * @param[in] wdog
 *   Pointer to the WDOG peripheral register block.
 *
 * @param[in] init
 *   The structure holding the WDOG configuration. A default setting
 *   #WDOG_INIT_DEFAULT is available for initialisation.
 ******************************************************************************/
void WDOGn_Init(WDOG_TypeDef *wdog, const WDOG_Init_TypeDef *init) {
	// Handle series-2 devices
	if (wdog->EN != 0U) {
		while (wdog->SYNCBUSY != 0U) {
			// Wait for any potential synchronisation to finish
		}
		wdog->EN_CLR = WDOG_EN_EN;
#if defined(_WDOG_EN_DISABLING_MASK)
		while (wdog->EN & _WDOG_EN_DISABLING_MASK) {
			/* Wait for disabling to finish */
		}
#endif
	}

	wdog->CFG = (init->debugRun ? WDOG_CFG_DEBUGRUN : 0U) | (init->em2Run ? WDOG_CFG_EM2RUN : 0U)
			| (init->em3Run ? WDOG_CFG_EM3RUN : 0U) | (init->em4Block ? WDOG_CFG_EM4BLOCK : 0U)
			| (init->resetDisable ? WDOG_CFG_WDOGRSTDIS : 0U)
			| ((uint32_t) (init->warnSel) << _WDOG_CFG_WARNSEL_SHIFT)
			| ((uint32_t) (init->winSel) << _WDOG_CFG_WINSEL_SHIFT)
			| ((uint32_t) (init->perSel) << _WDOG_CFG_PERSEL_SHIFT);

	WDOGn_Enable(wdog, init->enable);

	if (init->lock) {
		WDOGn_Lock(wdog);
	}

}

/***************************************************************************//**
 * @brief
 *   Lock the WDOG configuration.
 *
 * @details
 *   This prevents errors from overwriting the WDOG configuration, possibly
 *   disabling it. Only a reset can unlock the WDOG configuration once locked.
 *
 *   If the LFRCO or LFXO clocks are used to clock WDOG,
 *   consider using the option of inhibiting those clocks to be disabled.
 *   See the WDOG_Enable() initialization structure.
 *
 * @note
 *   This function modifies the WDOG CTRL register which requires
 *   synchronisation into the low-frequency domain. If this register is modified
 *   before a previous update to the same register has completed, this function
 *   will stall until the previous synchronisation has completed.
 *
 * @param[in] wdog
 *   A pointer to WDOG peripheral register block.
 ******************************************************************************/
void WDOGn_Lock(WDOG_TypeDef *wdog) {
#if defined(_WDOG_LOCK_MASK)
	wdog->LOCK = _WDOG_LOCK_LOCKKEY_LOCK;
#else
  // Wait for any pending previous write operation to have been completed in
  // the low-frequency domain.
  while ( (wdog->SYNCBUSY & WDOG_SYNCBUSY_CTRL) != 0U ) {
  }

  // Disable writing to the control register.
  BUS_RegBitWrite(&wdog->CTRL, _WDOG_CTRL_LOCK_SHIFT, 1);
#endif
}

/***************************************************************************//**
 * @brief
 *   Wait for the WDOG to complete all synchronization of register changes
 *   and commands.
 *
 * @param[in] wdog
 *   A pointer to WDOG peripheral register block.
 ******************************************************************************/
void WDOGn_SyncWait(WDOG_TypeDef *wdog) {
#if defined(_SILICON_LABS_32B_SERIES_2)
	while ((wdog->EN != 0U) && (wdog->SYNCBUSY != 0U)) {
		// Wait for synchronisation to finish
	}
#else
  while (wdog->SYNCBUSY != 0U) {
    // Wait for synchronisation to finish
  }
#endif
}

/***************************************************************************//**
 * @brief
 *   Unlock the WDOG configuration.
 *
 * @details
 *   Note that this function will have no effect on devices where a reset is
 *   the only way to unlock the watchdog.
 *
 * @param[in] wdog
 *   A pointer to WDOG peripheral register block.
 ******************************************************************************/
void WDOGn_Unlock(WDOG_TypeDef *wdog) {
#if defined(_WDOG_LOCK_MASK)
	wdog->LOCK = _WDOG_LOCK_LOCKKEY_UNLOCK;
#else
  (void) wdog;
#endif
}

#endif /* defined(WDOG_COUNT) && (WDOG_COUNT > 0) */
