/***************************************************************************//**
 * @file  hal_Timer0.c
 * @brief handler for the PWM timer
 * @project P0200 Techem Core Firmware
 * @date    11 April 2022
 * @author  abenrashed
 *******************************************************************************/

/***************************************************************************//**
 * @file main.c
 * @brief This project demonstrates pulse width modulation using the TIMER
 * module. The GPIO pin specified in the readme.txt is configured for output and
 * outputs a 1kHz, 30% duty cycle signal. The duty cycle can be adjusted by
 * writing to the CCVB or changing the global dutyCycle variable.
 *******************************************************************************/

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "sl_power_manager.h"
#include "em_timer.h"
#include "hal_Timer0.h"
#include "board.h"
#ifdef EVAL_BOARD
#include "efm32pg23b310f512im48.h"
#else
#include "efm32pg23b210f256im48.h"
#endif
#include "hal_gpio.h"

static uint32_t topValue = 0;
static volatile float dutyCycle = 0;
static bool Timer0On = false;
#define TIMER0_INIT                                                                   \
  {                                                                                   \
    false,                /* Enable timer when initialization completes. */           \
    false,                /* Stop counter during debug halt. */                       \
    timerPrescale64,      /* Divide by 64. */                                         \
    timerClkSelHFPerClk,  /* Select HFPER / HFPERB clock. */                          \
    false,                /* Not 2x count mode. */                                    \
    false,                /* No ATI. */                                               \
    timerInputActionNone, /* No action on falling input edge. */                      \
    timerInputActionNone, /* No action on rising input edge. */                       \
    timerModeUp,          /* Up-counting. */                                          \
    false,                /* Do not clear DMA requests when DMA channel is active. */ \
    false,                /* Select X2 quadrature decode mode (if used). */           \
    false,                /* Disable one shot. */                                     \
    false                 /* Not started/stopped/reloaded by other timers. */         \
  }

#define TIMER0_INITCC                                                        \
  {                                                                          \
    timerEventEveryEdge,    /* Event on every capture. */                    \
    timerEdgeRising,        /* Input capture edge on rising edge. */         \
    0,                      /* Not used by default, select PRS channel 0. */ \
    timerOutputActionNone,  /* No action on underflow. */                    \
    timerOutputActionNone,  /* No action on overflow. */                     \
    timerOutputActionNone,  /* No action on match. */                        \
    timerCCModePWM,         /* Disable compare/capture channel. */           \
    false,                  /* Disable filter. */                            \
    false,                  /* No PRS input. */                              \
    false,                  /* Clear output when counter disabled. */        \
    false,                  /* Do not invert output. */                      \
    timerPrsOutputDefault,  /* Use default PRS output configuration. */      \
    timerPrsInputNone       /* No PRS input, so input type is none. */       \
  }

/**************************************************************************//**
 * @brief
 *    Interrupt handler for TIMER0 that changes the duty cycle
 *
 * @note
 *    This handler doesn't actually dynamically change the duty cycle. Instead,
 *    it acts as a template for doing so. Simply change the dutyCycle
 *    global variable here to dynamically change the duty cycle.
 *****************************************************************************/
void TIMER0_IRQHandler(void) {
//ABR Interrupt not enabled, so ISR is not run. If we find from req. we need to
	//take some action, we will use it
	// Acknowledge the interrupt
	uint32_t flags = TIMER_IntGet(TIMER0);
	TIMER_IntClear(TIMER0, flags);

	// Update CCVB to alter duty cycle starting next period
	TIMER_CompareBufSet(TIMER0, 0, (uint32_t) (topValue * dutyCycle));
}

/**************************************************************************//**
 * @brief
 *    TIMER0 initialisation
 *****************************************************************************/
extern void Timer0_init(void) {
	TIMER_Init_TypeDef timerInit = TIMER0_INIT;  //TIMER_INIT_DEFAULT;
	// Configure TIMER0 Compare/Capture for output compare
	TIMER_InitCC_TypeDef timerCCInit = TIMER0_INITCC;

	CMU_ClockEnable(cmuClock_TIMER0, true);

	// Configure but do not start the timer
	TIMER_Init(TIMER0, &timerInit);

	// Configure CC Channel 0
	TIMER_InitCC(TIMER0, 0, &timerCCInit);

	TIMER0->EN_CLR = TIMER_EN_EN;

	CMU_ClockEnable(cmuClock_TIMER0, false);
}

/**************************************************************************//**
 * @brief
 *    TIMER0 PWM start
 *****************************************************************************/
extern void PWM_Timer0_Start(uint32_t frequency, float dutyCycle) {
	uint32_t timerFreq = 0;

	TIMER_Init_TypeDef timerInit = TIMER0_INIT;
	TIMER_InitCC_TypeDef timerCCInit = TIMER0_INITCC;

	CMU_ClockEnable(cmuClock_TIMER0, true);

	TIMER_Init(TIMER0, &timerInit);
	TIMER_InitCC(TIMER0, 0, &timerCCInit);

	// Use PWM mode, which sets output on overflow and clears on compare events

	// set PWM period
	timerFreq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timerInit.prescale + 1);
	topValue = (timerFreq / frequency);

	GPIO->TIMERROUTE[0].ROUTEEN |= GPIO_TIMER_ROUTEEN_CC0PEN;
	GPIO->TIMERROUTE[0].CC0ROUTE |= (DEF_AFE_ENABLE_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
			| (DEF_AFE_ENABLE_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
	// Set top value to overflow at the desired PWM_FREQ frequency
	TIMER_TopSet(TIMER0, topValue);

	// Set compare value for initial duty cycle
	TIMER_CompareSet(TIMER0, 0, (uint32_t) (topValue * dutyCycle));

	sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
	// Start the timer
	TIMER_Enable(TIMER0, true);
	Timer0On = true;
}

/**************************************************************************//**
 * @brief
 *    TIMER0 PWM start
 *****************************************************************************/
extern void PWM_Timer0_Stop(void) {

	if (true == Timer0On)
	{
		Timer0On = false;
		// Start the timer
		TIMER_Enable(TIMER0, false);
		TIMER0->EN_CLR = TIMER_EN_EN;
		CMU_ClockEnable(cmuClock_TIMER0, false);

		// Remove Route Timer0 CC0 output to PA8
		GPIO->TIMERROUTE[0].ROUTEEN &= ~GPIO_TIMER_ROUTEEN_CC0PEN;
		sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
	}
}
