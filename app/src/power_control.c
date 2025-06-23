/*******************************************************************************
 *
 * @file    soiling.h
 *
 * @brief   Soiling header file
 *
 * @date    15 Aug 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "power_control.h"
#include "app.h"
#include "spi_comms.h"
#include "os.h"
#include "em_emu.h"
#include "sl_power_manager.h"
#include "hal_BURTCTimer.h"
#include "led_buzzer.h"
#include "em_letimer.h"

static void disable_interrupts( void )
{
  static const uint8_t interrupts[ ] = \
  {
    SMU_SECURE_IRQn,
    SMU_PRIVILEGED_IRQn,
    SMU_NS_PRIVILEGED_IRQn,
    EMU_IRQn,
    TIMER0_IRQn,
    TIMER1_IRQn,
    TIMER2_IRQn,
    TIMER3_IRQn,
    TIMER4_IRQn,
    USART0_RX_IRQn,
    USART0_TX_IRQn,
    EUSART0_RX_IRQn,
    EUSART0_TX_IRQn,
    EUSART1_RX_IRQn,
    EUSART1_TX_IRQn,
    EUSART2_RX_IRQn,
    EUSART2_TX_IRQn,
    ICACHE0_IRQn,
    BURTC_IRQn,
    LETIMER0_IRQn,
    SYSCFG_IRQn,
    MPAHBRAM_IRQn,
    LDMA_IRQn,
    LFXO_IRQn,
    LFRCO_IRQn,
    ULFRCO_IRQn,
    GPIO_ODD_IRQn,
    GPIO_EVEN_IRQn,
    I2C0_IRQn,
    I2C1_IRQn,
    EMUDG_IRQn,
    HOSTMAILBOX_IRQn,
    ACMP0_IRQn,
    ACMP1_IRQn,
    WDOG0_IRQn,
    WDOG1_IRQn,
    HFXO0_IRQn,
    HFRCO0_IRQn,
    HFRCOEM23_IRQn,
    CMU_IRQn,
    IADC_IRQn,
    MSC_IRQn,
    DPLL0_IRQn,
    EMUEFP_IRQn,
    DCDC_IRQn,
    VDAC_IRQn,
    PCNT0_IRQn,
    SW0_IRQn,
    SW1_IRQn,
    SW2_IRQn,
    SW3_IRQn,
    KERNEL0_IRQn,
    KERNEL1_IRQn,
    M33CTI0_IRQn,
    M33CTI1_IRQn,
    FPUEXH_IRQn,
    SETAMPERHOST_IRQn,
    SEMBRX_IRQn,
    SEMBTX_IRQn,
    LESENSE_IRQn,
    SYSRTC_APP_IRQn,
    SYSRTC_SEQ_IRQn,
    LCD_IRQn,
    KEYSCAN_IRQn
  };

  __disable_irq( );

  for( int i = 0; i < sizeof( interrupts ); i++ )
  {
    const IRQn_Type irq = ( IRQn_Type )interrupts[ i ];

    NVIC_DisableIRQ( irq );

    NVIC_ClearPendingIRQ( irq );
  }
}

void pc_enter_em0_and_stop( void )
{
	DEBUG_APP("EM0 request rejected", false, 0u);
}

void pc_enter_em1_and_stop( void )
{
	DEBUG_APP("EM1 request rejected", false, 0u);
}

void pc_enter_em2_and_stop( void )
{
	RTOS_ERR err;
	
	OSSchedLock( &err );

	/* disable the BURTC related function, MCU1 will be in EM2 until pin reset */

	//BURTC_Enable(false); /* disable the BURTC timer */

	LETIMER_Enable(LETIMER0, false);

	/*Disable Watchdog*/
	//WDOGn_Enable(WDOG0, false);
	/*stop all tasks*/
#if defined FTM_BUILD || defined DEBUG_BUILD
	OSTaskSuspend(&CommsTaskTCB, &err);
#endif
	OSTaskSuspend(&eventsTasktcb, &err);
	//OSTaskSuspend(&wdogTimerTasktcb, &err);
	LEDBuzz_ShutDown();
	OSTaskSuspend(&InterMCUSPIComms_TaskTCB, &err);
	
	BURTCTimer_StopFrom(0); /* stop all the backup timer based events */
	BURTCTimer_Start(WdogTimer_event_1, false, WDOGTIMER_EVENT_PERIOD_TWO_MIN);

	/*Go to sleep in em2 mode*/
	sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
	sl_power_manager_sleep();

}
