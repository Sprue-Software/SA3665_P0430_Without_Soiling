/**
 * @file  hal_Timer0.h
 * @brief handler for the PWM timer
 * @project P0200 Techem Core Firmware
 * @date    11 April 2022
 * @author  abenrashed
 */
#ifndef HAL_TIMER0_H_
#define HAL_TIMER0_H_

// Global variables used to set top value and duty cycle of the timer for testing
#define PWM_FREQ          1000
#define DUTY_CYCLE_STEPS  0.3

extern void Timer0_init(void);
extern void PWM_Timer0_Start(uint32_t frequency, float dutyCycle);
extern void PWM_Timer0_Stop(void);

#endif /* HAL_TIMER0_H_ */
