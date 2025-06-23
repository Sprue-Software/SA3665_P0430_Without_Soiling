/*
 * assistance_light.c
 *
 *  Created on: 31 Aug 2023
 *      Author: araheem
 */


#include "assistance_light.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "hal_gpio.h"
#include "v3sctrl.h"

#define ASSIST_LOG_PERIOD 3U;
static bool bAssistanceLightStatus = false;


/**************************************************************************//**
 * @brief Enable Assistance light
 *****************************************************************************/
void GPIO_TurnAssistanceLEDon(void)
{
  /* Wait for 3VS supply to become free and then turn it on. */
  /* NOTE: Supply is NOT locked so other activities can still use the 3VS.
   *       These activities must turn off the assistance light for the
   *       duration of the activity and then return it to the previous state
   *       of the light prior to the new activity. */
  V3S_ON();

  /* Now turn on home assistance light */
  GPIO_PinOutSet(DEF_ASSIST_LIGHT_PORT, DEF_ASSIST_LIGHT_RESET_PIN);

  /* Maintain status */
  SetAssistanceLightStatus( true );
}

/**************************************************************************//**
 * @brief Disable Assistance light
 *****************************************************************************/
void GPIO_TurnAssistanceLEDoff(void)
{
  /* Wait for 3VS supply to become free and then turn it off. */
  V3S_OFF( );

  /* Now turn off home assistance light */
  GPIO_PinOutClear(DEF_ASSIST_LIGHT_PORT, DEF_ASSIST_LIGHT_RESET_PIN);

  /* Maintain status */
  SetAssistanceLightStatus( false );
}

/**************************************************************************//**
 * @brief Set Assistance light Status
 *****************************************************************************/
void SetAssistanceLightStatus(bool status)
{
  bAssistanceLightStatus = status;
}

/**************************************************************************//**
 * @brief Get Assistance light Status
 *****************************************************************************/
bool GetAssistanceLightStatus(void)
{
  return bAssistanceLightStatus;
}

/**************************************************************************//**
 * @brief Get Assistance light Status
 *****************************************************************************/
uint16_t GetAssistancelogPeriod(void)
{
  static uint16_t usAssist_light_log_period = 0U;
  usAssist_light_log_period = usAssist_light_log_period + ASSIST_LOG_PERIOD;
  return usAssist_light_log_period;
}
