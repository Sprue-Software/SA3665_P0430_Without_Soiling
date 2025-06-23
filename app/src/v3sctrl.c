/*******************************************************************************
 *
 * @file    v3sctrl.c
 *
 * @brief   3VS control file
 *
 * @date    15 Sept 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "v3sctrl.h"
#include "em_gpio.h"
#include "hal_gpio.h"

#include "os.h"

/**
 * @brief 3VS Mutex
 */
static OS_MUTEX mutex;

/******************************************************************************/
void v3s_ctrl_init( void )
{
  RTOS_ERR err;

  OSMutexCreate( &mutex, V3S_CTRL_MUTEX_NAME, &err );
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

  GPIO_PinModeSet( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN, gpioModePushPull, 0u );
}

/******************************************************************************/
bool v3s_ctrl_is_on( void )
{
  return( GPIO_PinOutGet( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN ) != 0UL );
}

/******************************************************************************/
bool v3s_ctrl_on_off( const bool on, const bool lock )
{
  RTOS_ERR err;

  const unsigned int previous_state = GPIO_PinOutGet( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );

  const bool in_isr = CORE_InIrqContext( );

  /* It's illegal to lock supply from an ISR */
  if( !( in_isr && lock ) )
  {
    /* Cannot do this in an ISR */
    if( in_isr == false )
    {
      OSMutexPend( &mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
    }

    if( on )
    {
      GPIO_PinOutSet( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );
    }
    else
    {
      GPIO_PinOutClear( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );
    }

    /* Cannot do this in an ISR */
    if( in_isr == false )
    {
      /* When not locking supply voltage, or turning supply voltage off, always
      * release the mutex. Mutex can only be locked when turning the supply on. */
      if( ( lock == false ) || ( on == false ) )
      {
        OSMutexPost( &mutex, OS_OPT_POST_NONE, &err);
        APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
      }
    }
  }

  return(  previous_state != 0UL ? true : false  );
}

/******************************************************************************/
void v3s_ctrl_on_unlock( const bool on )
{
  RTOS_ERR err;

  if( on )
  {
    GPIO_PinOutSet( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );
  }
  else
  {
    GPIO_PinOutClear( DEF_3VS_POWER_PORT, DEF_3VS_POWER_PIN );
  }

  const bool in_isr = CORE_InIrqContext( );

  /* Cannot do this in an ISR */
  if( in_isr == false )
  {
    OSMutexPost( &mutex, OS_OPT_POST_NONE, &err);
    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
  }
}
