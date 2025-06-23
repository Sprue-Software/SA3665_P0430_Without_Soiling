/*******************************************************************************
 *
 * @file    ctune.c
 *
 * @brief   User info page file
 *
 * @date    19 Feb 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include "em_msc.h"
#include "em_cmu.h"
#include "cmsis_gcc.h"

#include "debug.h"
#include "ctune.h"
#include "user_info_page.h"

/******************************************************************************/
bool ct_init( void )
{
	uint32_t value;

	bool ok = ct_get_tuning_value( &value );

	if( ok )
	{
  	ok = ( ( value >= CTUNE_MIN ) && ( value <= CTUNE_MAX ) );

		if( ok )
		{
			ok = ct_tune( value );
		}
	}

	return( ok );
}

/******************************************************************************/
bool ct_get_tuning_value( uint32_t * const value )
{
  const bool ok = uip_read( CT_TUNING_FLASH_LOCATION, value, sizeof( value ) );

  return( ok );
}

/******************************************************************************/
bool ct_set_tuning_value( const uint32_t value )
{
  const bool ok = uip_program( CT_TUNING_FLASH_LOCATION, &value, sizeof( value ) );

  return( ok );
}

/******************************************************************************/
bool ct_tune( const uint32_t value )
{
  const bool ok = ( ( value >= CTUNE_MIN ) && ( value <= CTUNE_MAX ) );

	if( ok )
	{
		CMU_LFXOInit_TypeDef lfxoInit = CMU_LFXOINIT_DEFAULT;

		lfxoInit.capTune = ( uint8_t )( value );    // Initialise the LFXO with new parameter

		CMU_LFXOInit( &lfxoInit );

		LFXO->CTRL |= LFXO_CTRL_FORCEEN;            // forcefully enable the LFXO

		while( ( LFXO->STATUS & LFXO_STATUS_RDY ) != LFXO_STATUS_RDY )
		{
			__NOP( );   // wait for the oscillator to enable and be ready
		}

		DEBUG_APP( "\nctune:", true, value );
	}

  return( ok );
}

/******************************************************************************/
