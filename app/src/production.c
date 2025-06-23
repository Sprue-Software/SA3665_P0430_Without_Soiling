/*******************************************************************************
 *
 * @file    production.c
 *
 * @brief   Production file
 *
 * @date    23 Mar 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include "production.h"
#include "data_logging.h"

#include "debug.h"


static uint32_t prodComp;

/**
 * @brief Set to true if production complete
 */



/******************************************************************************/
bool prod_set_value( uint32_t prod_comp )
{
  bool retVal = false;
	retVal = dl_set_production_complete( prod_comp );
  prodComp = prod_comp;
	DataLogging_SetCRC( );
	return retVal;

}



/******************************************************************************/
void prod_reset( void )
{
	dl_set_production_complete( 0xFFFFFFFF );
  prodComp = 0xFFFFFFFFU;
	DataLogging_SetCRC( );

}

/******************************************************************************/
uint32_t prod_get_value( void )
{
	return prodComp;
}

/******************************************************************************/
void prod_value_init(void)
{
  prodComp = dl_get_production_complete();
}
