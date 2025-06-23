/*******************************************************************************
 *
 * @file    production.h
 *
 * @brief   Production header file
 *
 * @date    23 Mar 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef PRODUCTION_H
#define PRODUCTION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h> 

/**
 * @brief Production complete magic number
 * 
 * @note This is set by FTM command used in the production process
 * 
 * @see prod_complete()
 */
#define PROD_COMP_AA (0xAA1305AAul)  // This is when FTM command "complete production" issued
#define PROD_COMP_BB (0xBB2910BBul)  // This is when after FTM complete production issued and 120hrs timer expired



/*******************************************************************************
 * @brief   Production process complete
 *
 * @details This function is called by FTM when the production process is
 *          complete
 *
 * @return bool: true/false for success/failure
 */
bool prod_set_value( uint32_t prod_comp );

/*******************************************************************************
 * @brief   Reset production process complete
 *
 * @details This function resets the production complete magic number
 */
void prod_reset( void );

/*******************************************************************************
 * @brief   Get production magic value
 *
 * @details This function get the current magic value
 *
 * @return  magic value
 */
uint32_t prod_get_value( void );

/*******************************************************************************
 * @brief   At start of device reads the magic value
 *
 * @details This function get the current magic value
 *
 * @return  magic value
 */
void prod_value_init(void);

/******************************************************************************/
#endif // PRODUCTION_H
