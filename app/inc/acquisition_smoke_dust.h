/*******************************************************************************
 *
 * @file    acquisition_smoke_dust.h
 *
 * @brief   Smoke dust correction header file
 *
 * @date    12th Oct 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef _APP_INC_ACQUISITION_SMOKE_DUST_H_
#define _APP_INC_ACQUISITION_SMOKE_DUST_H_

#include <stdbool.h>
#include <stdint.h>

#include "debug.h"

/*******************************************************************************
 * @brief Initialise smoke averaging
 *
 * This function initialises the smoke averaging function used in
 * performing the dust correction activity.
 *
 * @param threshold     Cal reflection threshold
 * 
 * @note This must be called before calling the smoke average functions.
 */
void smoke_dust_init( const uint32_t threshold );

/*******************************************************************************
 * @brief Perform dust correction
 *
 * This function takes in the raw smoke value  and calculates the corrected raw
 * smoke value.
 * 
 * Averages of the raw smoke values are maintained over the hour and 7 days.
 * 
 * The dust correction factor is applied to every raw smoke value to obtain the
 * corrected smoke value. The dust correction factor has low and high limits to
 * avoid undue corrections. The initial value of the factor will not provide any
 * correction. A new factor is calcultaed if the corrected smoke average over the
 * last 24 hours changes by more than a specifed amount. If no correction has been
 * done in the 24 hours a new correction factor will be calculated.
 *
 * @param[in] smoke_raw_value               Raw smoke value
 *
 * @warning This is designed to be called at the same rate as the smoke acquisition
 * 
 * @note The initial dust correct factor will perform no correction. A new dust
 *       correction factor will not be calculated until at least 7 days worth of
 *       smoke values have been averaged.
 *
 * @return Corrected raw smoke value
 */
uint32_t smoke_dust_perform_correction( uint32_t smoke_raw_value );

/*******************************************************************************
 * @brief Handle fault conditions
 *
 * This function handles smoke chanber errors. The smoke chamber fault is
 * either set or cleared depending on the corrected smoke level and the specified
 * limit. Also, events are logged to the EEPROM.
 *
 * @param[in] smoke_level_corr      Raw smoke corrected value
 *
 * @see FaultHandler_FaultSet() DataLogging_SetEventLogbookRecord() 
 *      DegradedSmokeChamberFault DEF_LBE_SMOKE_CHAM_CONT_START
 *      DEF_LBE_SMOKE_CHAM_CONT_END
 */
void smoke_dust_handle_chamber_errors( const uint32_t smoke_level_corr );

/*******************************************************************************
 * @brief Get dust correction factor
 *
 * This function returns the current in-use dust correction factor percentage
 * 
 * @note Factor is multiplied by 100, 0.8 to 1.2 becomes 80 to 120
 * 
 * @return Dust correction factor percentage
 */
uint8_t smoke_dust_get_correction_factor( void );

/*******************************************************************************
 * @brief Is test mode active
 *
 * This function determines if test mode is active
 * 
 * @return True if active, otherwise false
 */
bool smoke_dust_is_test_mode( void );

/*******************************************************************************
 * @brief Set test mode
 *
 * This function sets the dust correction activity into test mode
 * 
 * @param mode      True to set test mode
 */
void smoke_dust_set_test_mode( const bool mode );

/*******************************************************************************
 * @brief Get the raw test value
 *
 * This function gets the raw test value
 * 
 * @return Raw test value
 */
uint32_t smoke_dust_get_raw_test_value( void );

/*******************************************************************************
 * @brief Set the raw test value
 *
 * This function sets the raw test value
 * 
 * @param[in] value      Raw test value
 */
void smoke_dust_set_raw_test_value( const uint32_t value );

/*******************************************************************************
 * @brief Get the cal reflection test value
 *
 * This function gets the cal reflection test value
 * 
 * @return Cal reflection test value
 */
uint32_t smoke_dust_get_calref_test_value( void );

/*******************************************************************************
 * @brief Set the cal reflection test value
 *
 * This function sets the cal reflection test value
 * 
 * @param[in] value      Cal reflection test value
 */
void smoke_dust_set_calref_test_value( const uint32_t value );

/******************************************************************************/
#endif /* APP_INC_ACQUISITION_SMOKE_DUST_H_ */
