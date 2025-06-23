/*******************************************************************************
 *
 * @file    ctune.h
 *
 * @brief   ctune header file
 *
 * @date    19 Feb 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef CTUNE_H
#define CTUNE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h> 

/**
 * @brief Location in FLASH of ctune value
 * 
 * @note This is the last 4-bytes of the first 256 bytes
 */
#define CT_TUNING_FLASH_LOCATION (252)

/**
 * @brief Minimum ctune value
 */
#define CTUNE_MIN 0x01

/**
 * @brief ctune test value
 */
#define CTUNE_TEST (CTUNE_MIN+((CTUNE_MAX-CTUNE_MIN)/2))

/**
 * @brief Maxiimum ctune value
 */
#define CTUNE_MAX 0x4F

/*******************************************************************************
 * @brief   Tune initialisation
 *
 * @details This function tunes the low frequency crystal oscillator with the
 *          tuning value held in flash
 *
 * @return  true if tuned successfully
 */
bool ct_init( void );

/*******************************************************************************
 * @brief   Get ctune value from FLASH
 *
 * @details This function reads the value ctune from FLASH
 *
 * @param[in] value   Pointer to ctune
 * 
 * @note If a value of 0xFFFF (erased) is found, false will be returned
 * 
 * @return  true if read successfully
 */
bool ct_get_tuning_value( uint32_t * const value );

/*******************************************************************************
 * @brief   Set ctune value to FLASH
 *
 * @details This function writes the value ctune to FLASH
 *
 * @param[in] value   New ctune value
 * 
 * @return  true if written successfully
 */
bool ct_set_tuning_value( const uint32_t value );

/*******************************************************************************
 * @brief   Tune oscillator
 *
 * @details This function tunes the low frequency crystal oscillator with the
 *          specified value
 *
 * @param[in] value   New ctune value
 * 
 * @return  true if tuned successfully
 */
bool ct_tune( const uint32_t value );

/******************************************************************************/
#endif // CTUNE_H
