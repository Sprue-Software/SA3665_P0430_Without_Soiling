/*******************************************************************************
 *
 * @file    user_info_page.h
 *
 * @brief   User info page header file
 *
 * @date    16 Feb 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef USER_INFO_PAGE_H
#define USER_INFO_PAGE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h> 

/*******************************************************************************
 * @brief Page size
 * 
 * Size in bytes of used area in the information page
 * 
 * @note Full size is 1024, but only 256 bytes is currently used
 */
#define UIP_SIZE (256u)

/*******************************************************************************
 * @brief Programming mutex name
 */
#define UIP_MUTEX_NAME "uip"

/*******************************************************************************
 * @brief   Initialise
 *
 * @details This function this module for use
 * 
 * @note    This must be called before using an function
 */
void uip_init( void );

/*******************************************************************************
 * @brief   Get size of used information page
 *
 * @details This function gets the number of bytes allocated for use from the
 *          information page.
 * 
 * @note    The maximum number of bytes avaiable is 1024.
 * 
 * @return  Number of bytes allocated
 */
uint32_t uip_get_size( void );

/*******************************************************************************
 * @brief   Is information page erased
 *
 * @details This function determines if the entire in-use area of the information
 *          page is blank.
 * 
 * @return  true if it's erased
 */
bool uip_is_erased( void );

/*******************************************************************************
 * @brief   Is region in the information page erased
 *
 * @details This function determines if the specfified range of the information
 *          page is blank.
 * 
 * @param[in] start   Offset from start of page
 * @param[in] len     Length in bytes to check
 * 
 * @return  true if it's erased
 */
bool uip_is_region_erase( const uint32_t start, const uint32_t len );

/*******************************************************************************
 * @brief   Erase the information page
 *
 * @details This function erases the entire information page
 * 
 * @return  true if the erase has been successful
 */
bool uip_erase( void );

/*******************************************************************************
 * @brief   Program area in the information page
 *
 * @details This function programs the specified date and length into the
 *          information page. Exsisting items are preserved.
 * 
 * @param[in] start     Offset from start of page
 * @param[in] data      Pointer to data to program
 * @param[in] data_len  Length in bytes to program
 * 
 * @return  true on success, otherwise false
 */
bool uip_program( const uint32_t start, const uint8_t * data, const uint32_t data_len );

/*******************************************************************************
 * @brief   Read area from the information page
 *
 * @details This function reads the specified date and length from the
 *          information page.
 * 
 * @param[in] start     Offset from start of page
 * @param[in] data      Pointer to location to save data
 * @param[in] data_len  Length in bytes to read
 * 
 * @return  true on success, otherwise false
 */
bool uip_read( const uint32_t start, const uint8_t * data, const uint32_t data_len );

/******************************************************************************/
#endif /* USER_INFO_PAGE_H */
