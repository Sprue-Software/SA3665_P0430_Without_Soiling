/*******************************************************************************
 *
 * @file    CO_Calibration.h
 *
 * @brief   CO Calibration header file
 *
 * @date    26 Oct 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef CO_CALIBRATION_H
#define CO_CALIBRATION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h> 

#include "em_msc.h"

/**
 * @brief Minimum temperature value
 * 
 * Actual temp_min limit is 20C but factory team decide the final as it is hard
 * to maintain in winter
 * 
 * @note This is only used by the CLI when calibrating
 */
#define COCAL_TEMP_MIN (1500U)

/**
 * @brief Maximum temperature value
 * 
 * @note This is only used by the CLI when calibrating
 */
#define COCAL_TEMP_MAX (2600U)

/**
 * @brief Minimum humidity value
 * 
 * @note This is only used by the CLI when calibrating
 */
#define COCAL_HUMIDITY_MIN (35U)

/**
 * @brief Maximum humidity value
 * 
 * @note This is only used by the CLI when calibrating
 */
#define COCAL_HUMIDITY_MAX (60U)

/**
 * @brief Number of times sensors read
 * 
 * @note This is only used by the CLI when calibrating
 */
#define COCAL_NUM_OF_SENSOR_READS (1)

/**
 * @brief Calibration temperature
 * 
 * @note THIS DATA WAS TAKEN FROM CO SENSOR SERIAL NUMBER 2233470013
 */
#define COCAL_CALIB_TEMP_CODE (12)

/**
 * @brief Calibration humidity (%)
 * 
 * @note THIS DATA WAS TAKEN FROM CO SENSOR SERIAL NUMBER 2233470013
 */
#define COCAL_CALIB_HUMIDITY (51)

/**
 * @brief CO nA per ppm scaling factor
 * 
 * This value is used to scale the value
*/
#define COCAL_NA_PER_PPM_SCALING_FACTOR (1000)

/*******************************************************************************
 * @brief   Erase co calibration data
 *
 * @details This function erases all the calibration values held in the User Data
 *          area in internal FLASH.
 * 
 * @note A full page is erased
 * 
 * @return Returns mscReturnOk on success
*/
MSC_Status_TypeDef cocal_erase( void );

/*******************************************************************************
 * @brief   Check CRC for default calibration
 *
 * @details This function checks the default CRC for the FLASH calibration table
 * 
 * @return True id CRC correct, otherwise false
*/
bool cocal_check_def_crc( void );

/*******************************************************************************
 * @brief   Check CRC for calibration
 *
 * @details This function checks the CRC for the FLASH calibration table
 * 
 * @return True id CRC correct, otherwise false
*/
bool cocal_check_crc( void );

/*******************************************************************************
 * @brief   Program calibration table
 *
 * @details This function programs the specified calibration table in the User 
 *          Data area of the internal FLASH.
 * 
 * @param[in] addr      Destination address to program
 * @param[in] data      Source data to program
 * @param[in] data_len  Source data length
 * 
 * @note No CRC is calculated, the CRC must be included already
 * 
 * @return Returns mscReturnOk on success
*/
MSC_Status_TypeDef cocal_program( const uint8_t * addr, const uint8_t * data, const uint32_t data_len );

/*******************************************************************************
 * @brief   Program default calibration table
 *
 * @details This function programs the default calibration table in the User 
 *          Data area of the internal FLASH. When force is false, it is only
 *          programmed if the destination is fully esared already.
 * 
 * @param[in] force   Program regardless
 * 
 * @return Returns true on success, otherwise false
*/
bool cocal_progam_defaults( const bool force );

/*******************************************************************************
 * @brief   Update the programming buffer
 *
 * @details This functions updates the internal progamming buffer with specified
 *          data. The data consists of a stream of ascii hex numbers (2 digits)
 *          per value. Each call to this function will append data at the end of
 *          the previous call. The reset flasg can be used to reset the start
 *          start position to zero prior to updating the buffer. To reset the buffer
 *          without adding any data, set src to NULL.
 * 
 * @param[in] src       Pointer to source ascii string, NULL terminated
 * 
 * @warning The last 2 bytes of data before programming must be the CRC
 * 
 * @return Returns true on success, otherwise false
*/
bool cocal_update_prog_buf_ascii( const uint8_t * src );

/*******************************************************************************
 * @brief   Flash the programming buffer
 *
 * @details This functions writes the programming buffer to internal flash.
 * 
 * @warning The CRC must be correct to program
 * 
 * @return Returns true on success, otherwise false
*/
bool cocal_program_calib( void );

/*******************************************************************************
 * @brief   Get temperature compensation value
 *
 * @details This function gets the speified temperature compensation value from
 *          the calibration table.
 * 
 * @param[in] item   Temperature index
 * 
 * @note Index is from 0
 * 
 * @return Temperature compensation value
*/
uint8_t cocal_get_temperature_comp( const uint8_t item );

/*******************************************************************************
 * @brief   Get aging SF value
 *
 * @details This function gets the speified aging SF value from the calibration
 *          table.
 * 
 * @param[in] item   Aging index
 * 
 * @note Index is from 0
 * 
 * @return Aging SF value
*/
uint8_t cocal_get_aging_sf( const uint8_t item );

/*******************************************************************************
 * @brief   Read EEPROM calibration values
 *
 * @details This function gets the calibration from the EEPROM and stores them
 *          into internal values
 * 
 * @note Defaults for each value is used if the EEPROM is erased
*/
void cocal_read_eeprom_values( void );

/*******************************************************************************
 * @brief   Get in-use co na per ppm
 *
 * @details This function gets the in-use co na per ppm value
 * 
 * @return Co cf
*/
uint16_t cocal_get_co_na_per_ppm( void );

/*******************************************************************************
 * @brief   Get in-use co cal
 *
 * @details This function gets the in-use co cal value
 * 
 * @return Co cal
*/
uint16_t cocal_get_co_cal( void );

/*******************************************************************************
 * @brief   Get in-use co variance
 *
 * @details This function gets the in-use co variance value
 * 
 * @return Co variance
*/
uint16_t cocal_get_co_var_thresh( void );

/*******************************************************************************
 * @brief   Get in-use co calibration temperature
 *
 * @details This function gets the in-use calibration temperature. This is
 *          converted from the temperature code.
 * 
 * @return Calibration temperature
*/
uint16_t cocal_get_co_calib_temperature( void );

/*******************************************************************************
 * @brief   Get in-use co calibration humidity
 *
 * @details This function gets the in-use calibration humidity
 * 
 * @return Calibration humidity percentage
*/
uint8_t cocal_get_co_calib_humidity( void );

/*******************************************************************************
 * @brief   Inialise CO Calibration module
 *
 * @details This function is the initialisation function for the CO Calibration
 *          module and must be called before any exported functions are used.
*/
void cocal_init( void );

/******************************************************************************/
#endif  // End of CO_CALIBRATION_H
