/*******************************************************************************
 *
 * @file    debug.h
 *
 * @brief   Debug header file
 *
 * @date    03 Jan 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef _DEBUG_H_
#define _DEBUG_H_

/**
 * @brief No print debug macro
 */
#define DEBUG_NO_PRINT(str, numMode, dataValue)

/**
 * @brief No printf debug macro
 */
#define DEBUG_NO_PRINTF(...) (void)0

#ifdef DEBUG_BUILD

/**
 * @brief Print debug macro
 */
#define DEBUG_PRINT(str, numMode, dataValue) debug_out((str),(numMode),(dataValue))

/**
 * @brief Printf debug macro
 */
#define DEBUG_PRINTF(...) debug_printf(__VA_ARGS__)

/* Set the following to provide debug messages as required */

#define DEBUG_CO                    DEBUG_PRINT
#define DEBUG_HEAT                  DEBUG_NO_PRINT
#define DEBUG_SMOKE                 DEBUG_PRINT
#define DEBUG_SMOKE_DUST            DEBUG_NO_PRINT
#define DEBUG_AMBIENT               DEBUG_NO_PRINT
#define DEBUG_APP                   DEBUG_PRINT
#define DEBUG_BATT                  DEBUG_PRINT
#define DEBUG_DATA_LOGGING          DEBUG_NO_PRINT
#define DEBUG_DIAG                  DEBUG_PRINT
#define DEBUG_EEPROM                DEBUG_NO_PRINT
#define DEBUG_FAULT_HANDLER         DEBUG_PRINT
#define DEBUG_AFE                   DEBUG_PRINT
#define DEBUG_AFE_PRINTF            DEBUG_NO_PRINTF
#define DEBUG_SWITCHES              DEBUG_NO_PRINT
#define DEBUG_BUZZER                DEBUG_NO_PRINT
#define DEBUG_FTM                   DEBUG_NO_PRINT
#define DEBUG_TEMP_HUMID            DEBUG_NO_PRINT
#define DEBUG_SWI_HAND              DEBUG_NO_PRINT
#define DEBUG_TIME                  DEBUG_PRINT
#define DEBUG_CO_VAR                DEBUG_NO_PRINT
#define DEBUG_TELEGRAM              DEBUG_NO_PRINT
#define DEBUG_EVENTS                DEBUG_NO_PRINT
#define DEBUG_SPI                   DEBUG_NO_PRINT

#else

#define DEBUG_PRINTF(...) (void)0

/* DO NOT ENABLE, THESE ARE ALWAYS DISABLED IN A RELEASE BUILD */

#define DEBUG_CO                    DEBUG_NO_PRINT
#define DEBUG_HEAT                  DEBUG_NO_PRINT
#define DEBUG_SMOKE                 DEBUG_NO_PRINT
#define DEBUG_SMOKE_DUST            DEBUG_NO_PRINT
#define DEBUG_AMBIENT               DEBUG_NO_PRINT
#define DEBUG_APP                   DEBUG_NO_PRINT
#define DEBUG_BATT                  DEBUG_NO_PRINT
#define DEBUG_DATA_LOGGING          DEBUG_NO_PRINT
#define DEBUG_DIAG                  DEBUG_NO_PRINT
#define DEBUG_EEPROM                DEBUG_NO_PRINT
#define DEBUG_FAULT_HANDLER         DEBUG_NO_PRINT
#define DEBUG_AFE                   DEBUG_NO_PRINT
#define DEBUG_AFE_PRINTF            DEBUG_NO_PRINT
#define DEBUG_SWITCHES              DEBUG_NO_PRINT
#define DEBUG_LED_BUZZER            DEBUG_NO_PRINT
#define DEBUG_FTM                   DEBUG_NO_PRINT
#define DEBUG_TEMP_HUMID            DEBUG_NO_PRINT
#define DEBUG_SWI_HAND              DEBUG_NO_PRINT
#define DEBUG_TIME                  DEBUG_NO_PRINT
#define DEBUG_CO_VAR                DEBUG_NO_PRINT
#define DEBUG_TELEGRAM              DEBUG_NO_PRINT
#define DEBUG_EVENTS                DEBUG_NO_PRINT
#define DEBUG_SPI                   DEBUG_NO_PRINT
#define DEBUG_BUZZER                DEBUG_NO_PRINT

#endif  // DEBUG_BUILD

#endif  // _DEBUG_H_
