/*******************************************************************************
 *
 * @file    v3sctrl.h
 *
 * @brief   3VS control header file
 *
 * @date    15 Sept 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#ifndef APP_INC_3VS_CTRL_H_
#define APP_INC_3VS_CTRL_H_

#include <stdbool.h>

/**
 * @brief Mutex name
 */
#define V3S_CTRL_MUTEX_NAME     "3VS Ctrl Mutex"

/*******************************************************************************
 * @brief Turn on the 3VS supply
 * 
 * Turn on the 3VS supply. The 3VS mutex will be obtained before turning on the
 * supply. After the supply has been turned on, the mutex is released.
 * 
 * @warning This cannot be called from an ISR
 */
#define V3S_ON()                (void)v3s_ctrl_on_off(true,false)

/*******************************************************************************
 * @brief Turn on and lock the 3VS supply
 */
#define V3S_ON_LOCK()           v3s_ctrl_on_off(true,true)

/*******************************************************************************
 * @brief Unlock the 3VS supply and set state
 */
#define V3S_ON_UNLOCK(A)        v3s_ctrl_on_unlock((A))

/*******************************************************************************
 * @brief Turn off the 3VS supply
 */
#define V3S_OFF()               (void)v3s_ctrl_on_off(false,false)

/*******************************************************************************
 * @brief 3VS initialisation
 *
 * Initialise library for use and configure 3VS pin as an output, initially
 * low turning the supply off.
 *
 * @note This must be called before using any other functions
 */
void v3s_ctrl_init( void );

/*******************************************************************************
 * @brief Is the 3VS supply on
 *
 * This function returns true if the 3VS supply is on, otherwise false
 *
 * @note This call does not use the mutex
 */
bool v3s_ctrl_is_on( void );

/*******************************************************************************
 * @brief Turn 3VS supply on or off
 *
 * This function turns the 3VS supply either on or off. If required, it can
 * optionally lock the supply meaning no other caller can use the supply until
 * it's unlocked.
 *
 * @param[in] on    Turn the supply on
 * @param[in] lock  Lock the supply
 *
 * @return Previous supply status, true on, false off
 *
 * @note The supply can only be locked when turning on, not off
 */
bool v3s_ctrl_on_off( const bool on, const bool lock );

/*******************************************************************************
 * @brief Unlock the supply
 *
 * This function unlocks the supply allowing other users to use the supply.
 * Using the specified option, return the supply to its previous state.
 *
 * @param[in] on    Turn the supply on
 *
 * @warning Only call this if the the caller has alreadt locked the supply
 */
void v3s_ctrl_on_unlock( const bool on );

/******************************************************************************/
#endif /* APP_INC_3VS_CTRL_H_ */
