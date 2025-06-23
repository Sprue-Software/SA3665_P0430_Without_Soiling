/*
 * switches_handler.h
 *
 *  Created on: 15 Jun 2022
 *      Author: uhegde
 */

#ifndef APP_INC_SWITCHES_HANDLER_H_
#define APP_INC_SWITCHES_HANDLER_H_

#include "stdint.h"
#include "stdbool.h"

#include "debug.h"

void switches_handler_task(void *arg);
void set_ftm_btn_pattern(bool btn_sp, bool btn_lp, bool btn_lh, bool btn_5p);
void set_ftm_btn_event(bool bTimerOver);
bool get_ftm_btn_event();
bool get_ftm_shortP();
bool get_ftm_longP();
bool get_ftm_longHold();
bool get_ftm_5shortPress();


#endif /* APP_INC_SWITCHES_HANDLER_H_ */
