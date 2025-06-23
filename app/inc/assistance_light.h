/*
 * assistance_light.h
 *
 *  Created on: 31 Aug 2023
 *      Author: araheem
 */

#include "stdbool.h"
#include "os.h"
//#include "hal_gpio.h"

#ifndef APP_INC_ASSISTANCE_LIGHT_H_
#define APP_INC_ASSISTANCE_LIGHT_H_

void GPIO_TurnAssistanceLEDon(void);
void GPIO_TurnAssistanceLEDoff(void);
void SetAssistanceLightStatus(bool status);
bool GetAssistanceLightStatus(void);
uint16_t GetAssistancelogPeriod(void);

#endif /* APP_INC_ASSISTANCE_LIGHT_H_ */
