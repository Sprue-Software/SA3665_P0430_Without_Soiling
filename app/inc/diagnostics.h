/*
 *
 *  @file diagnostics.h
 *  @brief: This File describe variable required for diagnostic & fault
 *  Created on: 4 Apr 2022
 *  Author:
 */

#ifndef APP_INC_DIAGNOSTICS_H_
#define APP_INC_DIAGNOSTICS_H_

#include "debug.h"

#include "comms_handler.h"
#include "hal_switches.h"
#include "system_events.h"

bool heat_measurement(bool bist_test);
bool diagnostics(behaviour_state_enum_System_modes modes,
		Ads_state_t ADS_status, OS_FLAGS flags);
uint16_t getThermistorADC(void);

bool get_Assistance_light_Status(void);
OS_FLAGS diagnostics_GetDelayedFlags(void);
uint8_t runBuzzerBist();
void setBistResult(bool val);
bool getBistResult(void);
void setIsDeviceSmokeEnable(bool val);
bool getIsDeviceSmokeEnable(void);
#endif /* APP_INC_DIAGNOSTICS_H_ */
