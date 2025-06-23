/*
 * system_event.h
 *
 *  Created on: 4 Apr 2022
 *      Author: ndiwathe
 */

#ifndef APP_INC_SYSTEM_EVENTS_H_
#define APP_INC_SYSTEM_EVENTS_H_

#include "debug.h"
#include "hal_switches.h"

/* Enum defines */

/**
 * These are the various behavioural System modes
 */
typedef enum {
	Standby_Mode = 0, /**< System default Mode, device is de-mounted*/
	Commisioning_Mode,/**< System Mode, device is mounted first Time*/
	Operational_Mode,/**< System Operating Mode, device Successfully Pass Commissioning mode*/
	Transport_Mode,/**< System Mode, When device is returned/damaged/Faulty*/
	Functional_Test_Mode, /**< System Test Mode, All Test Functionality is possible*/
	Shutdown_Mode,/**< System Mode, When device is critically Low Battery Safe Shutdown*/
	NUM_Modes
} behaviour_state_enum_System_modes;

/**
 * These are the various behavioural Operational states
 */
typedef enum {
	state_Idle,/**< System default Operational State, */
	state_Smoke_Alarm,/**< System  Operational State, When Smoke Alarm conditions occurs */
	state_Smoke_Alarm_Silence,/**< System  Operational State, When Button press for silence Smoke Alarm */
	state_Heat_Alarm,/**< System  Operational State, When Heat Alarm conditions occurs */
	state_Heat_Alarm_Silence,/**< System  Operational State, When Button press for silence heat Alarm */
	state_CO_Alarm,/**< System  Operational State, When CO Alarm conditions occurs */
	state_CO_Alarm_Silence,/**< System  Operational State, When Button press for silence CO Alarm */
	state_Remote_Alarm,/**< System  Operational State, When CO Alarm conditions occurs */
	state_BISTMode,/**< System  Operational State, When MCU-2 requests or Serial Commands */
	State_Airing_Configuration,/**< System  Operational State, When Button Press initiated Airing Configuration with 60 sec timeout */
	State_Radio_PriData_Configuration, /**< System  Operational State, When Button Press initiated Radio Private Data with 60 sec timeout */
	NUM_State
} behaviour_state_enum_operational_States;

/**
 * These are the various ADS operate states
 */
typedef enum {
	operate_disabled, /**< operate_disabled, ADS off-base, shipped tag inserted*/
	operate_active, /**< operate_active, ADS on-base, shipping tag removed*/
} ADS_operate_state_enum;

/**
 * CO modes/state
 */
typedef enum {
	co_none, /**< co_none, no CO detected*/
	co_llw, /**< co_llw, CO detected at a low level warning */
	co_high, /**< co_high, high levels of CO detected*/
	co_super /**< co_super, very high levels of CO detected*/
} co_state_enum;

/**
 * Smoke modes/state
 */
typedef enum {
	Smoke_none, /**< Smoke_none, no Smoke detected*/
	Smoke_high, /**< Smoke_high, high levels of Smoke detected*/
	Smoke_super /**< Smoke_super, very high levels of Smoke detected*/
} Smoke_state_enum;

/**
 * Heat modes/state
 */
typedef enum {
	Heat_none, /**< Heat_none, no Heat detected*/
	Heat_high, /**< Heat_high, high levels of Heat detected*/
	Heat_super, /**< Heat_super, very high levels of Heat detected*/
} heat_state_enum;

/* Alarm States */
#define DEF_ALARM_END                        (0x00)
#define DEF_ALARM_SMOKE                        (0x01)
#define DEF_ALARM_HEAT                         (0x02)
#define DEF_ALARM_CO                         (0x03)
#define DEF_ALARM_TEST                         (0x04)
#define DEF_ALARM_MUTED                         (0x05)

/* Function prototypes */
void runOperateModule(OS_FLAGS flags_0, OS_FLAGS flags_1);
void runBehaviouralModule(OS_FLAGS flags_0, OS_FLAGS flags_1);
void clearEventFlag(OS_FLAGS flags_0, OS_FLAGS flags_1);
behaviour_state_enum_operational_States getBehavioural_Operational_State(
		void);
void setBehavioural_Operational_State(
		behaviour_state_enum_operational_States state);
behaviour_state_enum_System_modes getBehavioural_System_Modes(
		bool read_From_eeprom);
void setBehavioural_System_Modes(
		behaviour_state_enum_System_modes system_mode);
void Stop_All_timers_except_timestamp(void);
uint32_t OPERATE_EVENTS (void);
uint32_t DIAGNOSTIC_EVENTS (void);
void Start_Diagnostic_BIST(void);
void setRadioPriDataStatus(uint32_t status);
void startLaserBIST(void);
void checkBISTResults(void);
co_state_enum getCoState(void);
void SetStandByModeCheckButton(bool status);
bool GetStandByModeCheckButton(void);
void handle_state_laser_extended_test(void);

#endif /* APP_INC_SYSTEM_EVENTS_H_ */
