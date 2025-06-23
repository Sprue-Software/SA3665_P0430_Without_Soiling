/*
 * hal_AFE.h
 *
 *  Created on: 25 Apr 2022
 *      Author: ndiwathe
 */

#ifndef APP_INC_HAL_AFE_H_
#define APP_INC_HAL_AFE_H_

#include "debug.h"
#include <stdbool.h>
#include "os.h"

/*SPI register*/
#define AFE_WRITE_REGISTERS_0  0x00
#define AFE_WRITE_REGISTERS_1  0x02
#define AFE_WRITE_REGISTERS_2  0x04
#define AFE_WRITE_REGISTERS_3  0x06
#define AFE_WRITE_REGISTERS_4  0x08
#define AFE_WRITE_REGISTERS_5  0x0a
#define AFE_WRITE_REGISTERS_6  0x0c
#define AFE_WRITE_REGISTERS_7  0x0e
#define AFE_WRITE_REGISTERS_8  0x10
#define AFE_WRITE_REGISTERS_9  0x12

/* Buzzer Test ***************/
#define AFE_DUMMY_DATA  0xAA  /* Microchip provided*/
#define AFE_CLEAR_REG_7 0x80  /* Microchip provided*/
#define AFE_CHECK_THE_FAULT  (1u<<7)
/* Buzzer Test End  ***************/

/*ABUF [3:0] -- ABUF Channel select*/
#define AFE_REG_0_ABUF_CHANNEL_SELECT_OFF         0x0U
#define AFE_REG_0_ABUF_PHOTO_AMP_REF_OUTPUT       0x01U /* 0.2v Photo Amp refernce OUtput*/
#define AFE_REG_0_ABUF_PHOTO_AMP_OUTPUT           0x02U
#define AFE_REG_0_ABUF_CO_AMP_OUTPUT              0x03U
#define AFE_REG_0_ABUF_CO_0_3_V_REF_OUTPUT        0x04U
#define AFE_REG_0_ABUF_SCALED_VBST_VOLTAGE_OUTPUT   0x05U
#define AFE_REG_0_ABUF_SCALED_IRCAP_VOLTAGE_OUTPUT  0x06U
#define AFE_REG_0_ABUF_SCALED_VBAT_VOLTAGE_OUTPUT   0x07U
#define AFE_REG_0_ABUF_SCALED_VBATLTD_MONITOR_VOLTAGE_OUTPUT 0x08U
#define AFE_REG_0_ABUF_HORN_FAULT_TEST        0x09U
#define AFE_REG_0_ABUF_CO_INP_PIN_VOLATAGE      0x0AU
#define AFE_REG_0_ABUF_CO_INN_PIN_VOLATAGE      0x0BU
#define AFE_REG_0_ABUF_SCALED_LED1_VOLTAGE      0x0CU
#define AFE_REG_0_ABUF_SCALED_LED2_VOLTAGE      0x0DU
#define AFE_REG_0_ABUF_TEMP_SENSOR_INPUT      0x0EU
#define AFE_REG_0_ABUF_CO_INN_PIN_VOLTAGE       0x0FU
/* REG0  7-4  Not Used */

#define AFE_REG_1_LBE   1u  /* Low Boost Operations Enabled */
#define AFE_REG_1_HBE   (1u<<1)  /* High Boost Operations Enabled */
#define AFE_REG_1_IRCAP_POWER (1u<<2) /* IRCAP Power Enabled  */
/* REG1  7-3  Not Used */

#define AFE_REG_2_PHOTO_AMP_POWER_ON  1u
#define AFE_REG_2_PHOTO_INEGRATOR_STATUS (1u<<1)
#define AFE_REG_2_LED1_EN    (1u<<3)
#define AFE_REG_2_LED2_EN    (1u<<4)
#define AFE_REG_2_VBATLED_EN (1u <<5)
#define AFE_REG_2_RLED_EN (1u<<6)
#define AFE_SET_ZERO  0x00
/* REG2  7 & 2  Not Used */

#define AFE_REG_3_CO_AMP_POWER_ON   1u
#define AFE_REG_3_CO_REF_POWER_ON  (1u <<1)
#define AFE_REG_3_CO_SHUNT_ENABLED  (1u<<3)
#define AFE_REG_3_CO_CURRENT_SINK_SOURCE_POLARITY (1u<<4)
#define AFE_REG_3_CO_TEST_ON_OFF (1u<<5)
#define AFE_REG_3_CO_CURRENT_SINK_SOURCE_SELECT (1u<<6)
#define AFE_REG_3_CO_OP_AMP_OUTPUT_TRISTATE_ENABLED   (1u<<7) /*CO Op amp Output Tristate Enable*/

#define AFE_REG_4_HORN_ENABLE  1u /*Horn Enable*/
#define AFE_REG_4_HORN_TEST_ENABLE (1u <<1) /*Horn test  Enable*/

#define AFE_REG_4_HORN_TEST_SELECT_STANDBY 0x00u /*Standby*/

/*** Horn Test With Enable Bit Set */
#if 0
#define AFE_REG_4_HORN_TEST_SELECT_STANDBY_1 0x12u /*Standby*/
#define AFE_REG_4_HORN_FB_SHORT_VDD          0x22 /*FB short to VDD*/
#define AFE_REG_4_FB_OPEN_SHORT_TO_VSS_OR_HS_HB_SHORT_TO_VSS_OR_HS  0x3A /*FB open, HB open, FB short to VSS or HS, HB short to VSS or HS*/
#define AFE_REG_4_HS_OPEN_OR_SHORT_VSS  0x4A /*HS open or shorted to VSS */
#define AFE_REG_4_HS_SHORT_TO_VDD    0x52 /*HS short to VDD*/
#define AFE_REG_4_FB_TO_HB_SHORT   0x6A /*FB to HB short*/
#define AFE_REG_4_HB_SHORT_TO_VDD   0x72 /*HB short to VDD*/
#endif
/***********************************/
#define AFE_REG_4_HORN_TEST_RESULT  (1u <<7) /*  = Horn Test Pass 0 = Horn Test Fail*/

#define AFE_REG_4_HORN_TEST_REG(a)  ((a)<<3)

/*
Shorts to vdd     x       x 		    0100
*/
#define AFE_REG_4_HORN_TEST_1				AFE_REG_4_HORN_TEST_REG(0x04)                   /* FB short to VDD/VBAT */

/*
Opens   open      x       x 				0111
Opens   x         open    x 				0111
Shorts  to vss    x       x 		    0111
Shorts  to HS     x       x 			  0111
Shorts  x         to vss  x 		    0111
Shorts  x         to hs   x 			  0111
*/
#define AFE_REG_4_HORN_TEST_2				AFE_REG_4_HORN_TEST_REG(0x07)                   /* FB open, HB open, FB short to VSS or HS, HB short to VSS or HS */

/*
Opens   x         x       open			1001
Shorts  x         x       to vss 		1001
*/
#define AFE_REG_4_HORN_TEST_3				AFE_REG_4_HORN_TEST_REG(0x09)                   /* HS open or shorted to VSS */

/*
Shorts x          x       to vdd 		1010
*/
#define AFE_REG_4_HORN_TEST_4				AFE_REG_4_HORN_TEST_REG(0x0A)                   /* HS short to VDD/VBAT */

/*
Shorts to HB      x       x 			  1101
*/
#define AFE_REG_4_HORN_TEST_5				AFE_REG_4_HORN_TEST_REG(0x0D)                   /* FB to HB short */

/*
Shorts x          to vdd  x 		    1110
*/
#define AFE_REG_4_HORN_TEST_6				AFE_REG_4_HORN_TEST_REG(0x0E)                   /* HB short to VDD/VBAT */

#define AFE_REG_5_LOW_BOOST_VOLTAGE_SETTINGS          1u
#define AFE_REG_5_HIGH_BOOST_VOLTAGE_SETTINGS_11_5V   0x04u     /* High Boost Voltage 11.5v */
#define AFE_REG_5_HIGH_BOOST_VOLTAGE_SETTINGS_10_0V   0x02u     /* High Boost Voltage 10.0v */
#define AFE_REG_5_HIGH_BOOST_VOLTAGE_SETTINGS_8_5V    0x00u     /* High Boost Voltage 8.5 v */
#define AFE_REG_5_RDNOW                               (1u<<4)
/* REG5  7 & 5  Not Used */

#define AFE_REG_6_PHOTO_GAIN_1 0x00u
#define AFE_REG_6_PHOTO_GAIN_2 0x01u
#define AFE_REG_6_PHOTO_GAIN_4 0x02u
#define AFE_REG_6_PHOTO_GAIN_8 0x03u
#define AFE_REG_6_PHOTO_GAIN_16 0x04u
#define AFE_REG_6_PHOTO_GAIN_32 0x05u

#define AFE_REG_6_INTEGRATION_TIME_CONTROL_METHOD    (1u<<3)
#define AFE_REG_6_INTEGRATION_TIME_70_US 0x00u
#define AFE_REG_6_INTEGRATION_TIME_80_US 0x01u
#define AFE_REG_6_INTEGRATION_TIME_90_US 0x02u
#define AFE_REG_6_INTEGRATION_TIME_100_US 0x03u
#define AFE_REG_6_ENBUF_ENABLE  (1u << 7)
/* REG6  6  Not Used */

#define AFE_REG_7_LED1_CURRENT_20MA 0x00u
#define AFE_REG_7_LED1_CURRENT_40MA  0x01u
#define AFE_REG_7_LED1_CURRENT_50MA  0x02u
#define AFE_REG_7_LED1_CURRENT_60MA 0x03u
#define AFE_REG_7_LED1_CURRENT_80MA  0x04u
#define AFE_REG_7_LED1_CURRENT_100MA  0x05u
#define AFE_REG_7_LED1_CURRENT_150MA  0x06u
#define AFE_REG_7_LED1_CURRENT_200MA  0x07u
/* REG7  7 &3 Not Used */

#define AFE_REG_8_FEED_HORN_CONTROL_FOR_2_OR_3_PIN   1u
#define AFE_REG_8_HEN_CONTROL_ENABLED         (1u<<1)
#define AFE_REG_8_EN_CONTROL_ENABLED         (1u<<2)
/* REG8  7to 3    Not Used */

#define AFE_REG_9_LED1_TC_0   0
#define AFE_REG_9_LED1_TC_2   0x01u
#define AFE_REG_9_LED1_TC_3   0x02u
#define AFE_REG_9_LED1_TC_4  0x03u
#define AFE_REG_9_LED1_TC_5  0x04u

#define AFE_REG_9_LED2_TC_0   0
#define AFE_REG_9_LED2_TC_2   0x01u
#define AFE_REG_9_LED2_TC_3   0x02u
#define AFE_REG_9_LED2_TC_4  0x03u
#define AFE_REG_9_LED2_TC_5  0x04u
/* REG9  7 & 3    Not Used */

/**
 * AFE_setup_t
 * @brief This  will provide interface to control AFE
 */
typedef enum {
	setup_batteryVoltage = 0,/*setup ADC for battery voltage and impedance measurement*/
	setup_batteryImpedance,/* <NOT USED> setup ADC for battery impedance measurement*/
	setup_lightSensor,/*setup ADC for light sensor measurement*/
	setup_Co_High_gain,/*setup ADC for CO measurement*/
	setup_Co_Low_gain,/*setup ADC for CO measurement*/
	setup_CoSensorTest,/*CO sensor test measurement*/
	setup_thermistor, /*for thermistor measurement*/
	setup_SmokeTest, /* Smoke measurement*/
	setup_Buzzer_2_wire_init, /* 2 wire  Buzzer */
	setup_Buzzer_2_wire_on,/* 2 wire  Buzzer on */
	setup_Buzzer_2_wire_oFF,/* 2 wire  Buzzer off */
	setup_Buzzer_2_wire_deinit,/* 2 wire deinit  Buzzer */
	setup_Buzzer_3_wire_init,/* 3 wire init  Buzzer */
	setup_Buzzer_3_wire_deinit,/* 3 wire deinit  Buzzer */
	setup_Buzzer_3_wire_on,/* 3 wire  Buzzer */
	setup_Buzzer_3_wire_oFF,/* 3 wire  Buzzer */
	setup_BuzerTest, /* 3 wire  Buzzer Test*/
	setup_Heartbeat, /* Heartbeat Led*/
	setup_powerLEDOn,
	setup_powerLEDOff,
	setup_Soling,
	setup_FW_TEST_adc0,
	setup_FW_TEST_adc1,
	setup_FW_TEST_Adc_GPIO_SOIL_B,
	setup_FW_TEST_Adc_GPIO_SOIL_A,
	setup_FW_TEST_Adc_GPIO_LIGHT,
	END_SETUP,
    setup_Cmd_None = 0xFFFF
} AFE_setup_t;

/**
 * SmokeResult
 * @brief This result will provide details about dark & bright light levels
 */
typedef struct {
	uint32_t darkresult; /* Smoke Dark result*/
	uint32_t brightresult; /* Smoke bright result*/
} SmokeResult;

/**
 * Buzzer fault
 * @brief This result will provide different type of buzzer faults
 */
typedef struct {
	bool FB_short_to_VDD; /*  FB short to VDD*/
	bool FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS; /* FB open, HB open, FB short to VSS or HS, HB short to VSS or HS*/
	bool HS_open_or_shorted_to_VSS; /*HS open or shorted to VSS*/
	bool HS_short_to_VDD; /* HS short to VDD*/
	bool FB_to_HB_short; /*FB to HB short*/
	bool HB_short_to_VDD; /*HB short to VDD*/
} buzzerfault;

/**
   * CO fault
   * @brief This result will provide different type of buzzer faults
   */
  typedef enum
    {
     CO_Good_Sensor=0, /*  Good Sensor*/
     CO_Open_Circuit=1, /* CO Open Circuit*/
    CO_Closed_Circuit=2, /*Co Closed Circuit*/
   } Co_fault_state;

/**
 * Buzzer state
 * @brief This  will provide configuration for Buzzer
 */
typedef enum {
	Buzzer_3_wire_init = 0,/* Buzzer 3 wire*/
	Buzzer_2_wire_init,/* Buzzer 2 wire*/
	Buzzer_2_wire_on,/* 2 wireBuzzer on */
	Buzzer_3_wire_on,/* 3 wire Buzzer on*/
	Buzzer_3_wire_off, /* 3 wire Buzzer off*/
	Buzzer_2_wire_off,/* 2 wire Buzzer off*/
	Buzzer_3_wire_deinit,/* 3 wire Buzzer disable*/
	Buzzer_2_wire_deinit,/* 2 wire Buzzer disable*/
	END_Buzzer
} Buzzer_State;

/**
 *Buzzer Fault
 * @brief This  will provide configuration for Buzzer
 */
typedef enum {
	No_fault = 0, fault, END_fault
} Buzzer_Fault_State;


typedef struct
{
    uint32_t afe_adc_data;
}AFERspMessage_t;


/* Battery measurement global variables */
/* Battery measurement global variables */

extern OS_TCB AFETasktcb;
/***************************************************************************//**
 * Initialise IADC for single high accuracy conversion.
 ******************************************************************************/

void keep_CO_powered_up(void);
bool is_Buzzer_Off();
void hal_AFE_Post(AFE_setup_t setup, uint8_t *bist_result, SmokeResult *result, bool isHighPrio);
void hal_AFE_ADC_Setup(AFE_setup_t setup);
uint32_t Measure_ADC(void);
void hal_AFE_delay_ms(uint16_t time_ms_val);
void hal_AFE_delay_us(uint16_t time_ms_val);
void hal_AFE_delay_ms(uint16_t time_ms_val);
void EnableCO_withshunt_on(void);
void DisableCO_withshunt_on(void);
void hal_AFE_delay_us(uint16_t time_ms_val);
uint32_t get_battery_A_Voltage(void);
uint32_t get_battery_B_Voltage(void);
uint32_t get_battery_A_Impedance(void);
uint32_t get_battery_B_Impedance(void);
void Enable_SetHigh(void);
void Enable_SetLow(void);
buzzerfault get_the_buzzer_fault(void);
void hal_AFE_init(void);
void batt_A_Measurement(void);
void batt_B_Measurement(void);
bool batt_circuit_bist(void);
Co_fault_state get_the_co_fault (void);
void set_the_co_Open_circuit_fault ( void);
void set_the_co_Close_circuit_fault (void);
void set_the_buzzer_fault_FB_Short (void);
void set_the_buzzer_fault_FB_Open (void);
void set_the_buzzer_fault_HS_Open (void);
void set_the_buzzer_fault_HS_Short (void);
void set_the_buzzer_fault_FB_HB_Short (void);
void set_the_buzzer_fault_HB_Short_VDD (void);
void hal_AFE_AcquireADC(void);
void hal_AFE_ReleaseADC(void);
uint8_t hal_AFE_HornFaultTest(void);
bool hal_AFE_RegisterResponseVar(AFERspMessage_t *response);

void hal_AFE_dump( void );
void hal_AFE_POR( void );
void hal_IRCAP_Charge(uint16_t charge_time);
void reset_POR( void );

/*******************************************************************************
 * @brief   Correct mV
 *
 * @details This function corrects the given ADC value for ABUF errors
 *
 * @param[in] mV   ADC reading (mV)
 * 
 * @return Corrected mV value
 */
int32_t hal_AFE_abuf_correct( int32_t mV );

#endif /* APP_INC_HAL_AFE_H_ */

