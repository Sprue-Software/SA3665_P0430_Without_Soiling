/********************************************************************************************************
*
* 									GPIO
*
* Filename			: hal_gpio.c
* Version			: V1.00
* Programmer(s)		: ND/UH/ABR/AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "gpiointerrupt.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "os.h"
#include "app.h"
#include "spi_comms.h"
#include "hal_Timer0.h"
#include "hal_AFE.h"
#include "led_buzzer.h"
#include "hal_gpio.h"
#include "board.h"
#include "spi_comms.h"
#include "sl_i2cspm_i2c_config.h"

#include "v3sctrl.h"

/********************************************************************************************************
*********************************************************************************************************
*                                            PROTOTYPES
*********************************************************************************************************
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/
/****************************************************************************************************//**
*                                              GPIO_Init()
*
* @brief	Initialise the GPIO pins
*
* @param	pin		pin number
********************************************************************************************************/
void GPIO_Init(void) {
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* input pins */
    GPIO_PinModeSet(DEF_ADS_PORT, DEF_ADS_PIN, gpioModeInputPullFilter, 1u);/*ADS Button*/
    GPIO_PinModeSet(DEF_SELFTEST_BTN_PORT, DEF_SELFTEST_BTN_PIN, gpioModeInputPullFilter, 1u);			/* Test Button 						*/
    GPIO_PinModeSet(DEF_MCU2_READY_PORT, DEF_MCU2_READY_PIN, gpioModeInput, 0u);
    GPIO_PinModeSet(EUSART1_MISO_PORT, EUSART1_MISO_PIN, gpioModeInput, 0);								/* EUSART1 MISO						*/

    /* output pins */
    GPIO_PinModeSet(DEF_SPI_CS_MCU2_PORT, DEF_SPI_CS_MCU2_PIN, gpioModePushPull, 1u);					/* MCU-2 Chip select 				*/

    GPIO_PinModeSet(DEF_FAULT_LED_PORT, DEF_FAULT_LED_PIN, gpioModePushPull, 0u);						/* Fault LED 						*/
    GPIO_PinModeSet(DEF_CO_LED_PORT, DEF_CO_LED_PIN, gpioModePushPull, 0u);								/* CO LED 							*/
    GPIO_PinModeSet(DEF_SMOKE_LED_PORT, DEF_SMOKE_LED_PIN, gpioModePushPull, 0u);						/* Smoke LED 						*/
    GPIO_PinModeSet(DEF_LXTAL_TUNE_PORT, DEF_LXTAL_TUNE_PIN, gpioModePushPull, 0u);						/* Smoke LED 						*/

    GPIO_PinModeSet(DEF_SOILA_PORT, DEF_SOILA_PIN, gpioModePushPull, 0u);                   /* Deprecated soiling feature */
    GPIO_PinModeSet(DEF_SOILB_PORT, DEF_SOILB_PIN, gpioModePushPull, 0u);                   /* Deprecated soiling feature */
    GPIO_PinModeSet(MCU1_SOILA_ENABLE_PORT, MCU1_SOILA_ENABLE_PIN, gpioModePushPull, 0u);   /* Deprecated soiling feature */
    GPIO_PinModeSet(MCU1_SOILB_ENABLE_PORT, MCU1_SOILB_ENABLE_PIN, gpioModePushPull, 0u);   /* Deprecated soiling feature */

    GPIO_PinModeSet(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN, gpioModePushPull, 0u);								/* Buzzer 							*/
    GPIO_PinModeSet(DEF_SPI_CS_AFE_PORT, DEF_SPI_CS_AFE_PIN, gpioModePushPull, 1u);						/* AFE SPI chip select 				*/
    GPIO_PinModeSet(DEF_HEAT_POWER_PORT, DEF_HEAT_POWER_PIN, gpioModePushPull, 0u);					/* Thermistor power 				*/

    GPIO_PinModeSet(SL_I2CSPM_I2C_SCL_PORT, SL_I2CSPM_I2C_SCL_PIN, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(SL_I2CSPM_I2C_SDA_PORT, SL_I2CSPM_I2C_SDA_PIN, gpioModeWiredAnd, 1);

    GPIO_PinModeSet(DEF_BATTERY_LOAD_DRV_A_PORT, DEF_BATTERY_LOAD_DRV_A_PIN, gpioModePushPull, 0u);		/* Battery A 						*/
    GPIO_PinModeSet(DEF_BATTERY_LOAD_DRV_B_PORT, DEF_BATTERY_LOAD_DRV_B_PIN, gpioModePushPull, 0u);     /* Battery B 						*/

    GPIO_PinModeSet(DEF_LIGHT_SRC_PORT, DEF_LIGHT_SRC_PIN, gpioModePushPull, 0u); 		/* Light src, set low 	*/

    /* Home assistance light */
    GPIO_PinModeSet( DEF_ASSIST_LIGHT_PORT, DEF_ASSIST_LIGHT_RESET_PIN, gpioModePushPull, 0u );   /* Output, off */

    GPIO_PinModeSet(EUSART1_MOSI_PORT, EUSART1_MOSI_PIN, gpioModePushPull, 0);							/* EUSART1 MOSI						*/
    GPIO_PinModeSet(EUSART1_SCLK_PORT, EUSART1_SCLK_PIN, gpioModePushPull, 0);							/* EUSART1 SCLK						*/

    NVIC_EnableIRQ(GPIO_EVEN_IRQn);																		/* enable all IRQs 					*/
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/****************************************************************************************************//**
*                                         GPIO_InitPowerConfig()
*
* @brief	Initialise the power supply gpios
********************************************************************************************************/
void GPIO_InitPowerConfig(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(DEF_HEAT_POWER_PORT, DEF_HEAT_POWER_PIN, gpioModePushPull, 0u); 				    /* Thermistor power, set low		*/

    /* Initialise 3VS supply sharing */
    v3s_ctrl_init( );
}

/****************************************************************************************************//**
*                                            GPIO_PowerOn()
*
* @brief	Set the IO pin high
*
* @param	port	gpio port
* @param	pin		gpio pin
********************************************************************************************************/
void GPIO_PowerOn(uint8_t port, uint8_t pin)
{
	GPIO_PinModeSet(port, pin, gpioModePushPull, 1u);
}

/****************************************************************************************************//**
*                                            GPIO_PowerOff()
*
* @brief	Set the IO pin low
*
* @param	port	gpio port
* @param	pin		gpio pin
********************************************************************************************************/
void GPIO_PowerOff(uint8_t port, uint8_t pin)
{
	GPIO_PinModeSet(port, pin, gpioModeInputPullFilter, 0u); /* switch off the supply */
}

/****************************************************************************************************//**
*                                            GPIO_MCU2ReadyPinStatusGet()
*
* @brief	Get the status of MCU2 Ready Pin
*
* @return	Status of pin (high/low)
********************************************************************************************************/
uint32_t GPIO_MCU2ReadyPinStatusGet() {
	return GPIO_PinInGet(DEF_MCU2_READY_PORT, DEF_MCU2_READY_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_MCU2InterruptEnable()
*
* @brief	Enable MCU2 Ready Interrupt
********************************************************************************************************/
void GPIO_MCU2InterruptEnable(void) {
	GPIO_ExtIntConfig(DEF_MCU2_READY_PORT, DEF_MCU2_READY_PIN, DEF_MCU2_READY_PIN, 0, 1, true);
}

/****************************************************************************************************//**
*                                           GPIO_MCU2InterruptDisable()
*
* @brief	Disable MCU2 Ready Interrupt
********************************************************************************************************/
void GPIO_MCU2InterruptDisable(void) {
	GPIO_ExtIntConfig(DEF_MCU2_READY_PORT, DEF_MCU2_READY_PIN, DEF_MCU2_READY_PIN, 0, 0, false);
}

/****************************************************************************************************//**
*                                            GPIO_MCU2CSLowSet()
*
* @brief	Set MCU2 CS Pin Low
********************************************************************************************************/
void GPIO_MCU2CSLowSet(void) {
	GPIO_PinOutClear(DEF_SPI_CS_MCU2_PORT, DEF_SPI_CS_MCU2_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_MCU2CSHighSet()
*
* @brief	Set MCU2 CS Pin High
********************************************************************************************************/
void GPIO_MCU2CSHighSet(void) {
	GPIO_PinOutSet(DEF_SPI_CS_MCU2_PORT, DEF_SPI_CS_MCU2_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_AFECSLowSet()
*
* @brief	Set AFE CS Pin Low
********************************************************************************************************/
void GPIO_AFECSLowSet(void) {
	GPIO_PinOutClear(DEF_SPI_CS_AFE_PORT, DEF_SPI_CS_AFE_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_AFECSHighSet()
*
* @brief	Set AFE CS Pin High
********************************************************************************************************/
void GPIO_AFECSHighSet(void) {
	GPIO_PinOutSet(DEF_SPI_CS_AFE_PORT, DEF_SPI_CS_AFE_PIN);
}

/****************************************************************************************************//**
*                                           GPIO_TurnSmokeLEDOn()
*
* @brief	Turn Smoke LED On
********************************************************************************************************/
void GPIO_TurnSmokeLEDOn(void) {
	GPIO_PinOutSet(DEF_SMOKE_LED_PORT, DEF_SMOKE_LED_PIN);
}

/****************************************************************************************************//**
*                                          GPIO_TurnSmokeLEDOff()
*
* @brief	Turn Smoke LED Off
********************************************************************************************************/
void GPIO_TurnSmokeLEDOff(void) {
	GPIO_PinOutClear(DEF_SMOKE_LED_PORT, DEF_SMOKE_LED_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_TurnCOLEDOn()
*
* @brief	Turn CO LED On
********************************************************************************************************/
void GPIO_TurnCOLEDOn(void) {
	GPIO_PinOutSet(DEF_CO_LED_PORT, DEF_CO_LED_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_TurnCOLEDOff()
*
* @brief	STurn CO LED Off
********************************************************************************************************/
void GPIO_TurnCOLEDOff(void) {
	GPIO_PinOutClear(DEF_CO_LED_PORT, DEF_CO_LED_PIN);
}

/****************************************************************************************************//**
*                                            GPIO_TurnFaultLEDOn()
*
* @brief	Turn Fault LED On
********************************************************************************************************/
void GPIO_TurnFaultLEDOn(void) {
	GPIO_PinOutSet(DEF_FAULT_LED_PORT, DEF_FAULT_LED_PIN);
}

/****************************************************************************************************//**
*                                           GPIO_TurnFaultLEDOff()
*
* @brief	Turn Fault LED Off
********************************************************************************************************/
void GPIO_TurnFaultLEDOff(void) {
	GPIO_PinOutClear(DEF_FAULT_LED_PORT, DEF_FAULT_LED_PIN);
}

/**************************************************************************//**
 * @brief Enable ambient light_power
 *****************************************************************************/
void hal_gpio_enable_ambient_light_power(void)
{
  GPIO_PinOutSet(DEF_LIGHT_SRC_PORT, DEF_LIGHT_SRC_PIN);
}

/**************************************************************************//**
 * @brief Disable ambient light_power
 *****************************************************************************/
void hal_gpio_disable_ambient_light_power(void)
{
  GPIO_PinOutClear(DEF_LIGHT_SRC_PORT, DEF_LIGHT_SRC_PIN);
}
