/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

/********************************************************************************************************
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
********************************************************************************************************/
#include "board.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/
/* PORT A */
#define DEF_ADS_PORT                (gpioPortA)
#define DEF_ADS_PIN	 						    (0u)

#define DEF_MCU2_READY_PORT         (gpioPortA)
#define DEF_MCU2_READY_PIN          (6u)

#define EUSART0_TX_PORT             (gpioPortA)
#define EUSART0_TX_PIN              (7u)

#define EUSART0_RX_PORT             (gpioPortA)
#define EUSART0_RX_PIN              (8u)

#define DEF_SELFTEST_BTN_PORT				(gpioPortA)
#define DEF_SELFTEST_BTN_PIN				(9u)

/* CAD-B Board */
#define DEF_ASSIST_LIGHT_PORT         (gpioPortA)
#define DEF_ASSIST_LIGHT_RESET_PIN    (10u)

/* PORT B */
#define DEF_LIGHT_SENSE_PORT 			  (gpioPortB)
#define DEF_LIGHT_SENSE_PIN    		  (0u)

#define DEF_LIGHT_SRC_PORT  	 		  (gpioPortB)
#define DEF_LIGHT_SRC_PIN    		    (1u)

#define DEF_SOILA_PORT  	 				  (gpioPortB)
#define DEF_SOILA_PIN 		   				(3u)

#define DEF_SOILB_PORT  	 				  (gpioPortB)
#define DEF_SOILB_PIN 		   				(4u)

#define DEF_HEAT_POWER_PORT 				(gpioPortB)
#define DEF_HEAT_POWER_PIN 					(5u)

#define DEF_3VS_POWER_PORT   				(gpioPortB)
#define DEF_3VS_POWER_PIN	     			(6u)

/* PORT C */
#define DEF_SPI_CS_MCU2_PORT        (gpioPortC)
#define DEF_SPI_CS_MCU2_PIN         (0u)

#define EUSART1_MOSI_PORT					  (gpioPortC)
#define EUSART1_MOSI_PIN  					(1u)

#define EUSART1_MISO_PORT  					(gpioPortC)
#define EUSART1_MISO_PIN   					(2u)

#define EUSART1_SCLK_PORT  					(gpioPortC)
#define EUSART1_SCLK_PIN   					(3u)

#define DEF_FAULT_LED_PORT          (gpioPortC)
#define DEF_FAULT_LED_PIN           (4u)

#define MCU1_SOILA_ENABLE_PORT    	(gpioPortC)
#define MCU1_SOILA_ENABLE_PIN      	(5u)

#define MCU1_SOILB_ENABLE_PORT    	(gpioPortC)
#define MCU1_SOILB_ENABLE_PIN       (6u)

#define DEF_AFE_ENABLE_PORT         (gpioPortC)
#define DEF_AFE_ENABLE_PIN          (7u)

#define DEF_SPI_CS_AFE_PORT         (gpioPortC)
#define DEF_SPI_CS_AFE_PIN          (8u)

#define DEF_CO_LED_PORT             (gpioPortC)
#define DEF_CO_LED_PIN              (9u)

/* PORT D */
#define DEF_LXTAL_TUNE_PORT   				  (gpioPortD)
#define DEF_LXTAL_TUNE_PIN       			  (2u)

#define DEF_BATTERY_LOAD_DRV_A_PORT 		(gpioPortD)
#define DEF_BATTERY_LOAD_DRV_A_PIN  		(3u)

#define DEF_BATTERY_LOAD_DRV_B_PORT 		(gpioPortD)
#define DEF_BATTERY_LOAD_DRV_B_PIN  		(4u)

#define DEF_SMOKE_LED_PORT          		(gpioPortD)
#define DEF_SMOKE_LED_PIN           		(5u)

/* ADC */
#define IADC_INPUT_0_BUS          			BBUSALLOC
#define IADC_INPUT_0_BUSALLOC     			GPIO_BBUSALLOC_BEVEN0_ADC0
#define IADC_INPUT_0_BUSALLOC_ODD   		GPIO_BBUSALLOC_BODD0_ADC0

#define IADC_INPUT_0_PORT_PIN_LIGHT         iadcPosInputPortBPin0;			/* Ambient Light ADC input 	*/
#define IADC_INPUT_0_PORT_PIN_SOIL_A        iadcPosInputPortBPin3;			/* SOIL-A ADC input 		*/
#define IADC_INPUT_0_PORT_PIN_SOIL_B        iadcPosInputPortBPin4;			/* SOIL-B ADC input 		*/



#define DEF_MCU2_READY_PIN_HIGH     (1u)
#define DEF_MCU2_READY_PIN_LOW      (0u)



/********************************************************************************************************
*********************************************************************************************************
*                                               FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

void GPIO_Init(void);
uint32_t GPIO_MCU2ReadyPinStatusGet();
void GPIO_MCU2CSLowSet(void);
void GPIO_MCU2CSHighSet(void);
void GPIO_AFECSLowSet(void);
void GPIO_AFECSHighSet(void);
void GPIO_MCU2InterruptEnable(void);
void GPIO_MCU2InterruptDisable(void);
void GPIO_InitPowerConfig(void);
void GPIO_PowerOn(uint8_t port, uint8_t pin);
void GPIO_PowerOff(uint8_t port, uint8_t pin);
void GPIO_TurnSmokeLEDOn(void);
void GPIO_TurnSmokeLEDOff(void);
void GPIO_TurnCOLEDOn(void);
void GPIO_TurnCOLEDOff(void);
void GPIO_TurnFaultLEDOn(void);
void GPIO_TurnFaultLEDOff(void);
void hal_gpio_enable_ambient_light_power(void);
void hal_gpio_disable_ambient_light_power(void);


/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* _HAL_GPIO_H_ */
