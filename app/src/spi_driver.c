/********************************************************************************************************
*
* 										      SPI DRIVER
*
* Filename			: spi_driver.c
* Version			: V1.00
* Programmer(s)		: AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "em_cmu.h"
#include "em_eusart.h"
#include "hal_gpio.h"
#include "app.h"
#include "spi_comms.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

#define DEF_EUSART1_TX_RX_ENABLE	  (0x00000003u)

#define NOP_COUNTS (25)

/********************************************************************************************************
*********************************************************************************************************
*                                              PROTOTYPES
*********************************************************************************************************
********************************************************************************************************/

void SPIDriver_CheckStatus( void );
static void GPIOINt_callback(uint8_t intNo);

/********************************************************************************************************
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

static inline void nop_delay( void )
{
	for( int i = 0; i < NOP_COUNTS; i++ )
  {
		__NOP();
	}
}


/****************************************************************************************************//**
*                                            GPIOINt_callback()
*
* @brief  Callback function for external interrupts
*
* @param  pin   pin number
********************************************************************************************************/
static void GPIOINt_callback(uint8_t pin) {
  RTOS_ERR err;

#if 1
  if(pin == DEF_MCU2_READY_PIN)                         /* interrupt caused by MCU2         */
  {
    OSFlagPost(&InterMCUSPIComms_CommandsFlag,                      /* post flag to initiate SPI communication  */
               SPI_CMD_Slave_Transfer,
             OS_OPT_POST_FLAG_SET,
             &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
  }
#endif
}
/****************************************************************************************************//**
*                                           SPIDriver_Init()
*
* @brief	Initialise EUSART1 for SPI communication
*
* @note		This function needs to be called at the time of application initialisation.
********************************************************************************************************/
void SPIDriver_Init( void )
{
  CMU_ClockEnable( cmuClock_EUSART1, true );

  EUSART_SpiAdvancedInit_TypeDef adv = EUSART_SPI_ADVANCED_INIT_DEFAULT;

  adv.msbFirst = true;

  EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;

  init.bitRate 						                = 4000000;
  init.advancedSettings 			            = &adv;
  init.advancedSettings->autoCsEnable     = false;

  GPIO->EUSARTROUTE[ 1 ].TXROUTE          = ( EUSART1_MOSI_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT ) |
                                            ( EUSART1_MOSI_PIN  << _GPIO_EUSART_TXROUTE_PIN_SHIFT  );

  GPIO->EUSARTROUTE[ 1 ].RXROUTE          = ( EUSART1_MISO_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT ) |
                                            ( EUSART1_MISO_PIN  << _GPIO_EUSART_RXROUTE_PIN_SHIFT  );

  GPIO->EUSARTROUTE[ 1 ].SCLKROUTE        = ( EUSART1_SCLK_PORT << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT ) |
                                            ( EUSART1_SCLK_PIN  << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT  );

  GPIO->EUSARTROUTE[ 1 ].ROUTEEN          = GPIO_EUSART_ROUTEEN_RXPEN |
                                            GPIO_EUSART_ROUTEEN_TXPEN |
                                            GPIO_EUSART_ROUTEEN_SCLKPEN;

  EUSART_SpiInit( EUSART1, &init );

  
  GPIO_ExtIntConfig(DEF_MCU2_READY_PORT, DEF_MCU2_READY_PIN, DEF_MCU2_READY_PIN, 0, 1, true);		  /* enable falling-edge interrupt	  */
  GPIOINT_CallbackRegister(DEF_MCU2_READY_PIN, GPIOINt_callback);									  /* register callback function 	  */
}

/****************************************************************************************************//**
*                                       SPIDriver_CheckStatus()
*
* @brief	Check the status of EUSART1
*
* @note		This function needs to be called before any spi transaction to enable the peripheral if
*           previously disabled.
********************************************************************************************************/
void SPIDriver_CheckStatus( void )
{
	const uint32_t status = EUSART_StatusGet( EUSART1 );				/* get status of EUSART1	*/

	if( !( status & DEF_EUSART1_TX_RX_ENABLE ) )
  {
		CMU_ClockEnable( cmuClock_EUSART1, true );

		EUSART_Enable( EUSART1, eusartEnable );						        /* Enable TX and RX 		*/
	}
}

/****************************************************************************************************//**
*                                        SPIDriver_Transmit()
*
* @brief   	Transmit SPI data
*
* @param	txBuffer[]	pointer to data to transmit.
*
* @param	len			size of the data to transmit.
********************************************************************************************************/
void SPIDriver_Transmit( uint8_t txBuffer[ ], uint32_t len )
{
  nop_delay( );

	SPIDriver_CheckStatus( );

  for( int i = 0; i < ( int )len; i++ )
  {
  	EUSART_Spi_TxRx( EUSART1, txBuffer[ i ] );
  }

  nop_delay( );
}

/****************************************************************************************************//**
*                                        SPIDriver_Receive()
*
* @brief   	Receive SPI data
*
* @param	rxBuffer[]	pointer to data to receive.
*
* @param	len			size of the data to receive.
********************************************************************************************************/
void SPIDriver_Receive( uint8_t rxBuffer[ ], uint16_t len )
{
  nop_delay( );

	SPIDriver_CheckStatus( );

  for( int i = 0; i < ( int )len; i++ )
  {
  	rxBuffer[ i ] = EUSART_Spi_TxRx( EUSART1, 0x00 );
  }

  nop_delay( );
}

/****************************************************************************************************//**
*                                       SPIDriver_Transfer()
*
* @brief   	Transmit and Receive SPI data
*
* @param	txBuffer[]	pointer to data to transmit.
*
* @param	rxBuffer[]	pointer to data to receive.
*
* @param	len			size of the data to transmit and receive.
********************************************************************************************************/
void SPIDriver_Transfer( uint8_t txBuffer[ ], uint8_t rxBuffer[ ], uint16_t len )
{
  nop_delay( );

	SPIDriver_CheckStatus( );

  for( int i = 0; i < ( int )len; i++ )
  {
  	rxBuffer[ i ] = EUSART_Spi_TxRx( EUSART1, txBuffer[ i ] );
  }

  nop_delay( );
}
