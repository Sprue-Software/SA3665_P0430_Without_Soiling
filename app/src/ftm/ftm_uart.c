/**
 * @file ftm_uart.c
 * @date  29 Jul 2024
 * @author  uhegde
 */


#include "P0200_FTM.h"
#include "hal_gpio.h"

static bool isUARTdefault = true;

/*******************************************************************************
 * @brief set_ftm_uart_factory_test
 * @details enable the special factory UART test mode
 * @param none
 * @return none
 ******************************************************************************/
static void set_ftm_uart_factory_test(void)
{
  commsAppInit(false);
  GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModeWiredAnd, 1u);
  setDefaultUART(false);
}

/*******************************************************************************
 * @brief restore_uart_tx_pin_mode
 * @details restore the default uart tx pin status
 * @param none
 * @return none
 ******************************************************************************/
void restore_uart_tx_pin_mode(void)
{
  commsAppInit(true);
  GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
  setDefaultUART(true);
}

/*******************************************************************************
 * @brief FTM_Uart_FactoryTest
 * @details enable the special factory UART test mode
 * @param const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData
 * @return nextGenCommsAckNackReason_t
 ******************************************************************************/
nextGenCommsAckNackReason_t FTM_Uart_FactoryTest(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData)
{
  (void) message;
  (void) messageSize;
  (void) resultData;
  DEBUG_FTM("\nFTM Uart factory test mode", false, 0U);
  set_ftm_uart_factory_test();
  return NG_NACK_REASON_OK;
}

void setDefaultUART(bool enable)
{
   isUARTdefault = enable;
}

bool getDefaultUART(void)
{
  return isUARTdefault;
}
