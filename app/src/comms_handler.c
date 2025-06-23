/**
 * @file
 * @remarks compiler preprocessor option DEBUG_MSG_ADD_LF to enable LF at enf of each debug_out message
 * @remarks compiler preprocessor option FCT_BUILD. Used to distinguish between BIST and FCT mode
 */
#include  <cpu/include/cpu.h>
#include "em_core.h"
#include "em_eusart.h"
#include "em_ldma.h"
#include "dmadrv.h"

#include "hal_gpio.h"
#include "em_cmu.h"
#include "cmsis_os2.h"
#include "comms_handler.h"
#include "hal_BURTCTimer.h"
#include "hal_AFE.h"
#include "P0200_FTM.h"

#include <stdio.h>
#include <stdarg.h>
#include "uartCLI.h"

#define STW_EUART_UNUSE_CLOSED

#if defined(FTM_BUILD)
#include "crc.h"
#endif

#ifdef FTM_BUILD
#include "P0200_FTM.h"
#endif

/*********************************************** Local DEFINES *************************************************/
#if defined DEBUG_BUILD 
#define DEBUG_LOGGING_EVENT   (1u << 0u)
#endif

#ifdef FTM_BUILD
#define COMMS_UART_TX     (1u << 2u)
#define COMMS_UART_RX     (1u << 3u)
#define COMMS_UART_XMODEM   (1u << 4u)
#endif 

#if defined DEBUG_BUILD && defined FTM_BUILD
#define COMMS_EVENT_ALL     (DEBUG_LOGGING_EVENT | COMMS_UART_TX | COMMS_UART_RX | COMMS_UART_XMODEM)
#elif defined DEBUG_BUILD
#define COMMS_EVENT_ALL     (DEBUG_LOGGING_EVENT)
#elif defined FTM_BUILD
#define COMMS_EVENT_ALL (COMMS_UART_TX | COMMS_UART_RX | COMMS_UART_XMODEM)
#endif

#ifdef FTM_BUILD
#define XMODEM_RX_BUFFER_SIZE     8u
#define XMODEM_CMD_EOT            0x04u /**< XModem end message (transmition)*/
#define XMODEM_CMD_ACK            0x06u /**< XModem message acknowledge*/
#define XMODEM_CMD_NAK            0x15u /**< XModem message NOT acknowledge*/
#define XMODEM_CMD_C              0x43u /**< ASCII 'C' indicator of XModem CRC16 start of message*/
#define XMODEM_CMD_SOH            0x01u /**< XModem start of message*/
#define XMODEM_RETRY_COUNT_LIMIT  0x0Au /**< Maximum number of retries before we cancel the transfert*/
#define XMODEM_PACKAGE_DATA_SIZE  128u  /**< The size of the data in a XModem package*/
#endif

#if defined DEBUG_BUILD
#ifndef FTM_BUILD
#error If DEBUG_BUILD is defined, FTM_BUILD has to be defined for CLI to work
#endif
#endif

/*********************************************** Local typedefs *************************************************/
#ifdef FTM_BUILD

/**
 * XMODEM states
 */
typedef enum {
  XMODEM_IDLE, /**< XMODEM_IDLE*/
  XMODEM_WAIT_FOR_C, /**< XMODEM_WAIT_FOR_C*/
  XMODEM_WAIT_FOR_ACK, /**< XMODEM_WAIT_FOR_ACK*/
  XMODEM_WAIT_FOR_EOT_ACK/**< XMODEM_WAIT_FOR_EOT_ACK*/
} xmodemState_t;
#endif

/*********************************************** Static Variables *************************************************/
static AFERspMessage_t UartTaskResponse;

#if defined FTM_BUILD || defined DEBUG_BUILD
static OS_SEM SEM_LDMAtxComplete; /*Semaphore for LDMA transfer complete*/
#endif

#ifdef DEBUG_BUILD
OS_SEM SerialTX_Sema;
static msgBuffer_t debugMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
#endif


#ifdef FTM_BUILD
static volatile xmodemState_t xModemState = XMODEM_IDLE;
static msgBuffer_t receiveMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
static msgBuffer_t transmitMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
static xModemCallback_t xModemCallback = NULL;
static uint8_t xModemPacketId = 0u;
static uint8_t xModemRxBuffer[XMODEM_RX_BUFFER_SIZE];
static uint8_t xModemRxBufferReadIndex = 0u;
static uint8_t xModemRxBufferWriteIndex = 0u;
#endif


/*********************************************** Static Function Declaration **************************************/

#if defined FTM_BUILD 
static void sendUartPacket(const commsMsg_t* message);
static void sendUartPackets(void);
#endif


#ifdef FTM_BUILD
static void xModemCommsHandler(void);
static bool xModemSendPackage(const uint8_t packetId, const bool resendPacket); 
static void xModemSendEOT(void);
#endif

#ifdef DEBUG_BUILD
static uint8_t nibbleToChar(const uint8_t Nibble); 
static void sendDebugPacket(void);
#endif

#if defined FTM_BUILD || defined DEBUG_BUILD
static void dma_tx_callback(void);
static void eusart_send_data(uint8_t string[], const uint8_t dataLen);
#endif

/*********************************************** Global Variables *************************************************/
#ifdef DEBUG_BUILD
extern bool UARTCLI_IsCliModeActive(void);
#endif

#if defined FTM_BUILD || defined DEBUG_BUILD
unsigned int dmaTxChannel = 4u;
/*task variables*/
OS_TCB CommsTaskTCB;
OS_FLAG_GRP CommsEventFlags;
#endif


#ifdef FTM_BUILD
nextGenCommsDriverInterface_t ngCommsDriver =
    {
        .isProtocolActive = false,
        .timeoutReset = resetProtocolTimoutTimer,
        .sendPacket = sendUartPacket,
        .commandTableSize = P0200_FTM_COMMAND_COUNT,
        .commandTable = P0200_FTMCommandTable

    };
#endif

static bool comms_app_init = false;

/***************************************************** Static Function definition **********************************/

#ifdef FTM_BUILD

static void xModemSendEOT(void)
{
  uint8_t  EOT[1u] = {XMODEM_CMD_EOT};
  eusart_send_data(EOT, 1);
}


static bool xModemSendPackage(const uint8_t packetId, const bool resendPacket) {
  static uint8_t xModemPackage[TX_BUFFER_SIZE] = {0};
  static bool moreDataToSend = false;
  if (!resendPacket) {
    xModemPackage[0u] = XMODEM_CMD_SOH;
    xModemPackage[1u] = packetId;
    xModemPackage[2u] = 255u - packetId;
    moreDataToSend = xModemCallback(packetId, &xModemPackage[3U]); /*Get the data for the datapack*/
    const uint16_t crc = CRC_UartCalculate(&xModemPackage[3U], XMODEM_PACKAGE_DATA_SIZE);
    xModemPackage[3u + XMODEM_PACKAGE_DATA_SIZE] = (uint8_t) ((crc >> 8u) & 0xFFu);
    xModemPackage[3u + XMODEM_PACKAGE_DATA_SIZE + 1u] = (uint8_t) (crc & 0xFFu);
  }
  eusart_send_data(xModemPackage, TX_BUFFER_SIZE);

  return moreDataToSend; //should be return from be callback function denoting of there is any data left*/
}


static void xModemCommsHandler(void) {
	if (xModemRxBufferReadIndex != xModemRxBufferWriteIndex) {
		uint8_t receivedByte = xModemRxBuffer[xModemRxBufferReadIndex];
		xModemRxBufferReadIndex = (xModemRxBufferReadIndex + 1u) % XMODEM_RX_BUFFER_SIZE;
		static uint8_t retryCounter = 0;
		static bool moreDataToSend = false;
		switch (xModemState) {
			case XMODEM_WAIT_FOR_C:
				if (receivedByte == XMODEM_CMD_C) {
					retryCounter = 0u;
					xModemPacketId = 1u;
					moreDataToSend = xModemSendPackage(xModemPacketId, false);
					xModemState = XMODEM_WAIT_FOR_ACK;
				}
				break;
			case XMODEM_WAIT_FOR_ACK:
				if (receivedByte == XMODEM_CMD_ACK) {
					if (moreDataToSend) {
						if (xModemPacketId < (xModemPacketId + 1)) {
							xModemPacketId++;
							retryCounter = 0u;
							moreDataToSend = xModemSendPackage(xModemPacketId, false);
						} else {
							/*To many packages > 255*/
							xModemModeExit();
						}
					} else {
						xModemSendEOT();
						xModemState = XMODEM_WAIT_FOR_EOT_ACK;
					}
				} else if (receivedByte == XMODEM_CMD_NAK) {
					if (retryCounter > XMODEM_RETRY_COUNT_LIMIT) {
						xModemModeExit();
					} else {
						moreDataToSend = xModemSendPackage(xModemPacketId, true);
						retryCounter++;
					}
				} else {
					/*Unexpected value*/
					xModemModeExit();
				}
				break;
			case XMODEM_WAIT_FOR_EOT_ACK:
				if ((receivedByte == XMODEM_CMD_ACK) || (retryCounter > XMODEM_RETRY_COUNT_LIMIT)) {
					xModemModeExit();
				} else {
					(void) xModemSendPackage(xModemPacketId, true);
				}
				break;
			case XMODEM_IDLE:
			default:
				/*We should never get here, however if we do set xModemState to IDLE*/
				xModemModeExit();
				break;
		}
	}
}
#endif

#if defined FTM_BUILD

static void sendUartPackets(void) {
  static uint8_t uartOutBuffer[TX_BUFFER_SIZE] = {0u};
  uint8_t i = 0u;

  while ((transmitMsgBuffer.readIndex != transmitMsgBuffer.writeIndex) &&
       ((i + transmitMsgBuffer.buffer[transmitMsgBuffer.readIndex].bufferLength) < TX_BUFFER_SIZE)) {
    const commsMsg_t uartMsg = transmitMsgBuffer.buffer[transmitMsgBuffer.readIndex];
    for (uint8_t x = 0u; x < uartMsg.bufferLength; x++) {
      uartOutBuffer[i] = uartMsg.buffer[x];
      i++;
    }
    transmitMsgBuffer.readIndex = (transmitMsgBuffer.readIndex + 1u) % NG_MSG_BUFFER_COUNT; /*Update read pointer*/
  }
  eusart_send_data(uartOutBuffer, i);
}

static void sendUartPacket(const commsMsg_t* message) {
  RTOS_ERR err;
  /*Temporary prevent OS from scheduling to other tasks, as this would end up with a possible deadlock*/
  OSSchedLock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  memcpy(&transmitMsgBuffer.buffer[transmitMsgBuffer.writeIndex], message, sizeof(commsMsg_t));
  transmitMsgBuffer.writeIndex = (transmitMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT;
  OSSchedUnlock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
            COMMS_UART_TX, OS_OPT_POST_FLAG_SET,    //Set the flag
            &err);
}

#endif

#if defined FTM_BUILD || defined DEBUG_BUILD

/**
 * @brief dma tx completion callback, signals the TX is done and buffer can be filled again
 * @post SEM_LDMAtxComplete Post
 */
static void dma_tx_callback(void) {
  RTOS_ERR err;

  /*Post semaphore to signal buffer has been transmitted and can be filled again when required*/
  (void) OSSemPost(&SEM_LDMAtxComplete, /* Pointer to user-allocated semaphore.    */
           OS_OPT_POST_ALL, /*  POST to ALL tasks that are waiting on the semaphore.     */
           &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);/*   Check error code.  */
#ifdef DEBUG_BUILD
  if(false == UARTCLI_IsCliModeActive())
  {
      //If not al messages are send we need to repost the DEBUG_LOGGING_EVENT
      if (debugMsgBuffer.readIndex != debugMsgBuffer.writeIndex) {
        (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                  DEBUG_LOGGING_EVENT, OS_OPT_POST_FLAG_SET,    //Set the flag
                  &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
      }
#endif

#ifdef FTM_BUILD
      //If not al messages are send we need to repost the UART_TX event
      if (transmitMsgBuffer.readIndex != transmitMsgBuffer.writeIndex) {
        (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                  COMMS_UART_TX, OS_OPT_POST_FLAG_SET,    //Set the flag
                  &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
      }
#endif
#ifdef DEBUG_BUILD
  }
#endif
}

/**
 * Transmit data via the EUSART/LDMA. Pends on semaphore to ensure all data in LDMA buffers has been send
 * @param string Data to transmit. !!! Can't be a variable that goes out of scope
 * @param dataLen Length of data to transmit, if data is longer than TX_BUFFER_SIZE it is truncated.
 */
static void eusart_send_data(uint8_t string[], const uint8_t dataLen) {
  uint8_t sizeToTransmit = (dataLen > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : dataLen;

  RTOS_ERR err;

  /* Acquire resource protected by semaphore.*/
  (void) OSSemPend(&SEM_LDMAtxComplete, /* Pointer to user-allocated semaphore.*/
           0u, /* Wait indefinitely.*/
           OS_OPT_PEND_BLOCKING, /* Task will block.*/
           NULL, /* Timestamp is not used.*/
           &err);
  if (err.Code == RTOS_ERR_NONE) {

#ifndef DEBUG_BUILD
#ifdef STW_EUART_UNUSE_CLOSED
      if(getDefaultUART() == true)
      {
      GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
      GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;
      }
      else
      {
              /*Do nothing*/
      }
#endif
#endif

    /* Resource acquired. */
    (void) DMADRV_MemoryPeripheral(dmaTxChannel, dmadrvPeripheralSignal_EUSART0_TXBL, (void *) &(EUSART0->TXDATA),
                     (void *) &string[0u], true, sizeToTransmit, dmadrvDataSize1,
                     (void *) dma_tx_callback, NULL);
  }
}
#endif

#ifdef DEBUG_BUILD
/**
 * @brief Build debug message into a packet and send via the LEUART
 */
static void sendDebugPacket(void)
{
  static uint8_t debugOutBuffer[ TX_BUFFER_SIZE ] = { 0u };

  uint8_t i = 0u;

  while( ( debugMsgBuffer.readIndex != debugMsgBuffer.writeIndex ) && ( ( i + debugMsgBuffer.buffer[ debugMsgBuffer.readIndex ].bufferLength ) < TX_BUFFER_SIZE ) )
  {
    const commsMsg_t debugMsg = debugMsgBuffer.buffer[ debugMsgBuffer.readIndex ];

    for (uint8_t x = 0u; x < debugMsg.bufferLength; x++)
    {
      debugOutBuffer[i] = debugMsg.buffer[x];
      i++;
    }

    debugMsgBuffer.readIndex = (debugMsgBuffer.readIndex + 1u) % NG_MSG_BUFFER_COUNT; /*Update read pointer*/
  }
  eusart_send_data(debugOutBuffer, i);
}

/**
 * @brief Converts the lowest 4-bit (nibble) of the given value to a ASCII HEX char
 * @param Nibble The nibble to convert to ASCII HEX
 * @return The converted nibble as ASCII HEX '0'..'9' - 'A'..'F'
 */
static uint8_t nibbleToChar(const uint8_t Nibble) {
  uint8_t maskedNibble = Nibble & 0x0Fu;

  return (maskedNibble <= 9u) ? (uint8_t) (maskedNibble + (uint8_t) '0') : (maskedNibble - 10u + (uint8_t) 'A');
}
#endif

/******************************************************** Global Function definition **********************************/

#ifdef DEBUG_BUILD
//SK: @Note need to investigate the DMA issue. Once Solved, need to delete this blocking DMA TX call
void eusart_send_data_blocking(uint8_t string[]) 
{
  uint16_t dataLen = strlen(string);
  uint8_t sizeToTransmit = (dataLen > TX_BUFFER_SIZE) ? TX_BUFFER_SIZE : dataLen;

  RTOS_ERR err;

  /* Acquire resource protected by semaphore.*/
  (void) OSSemPend(&SEM_LDMAtxComplete, /* Pointer to user-allocated semaphore.*/
           0u, /* Wait indefinitely.*/
           OS_OPT_PEND_BLOCKING, /* Task will block.*/
           NULL, /* Timestamp is not used.*/
           &err);
  if (err.Code == RTOS_ERR_NONE) {
    /* Resource acquired. */
    (void) DMADRV_MemoryPeripheral(dmaTxChannel, dmadrvPeripheralSignal_EUSART0_TXBL, (void *) &(EUSART0->TXDATA),
                     (void *) &string[0u], true, sizeToTransmit, dmadrvDataSize1,
                     (void *) dma_tx_callback, NULL);
  }
  bool dmaActive;
  while(true)
  {
      DMADRV_TransferActive(dmaTxChannel, &dmaActive);
      if (!dmaActive) 
      {
          break;
      }
      OSTimeDly(2, OS_OPT_TIME_DLY, &err);
  }
}
#endif



#ifdef FTM_BUILD 
uint16_t nextGenComms_CalculateCRC(const uint8_t buffer[], const uint16_t length)
{
  return CRC_UartCalculate(buffer, length);
}

bool xModemModeEnter(xModemCallback_t callback) {
  bool result = callback != NULL;
  if (result) {
    xModemCallback = callback;
    xModemState = XMODEM_WAIT_FOR_C;
  }
  return result;
}

void xModemModeExit(void) {
  xModemCallback = NULL;
  xModemState = XMODEM_IDLE;
}
#endif

#if defined FTM_BUILD || defined DEBUG_BUILD

void EUSART0_RX_IRQHandler(void) {
  static bool canWriteToRx = false;
  static bool canReceive = true;
#if defined FTM_BUILD 
  static volatile commsMsg_t *currentRxBuffer = &receiveMsgBuffer.buffer[0u];
#endif
  CORE_DECLARE_IRQ_STATE;
  OSIntEnter();
  CORE_ENTER_ATOMIC();
  volatile uint32_t pendingInterrupts = EUSART_IntGet(EUSART0);
  const uint8_t inComingByte = EUSART0->RXDATA; /*Always read the byte or interrupts keep firing*/
#ifdef DEBUG_BUILD
  if(true != UARTCLI_IsCliModeActive())
  {
#endif
#ifdef FTM_BUILD
    if (xModemState != XMODEM_IDLE) {
      if (((xModemRxBufferWriteIndex + 1u) % XMODEM_RX_BUFFER_SIZE) != xModemRxBufferReadIndex) {
        xModemRxBuffer[xModemRxBufferWriteIndex] = inComingByte;
        xModemRxBufferWriteIndex = (xModemRxBufferWriteIndex + 1u) % XMODEM_RX_BUFFER_SIZE;
        RTOS_ERR err;
        (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                          COMMS_UART_XMODEM,
                          OS_OPT_POST_FLAG_SET,    //Set the flag
                          &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
      }
      else
      {
        xModemModeExit(); /*We should never get here, but if we do exit xModem mode*/
      }
      EUSART_IntClear(EUSART0, pendingInterrupts);

    }
    else
    {

      if((pendingInterrupts & EUSART_IF_SIGF) == EUSART_IF_SIGF)
      {
        EUSART_IntClear(EUSART0, EUSART_IF_SIGF | EUSART_IF_RXFL);

        if(((receiveMsgBuffer.writeIndex + 1) % NG_MSG_BUFFER_COUNT) != receiveMsgBuffer.readIndex)
        {
          canReceive = true;
          if(canWriteToRx)
          { /*Only fire pending flag if we were receiving data*/
            receiveMsgBuffer.writeIndex = ((receiveMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT);
            RTOS_ERR err;
            (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                              COMMS_UART_RX, OS_OPT_POST_FLAG_SET,    //Set the flag
                              &err);
            APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
          }
        }
        else
        {
          canReceive = false;
        }

        canWriteToRx = false;
      }
      else if((pendingInterrupts & EUSART_IF_STARTF) == EUSART_IF_STARTF)
      {
        EUSART_IntClear(EUSART0, EUSART_IF_STARTF | EUSART_IF_RXFL);

        /*Only allow writing to RX buffer if there is space left*/
        if(canReceive)
        {
          currentRxBuffer = &receiveMsgBuffer.buffer[receiveMsgBuffer.writeIndex];
          /*Reset the current write pointer in the current rx data-buffer if buffer is not full*/
          currentRxBuffer->bufferLength = 0u;
          canWriteToRx = true;
        }/*clear the pending flag*/
      }

      else if((pendingInterrupts & EUSART_IF_RXFL) == EUSART_IF_RXFL)
      {
        EUSART_IntClear(EUSART0, EUSART_IF_RXFL);
        if(canWriteToRx)
        {
          if(currentRxBuffer->bufferLength < NG_MSG_BUFFER_SIZE)
          {
            currentRxBuffer->buffer[currentRxBuffer->bufferLength] = inComingByte;
            currentRxBuffer->bufferLength++;
          }
          else
          {
            //Not enough room, set overflow flag and UART_RX flag
            canWriteToRx = false;
            currentRxBuffer->bufferOverflow = true;
          }
        }
        /*else ignore incoming byte*/
      }
      else
      {
        /*MISRA - do noting*/
      }
    }
#endif
#ifdef DEBUG_BUILD
  }
  else
  {
      UARTCLI_RxData(inComingByte);
      EUSART_IntClear(EUSART0, pendingInterrupts);
  }
#endif
  EUSART_IntClear(EUSART0, pendingInterrupts &
               ~(EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF)); //Clear all unused interrupt flags
  CORE_EXIT_ATOMIC();

  OSIntExit();

}

#ifndef DEBUG_BUILD
  #ifdef STW_EUART_UNUSE_CLOSED
  void EUSART0_TX_IRQHandler(void)
  {
    CORE_DECLARE_IRQ_STATE;
    OSIntEnter();
    CORE_ENTER_ATOMIC();

    if(getDefaultUART() == true)
    {
    volatile uint32_t pendingInterrupts = EUSART_IntGet(EUSART0);

    if ((pendingInterrupts & EUSART_IF_TXIDLE) == EUSART_IF_TXIDLE)
    {
      GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModeWiredAnd, 0u);
      GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN;
     
      EUSART_IntClear(EUSART0, EUSART_IF_TXIDLE);
    }
    }

    CORE_EXIT_ATOMIC();
    OSIntExit();
  }
  #endif
#endif

/**
 * @brief DMA driver initialisation
 */
void commsAppInit(bool enable) {
  uint32_t status;
  RTOS_ERR err;
  static bool clocksenable = true;
#ifdef FTM_BUILD
	xModemState = XMODEM_IDLE;
#endif
  /*USART0 configuration and initialisation*/
  /*Setup of the GPIO pins*/

	if(clocksenable == true)
	{
	    CMU_ClockEnable(cmuClock_GPIO, true);
	}

  GPIO_PinModeSet(EUSART0_RX_PORT, EUSART0_RX_PIN, gpioModeInputPull, 0u);
#ifdef DEBUG_BUILD
  if(enable == true)
  {
      GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
      GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN | GPIO_EUSART_ROUTEEN_RXPEN;
  }
  else
  {
      GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
      GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN;
  }
#else
#ifdef STW_EUART_UNUSE_CLOSED
  if(enable == true)
  {
  GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModeWiredAnd, 0u);
  GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN;
  }
  else
  {
      GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
      GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN;

  }
#else
  GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1u);
  GPIO->EUSARTROUTE[0u].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN | GPIO_EUSART_ROUTEEN_RXPEN;
#endif
#endif
  GPIO->EUSARTROUTE[0u].TXROUTE =
      (EUSART0_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT) | (EUSART0_TX_PIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[0u].RXROUTE =
      (EUSART0_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT) | (EUSART0_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  /*Setup USART0*/
  if(clocksenable == true)
  {
      CMU_ClockSelectSet(cmuClock_EUSART0, cmuSelect_LFRCO);
      CMU_ClockEnable(cmuClock_EUSART0, true);
  }
  EUSART_UartInit_TypeDef usart0InitConfig = EUSART_UART_INIT_DEFAULT_LF;

#ifdef DEBUG_BUILD
  if(clocksenable == true)
  {
  EUSART_AdvancedInit_TypeDef advance_init = EUSART_ADVANCED_INIT_DEFAULT;
  usart0InitConfig.advancedSettings = &advance_init;
  usart0InitConfig.advancedSettings->dmaWakeUpOnRx = true;
  usart0InitConfig.advancedSettings->dmaWakeUpOnTx = true;
  usart0InitConfig.advancedSettings->dmaHaltOnError = true;
  }
#endif

  if(clocksenable == true)
  {
  EUSART_UartInitLf(EUSART0, &usart0InitConfig);
  EUSART_Enable(EUSART0, false);
  EUSART0->STARTFRAMECFG = (uint32_t) STX;
  EUSART0->SIGFRAMECFG = (uint32_t) ETX;
  EUSART_IntClear(EUSART0, 0xFFFFFFFFu);
  }

#ifdef DEBUG_BUILD
  if(clocksenable == true)
  {
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
  }
#else
#ifdef STW_EUART_UNUSE_CLOSED
  if(enable == true)
  {
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF | EUSART_IF_TXIDLE);
  }
  else
  {
      EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
  }
#else
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
#endif
#endif


  if(clocksenable == true)
  {
	  /*Configure the USART0 DMA TX*/
	  Ecode_t dmaInit = DMADRV_Init();
	  EFM_ASSERT(dmaInit == ECODE_OK);
	  // Allocate channels for transmission and reception
	  status = DMADRV_AllocateChannel(&dmaTxChannel, NULL);
	  EFM_ASSERT(status == ECODE_EMDRV_DMADRV_OK);
	
	  /* Create semafore for DMA Complete*/
	  char cLDMATxDoneSemaphoreString[] = "LDMA_TX_Done";
	  OSSemCreate(&SEM_LDMAtxComplete, /* Pointer to user-allocated semaphore.*/
	        &cLDMATxDoneSemaphoreString[0u], /* Name used for debugging.*/
	        1u, /*Initial count: available in this case.*/
	        &err);
	  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
	#ifdef DEBUG_BUILD
	  char SetialTxSemaName[] = "Serial Tx Semaphore";
	  OSSemCreate(&SerialTX_Sema, &SetialTxSemaName[0u], 1, &err);
	  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
	#endif
	
		/* create comm event flag */
		OSFlagCreate(&CommsEventFlags, /*   Pointer to user-allocated event flag.         */
		"CommTaskEventFlags", /*   Name used for debugging.                  */
		0, /*   Initial flags, all cleared.                   */
		&err);
		/*   Check error code Need to change assert and trigger watchdog     */
		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
  }
  /*Only allow reception when DMA setup is completed*/
  if(clocksenable == true)
  {
  	EUSART_Enable(EUSART0, eusartEnable);
  	NVIC_EnableIRQ(EUSART0_RX_IRQn);
  }

#ifdef DEBUG_BUILD
#else
#ifdef STW_EUART_UNUSE_CLOSED
  if( getDefaultUART() == true)
  {
  	NVIC_EnableIRQ(EUSART0_TX_IRQn);
  }
#endif
#endif

  // Comms app is now ok to use
  comms_app_init = true;
  clocksenable = false;

  DEBUG_APP("Device PowerOn\n", false, 0u);
}


void commsAppInit_FTMMode(bool enter_mode) {
  uint32_t status;
  RTOS_ERR err;
#ifdef FTM_BUILD
	xModemState = XMODEM_IDLE;
#endif
  /*USART0 configuration and initialisation*/
  /*Setup of the GPIO pins*/
  EUSART_UartInit_TypeDef usart0InitConfig = EUSART_UART_INIT_DEFAULT_LF;

  if (enter_mode == true)
  {
	  EUSART_AdvancedInit_TypeDef advance_init = EUSART_ADVANCED_INIT_DEFAULT;
	  usart0InitConfig.advancedSettings = &advance_init;
	  usart0InitConfig.advancedSettings->dmaWakeUpOnRx = true;
	  usart0InitConfig.advancedSettings->dmaWakeUpOnTx = true;
	  usart0InitConfig.advancedSettings->dmaHaltOnError = true;
  }

  EUSART_UartInitLf(EUSART0, &usart0InitConfig);
  EUSART_Enable(EUSART0, false);
  EUSART0->STARTFRAMECFG = (uint32_t) STX;
  EUSART0->SIGFRAMECFG = (uint32_t) ETX;
  EUSART_IntClear(EUSART0, 0xFFFFFFFFu);
#ifdef DEBUG_BUILD
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
#else
#ifdef STW_EUART_UNUSE_CLOSED
  if( getDefaultUART() == true)
  {
  	EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF | EUSART_IF_TXIDLE);
  }
  else
  {
    EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
  }
#else
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);
#endif
#endif

  /*Only allow reception when DMA setup is completed*/
  EUSART_Enable(EUSART0, eusartEnable);
  NVIC_EnableIRQ(EUSART0_RX_IRQn);

#ifdef DEBUG_BUILD
#else
#ifdef STW_EUART_UNUSE_CLOSED
  if( getDefaultUART() == true)
  {
  	NVIC_EnableIRQ(EUSART0_TX_IRQn);
  }
#endif
#endif

  // Comms app is now ok to use
  comms_app_init = true;

  DEBUG_APP("Device PowerOn\n", false, 0u);

}


/**
 * desc UART debug task
 * @param arg not used
 */
void uartCommsTask(void *arg) {
    RTOS_ERR err;
    OS_FLAGS flags;
    const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)arg;
    OSTaskRegSet(DEF_NULL, 0, ptrToMyTCB, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    bool result = hal_AFE_RegisterResponseVar(&UartTaskResponse);
    if(false == result)
    {
        ;
    }

    while (true) {
    flags = OSFlagPend(&CommsEventFlags, /* Pointer to user-allocated event flag. */
               COMMS_EVENT_ALL, /* Flag bitmask to match. */
               0u, /* Wait indefinitely. */
               OS_OPT_PEND_FLAG_SET_ANY | /* Wait until ANY flags are set and */
               OS_OPT_PEND_BLOCKING | /* task will block and */
               OS_OPT_PEND_FLAG_CONSUME, /* consume flags */
               NULL, /* Timestamp is not used. */
               &err);
    /* Check error code. */
    if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
#ifdef FTM_BUILD
      if (xModemState == XMODEM_IDLE) {
        if ((flags & COMMS_UART_RX) == COMMS_UART_RX) {
          while (receiveMsgBuffer.readIndex != receiveMsgBuffer.writeIndex) {
            nextGenComms_HandlePackage(&ngCommsDriver,
                           &receiveMsgBuffer.buffer[receiveMsgBuffer.readIndex]);
            receiveMsgBuffer.readIndex = (receiveMsgBuffer.readIndex + 1u) % NG_MSG_BUFFER_COUNT;
          }
        }

        if ((flags & COMMS_UART_TX) == COMMS_UART_TX) {
          bool dmaActive;
          DMADRV_TransferActive(dmaTxChannel, &dmaActive);
          if (!dmaActive) {
            sendUartPackets();
          } else {
            (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                      COMMS_UART_TX, OS_OPT_POST_FLAG_SET,    //Set the flag
                      &err);
            APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
          }
        }
#endif

#ifdef DEBUG_BUILD
        if ((flags & DEBUG_LOGGING_EVENT) == DEBUG_LOGGING_EVENT)
        {
          /*Only allow to send a message if DMA is not active*/
          bool dmaActive;
        
          DMADRV_TransferActive(dmaTxChannel, &dmaActive);
        
          if (!dmaActive)
          {
            sendDebugPacket();
          }
        }
#endif

#ifdef FTM_BUILD
      } else /*XMODEM Active*/
      {

        /*Make sure we don't lose any messages that need to be handled (RX/TX)*/
        if ((flags & COMMS_UART_RX) == COMMS_UART_RX) {
          (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                    COMMS_UART_RX, OS_OPT_POST_FLAG_SET,    //Set the flag
                    &err);
          APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
        }
        if ((flags & COMMS_UART_TX) == COMMS_UART_TX) {
          (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                    COMMS_UART_TX, OS_OPT_POST_FLAG_SET,    //Set the flag
                    &err);
        }
	      if ((flags & COMMS_UART_XMODEM) == COMMS_UART_XMODEM) {
		      xModemCommsHandler();
	      }
#endif

#ifdef DEBUG_BUILD
        if ((flags & DEBUG_LOGGING_EVENT) == DEBUG_LOGGING_EVENT) {
          (void) OSFlagPost(&CommsEventFlags, /*Pointer to user-allocated event flag.*/
                    DEBUG_LOGGING_EVENT, OS_OPT_POST_FLAG_SET,    //Set the flag
                    &err);
        }
#endif
#ifdef FTM_BUILD
      }
#endif
    }
  }
}

#ifdef DEBUG_BUILD
// @Note: Need to be called after acquiring the Serial Mutex
void FlushDebugMessages(void)
{
  memset(debugMsgBuffer.buffer, 0, sizeof(debugMsgBuffer));
  debugMsgBuffer.writeIndex = debugMsgBuffer.readIndex = 0;
}
#endif

#endif

/**
 * @brief Format and send debug message to the commsHandler to output
 * @details
 * @param str   String to be output
 * @param numMode true, if using \p dataValue; false if \p dataValue is not used
 * @param dataValue   Number to add to message. Number will be output in ASCII representation of hex value
 * @note this is a blocking function
 */
void debug_out(const char str[], const bool numMode, const uint32_t dataValue)
{
  // Comms app must be initialised for debug to work!
  if( comms_app_init == false )
  {
    return;
  }

#ifdef DEBUG_BUILD
  if (false == UARTCLI_IsCliModeActive())
  {
    RTOS_ERR err;

    OSSemPend(&SerialTX_Sema, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    APP_RTOS_ASSERT_DBG(((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)), 1u);

    static const uint8_t nibblesInValue = (sizeof(dataValue) * 2u);

// Max string length is calculated based on max debug message length and if there is a dataValue. Value takes precedence over text
#ifdef DEBUG_MSG_ADD_LF //@Note: SK =============== Please verify it's use and remove it if not needed
    const uint8_t maxStringLength =
        NG_MSG_BUFFER_SIZE - 3u /*Start, Stop, LF*/ - (numMode ? (nibblesInValue + 1u) : 0u);
#else
    const uint8_t maxStringLength = NG_MSG_BUFFER_SIZE - 2u /*Start & Stop*/ - (numMode ? (nibblesInValue + 1u) : 0u);
#endif

    commsMsg_t msgDebug = {.bufferLength = 0u, .buffer = {0u}};

    uint8_t i = 0u;

    msgDebug.buffer[i++] = (uint8_t)'>'; /*Add start of message char*/

    uint8_t y = 0u;

    while ((str[y] != '\0') && (i < maxStringLength))
    {
      msgDebug.buffer[i] = str[y];

      i++;
      y++;
    }

    /*Add value to the message if needed*/
    if (numMode)
    {
      msgDebug.buffer[i] = ' '; /* Add space between text and value */

      uint32_t valueToConvert = dataValue;

      for (uint8_t j = nibblesInValue; j > 0u; j--)
      {
        const uint8_t nibble = valueToConvert & 0x0Fu;

        msgDebug.buffer[i + j] = nibbleToChar(nibble);

        valueToConvert >>= 4u;
      }

      i += (nibblesInValue + 1u);
    }

    msgDebug.buffer[i] = (uint8_t)'<'; /*Add end of message char*/

    i++;
#ifdef DEBUG_MSG_ADD_LF //@Note: SK =============== Please verify it's use and remove it if not needed
    msgDebug.buffer[i] = (uint8_t)'\n';
    i++;
#endif
    msgDebug.bufferLength = i;

    CORE_irqState_t irqState;

    CORE_ENTER_CRITICAL();

    uint8_t posted_msg_count = 0u;

    /* Only allow debug message to be sent if there is room in the buffer.
     * If we have no room in the debug message buffer, when not in round-robin mode,
     * we need to discard the debug message or would end up in deadlock */
    if (((debugMsgBuffer.writeIndex + 1) % NG_MSG_BUFFER_COUNT) != debugMsgBuffer.readIndex)
    {
      memcpy(&debugMsgBuffer.buffer[debugMsgBuffer.writeIndex], &msgDebug, sizeof(commsMsg_t));

      debugMsgBuffer.writeIndex = (debugMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT;

      posted_msg_count++;
    }

    CORE_EXIT_CRITICAL();

    if( posted_msg_count > 0 )
    {
      (void)OSFlagPost(&CommsEventFlags, DEBUG_LOGGING_EVENT, OS_OPT_POST_FLAG_SET, &err);
      APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
    }

    OSSemPost(&SerialTX_Sema, OS_OPT_POST_NONE, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  }
#endif
}

void debug_printf( const char * format, ... )
{
  char buf[ 128 ];

  va_list args;

  va_start( args, format );

  vsprintf( buf, format, args );

  va_end( args );

  debug_out( buf, false, 0ul );
}
