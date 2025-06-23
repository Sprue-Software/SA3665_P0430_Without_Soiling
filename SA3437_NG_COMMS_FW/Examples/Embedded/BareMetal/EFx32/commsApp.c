#include <stdint.h>

#include "em_ldma.h"
#include "em_eusart.h"
#include "em_gpcrc.h"
#include "em_cmu.h"
#include "dmadrv.h"
#include "pin_config.h"

#include "DummyDeviceProtocol.h"

#define NG_EUSART EUSART1

static bool dmaActive = false;
static unsigned int dmaTxChannel = 4u;

static msgBuffer_t receiveMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
static msgBuffer_t transmitMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};

static void sendUartPacket(const commsMsg_t *message);

nextGenCommsDriverInterface_t ngCommsDriver =
		{
				.isProtocolActive = false,
				.timeoutReset = NULL,
				.sendPacket = sendUartPacket,
				.commandTableSize = DUMMY_DEVICE_PROTOCOL_COMMAND_COUNT,
				.commandTable = DummyDeviceProtocolCommandTable

		};

void sendUartPacket(const commsMsg_t *message) {
	transmitMsgBuffer.buffer[transmitMsgBuffer.writeIndex] = *message;
	transmitMsgBuffer.writeIndex = (transmitMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT;
}


uint16_t nextGenComms_CalculateCRC(const uint8_t buffer[], const uint8_t length) {
	CMU_ClockEnable(cmuClock_GPCRC, true);
	GPCRC->CTRL = _GPCRC_CTRL_RESETVALUE;
	GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;

	init.crcPoly = 0x00001021;
	init.initValue = 0x0000;
	init.reverseBits = true;
	init.enable = true;
	GPCRC_Init(GPCRC, &init);

	GPCRC_Start(GPCRC);

	for (uint8_t i = 0; i < length; i++) {            /* add command and data bytes */
		GPCRC_InputU8(GPCRC, buffer[i]);
	}

	uint16_t crcResult = (uint16_t) GPCRC_DataReadBitReversed(GPCRC);     /* return the final CRC value */

	CMU_ClockEnable(cmuClock_GPCRC, false);
	return crcResult;

}

void commsAppInit(void) {

	uint32_t status;

	/*USART1 configuration and initialisation*/
	/*Setup of the GPIO pins*/
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(EUSART1_TX_PORT, EUSART1_TX_PIN, gpioModePushPull, 1u);
	GPIO_PinModeSet(EUSART1_RX_PORT, EUSART1_RX_PIN, gpioModeInputPull, 0u);
	GPIO->EUSARTROUTE[1u].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN;
	GPIO->EUSARTROUTE[1u].TXROUTE =
			(EUSART1_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT) | (EUSART1_TX_PIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
	GPIO->EUSARTROUTE[0u].RXROUTE =
			(EUSART1_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT) | (EUSART1_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
	/*Setup USART0*/
	CMU_ClockSelectSet(cmuClock_EUSART1, cmuSelect_HFXO);
	CMU_ClockEnable(cmuClock_EUSART1, true);
	EUSART_UartInit_TypeDef usart1InitConfig = EUSART_UART_INIT_DEFAULT_HF;
	EUSART_UartInitHf(EUSART1, &usart1InitConfig);
	EUSART_Enable(EUSART1, false);
	EUSART1->STARTFRAMECFG = (uint32_t) STX;
	EUSART1->SIGFRAMECFG = (uint32_t) ETX;
	EUSART_IntClear(EUSART1, 0xFFFFFFFFu);
	EUSART_IntEnable(EUSART1, EUSART_IF_RXFL | EUSART_IF_STARTF | EUSART_IF_SIGF);

	/*Initialise the USART TX DMA channel*/
	status = DMADRV_AllocateChannel(&dmaTxChannel, NULL);
	EFM_ASSERT(status == ECODE_EMDRV_DMADRV_OK);

	/*Only allow reception when DMA setup is completed*/
	EUSART_Enable(EUSART1, eusartEnable);
	NVIC_EnableIRQ(EUSART1_RX_IRQn);
}

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
	(void) DMADRV_MemoryPeripheral(dmaTxChannel, dmadrvPeripheralSignal_EUSART1_TXBL, (void *) &(NG_EUSART->TXDATA),
								   (void *) &uartOutBuffer[0u], true, i, dmadrvDataSize1,
								   NULL, NULL);
}

void commsAppProcess(void) {

	if (transmitMsgBuffer.readIndex != transmitMsgBuffer.writeIndex) {
		/*We can only transmit a package if the DMA is not active*/
		DMADRV_TransferActive(dmaTxChannel, &dmaActive);
		if (!dmaActive) {
			sendUartPackets();
		}
	}

	//check if we received a message and if so handle it
	if (receiveMsgBuffer.readIndex != receiveMsgBuffer.writeIndex) {
		nextGenComms_HandlePackage(&ngCommsDriver,
								   &receiveMsgBuffer.buffer[receiveMsgBuffer.readIndex]);
		receiveMsgBuffer.readIndex = (receiveMsgBuffer.readIndex + 1u) % NG_MSG_BUFFER_COUNT;
	}
}


void EUSART1_RX_IRQHandler(void) {
	static bool canWriteToRx = false;
	static bool canReceive = true;
	static volatile commsMsg_t *currentRxBuffer = &receiveMsgBuffer.buffer[0u];
	volatile uint32_t pendingInterrupts = EUSART_IntGet(NG_EUSART);
	const uint8_t inComingByte = NG_EUSART->RXDATA; /*Always read the byte or interrupts keep firing*/
	if ((pendingInterrupts & EUSART_IF_SIGF) == EUSART_IF_SIGF) {
		EUSART_IntClear(NG_EUSART, EUSART_IF_SIGF | EUSART_IF_RXFL);
		if (((receiveMsgBuffer.writeIndex + 1) % NG_MSG_BUFFER_COUNT) != receiveMsgBuffer.readIndex) {
			canReceive = true;
			if (canWriteToRx) { /*Only fire pending flag if we were receiving data*/
				receiveMsgBuffer.writeIndex = ((receiveMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT);
			}
		} else {
			canReceive = false;
		}
		canWriteToRx = false;
	} else if ((pendingInterrupts & EUSART_IF_STARTF) == EUSART_IF_STARTF) {
		EUSART_IntClear(NG_EUSART, EUSART_IF_STARTF | EUSART_IF_RXFL);
		/*Only allow writing to RX buffer if there is space left*/
		if (canReceive) {
			currentRxBuffer = &receiveMsgBuffer.buffer[receiveMsgBuffer.writeIndex];
			/*Reset the current write pointer in the current rx data-buffer if buffer is not full*/
			currentRxBuffer->bufferLength = 0u;
			canWriteToRx = true;
		}/*clear the pending flag*/
	} else if ((pendingInterrupts & EUSART_IF_RXFL) == EUSART_IF_RXFL) {
		EUSART_IntClear(NG_EUSART, EUSART_IF_RXFL);
		if (canWriteToRx) {
			if (currentRxBuffer->bufferLength < NG_MSG_BUFFER_SIZE) {
				currentRxBuffer->buffer[currentRxBuffer->bufferLength] = inComingByte;
				currentRxBuffer->bufferLength++;
			} else {
				//Not enough room, set overflow flag and UART_RX flag
				canWriteToRx = false;
				currentRxBuffer->bufferOverflow = true;
			}
		}
		/*else ignore incoming byte*/
	} else {
		/*MISRA - do noting*/
	}
	EUSART_IntClear(NG_EUSART, pendingInterrupts &
							   ~(EUSART_IF_RXFL | EUSART_IF_STARTF |
								 EUSART_IF_SIGF)); //Clear all unused interrupt flags
}
