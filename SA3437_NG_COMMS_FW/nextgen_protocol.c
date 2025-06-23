#include <stdbool.h>
#include "nextgen_protocol.h"

#define NG_COMMS_NACK_MSG_LENGTH  9u    /**< Total Length of an unescaped NACK Message*/
#define NG_COMMS_CRC_LENGTH       2u    /**< Length of CRC value in bytes*/
#define ACK                       0x06u /**<Message acknowledge. !!!This is the same as XMODEM_CMD_ACK*/
#define NACK                      0x15u /**<Message not acknowledge. !!!This is the same as XMODEM_CMD_NACK*/
#define ESC                       0x1Bu /**<The escape byte value*/
#define ESC_OFFSET                0x20u /**<Offset added to byte after ESC*/

static bool nextGenComms_DeEscapeMessage(const volatile commsMsg_t *receivedMessage, commsMsg_t *deEscaped);


nextGenCommsAckNackReason_t nextGenComms_CommandNotImplemented(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData) {
	(void) message;
	(void) messageSize;
	(void) resultData;
	return NG_NACK_REASON_CommandNotImplemented;
}

/**
 * @brief Try to find a \p commandId in #bistCommandTable
 * @param commandId The command ID we search for in #bistCommandTable
 * @return SIZE_MAX if command is not found.
 *         Index to the record in the table for the requested \p command
 */
static size_t
FindCommand(const nextGenCommsCommand_t *const commandTable, const size_t commandTable_size, const uint16_t command) {
	size_t low = 0u;
	size_t high = commandTable_size;
	size_t mid = 0u;
	uint16_t midCommand = 0u;
	while ((low <= high) && (high != SIZE_MAX)) {
		mid = ((low + high) / 2u);
		midCommand = commandTable[mid].command;
		if (midCommand == command) {
			break;
		} else if (command > midCommand) {
			low = mid + 1u;
		} else {
			high = mid - 1u;
		}
	}
	return (midCommand == command) ? mid : SIZE_MAX;
}

/**
 * @brief Add escape sequence to the message where applicable.
 * @details Adds STX, ETX to the message and escapes the data giving in \p message
 * @param message [I] The message to escape
 * @param escapedMessage [O] The escaped message ready to be send
 * @return
 */
static void nextGenComms_EscapeMessage(const commsMsg_t *message, commsMsg_t *escapedMessage) {
	const uint16_t messageLength = message->bufferLength;
	escapedMessage->buffer[0u] = STX;
	escapedMessage->bufferLength = 1u;
	for (uint16_t i = 0u; (i < messageLength) && (escapedMessage->bufferLength < (NG_MSG_BUFFER_SIZE - 1u)); i++) {
		const uint8_t currentByte = message->buffer[i];
		if ((currentByte == STX) || (currentByte == ETX) || (currentByte == ESC)) {
			escapedMessage->buffer[escapedMessage->bufferLength] = ESC;
			escapedMessage->bufferLength++;
			escapedMessage->buffer[escapedMessage->bufferLength] = currentByte + ESC_OFFSET;
			escapedMessage->bufferLength++;
		} else {
			escapedMessage->buffer[escapedMessage->bufferLength] = currentByte;
			escapedMessage->bufferLength++;
		}
	}
	escapedMessage->buffer[escapedMessage->bufferLength] = ETX;
	escapedMessage->bufferLength++;
}

/**
 * @brief Sends an ACK response with the given data and command as payload
 * @param driverInterface [I]
 * @param command [I] The command this ack is for
 * @param messageData [I] The response data for this ACK'ed command
 * @return NA
 */
void nextGenComms_SendAckPacket(const nextGenCommsDriverInterface_t *const driverInterface, const uint16_t command,
								const commsMsg_t *messageData) {
	commsMsg_t ackMessage;
	ackMessage.buffer[0u] = 0x00;
	ackMessage.buffer[1u] = ACK;

	const uint16_t dataLength = messageData->bufferLength + 2u; /*Message data + sizeof(CMD/SUBCMD)*/
	ackMessage.buffer[2u] = (uint8_t) ((dataLength >> 8u) & 0xFFu); /*Data length MSB*/
	ackMessage.buffer[3u] = (uint8_t) (dataLength & 0xFFu); /*Data length LSB*/

	ackMessage.buffer[4u] = (uint8_t) ((command >> 8u) & 0xFFu); /*Command MSB*/
	ackMessage.buffer[5u] = (uint8_t) (command & 0xFFu); /*Command LSB*/
	ackMessage.bufferLength = 6u;
	for (uint8_t i = 0u;
		 (i < messageData->bufferLength) && (ackMessage.bufferLength < (NG_MSG_BUFFER_SIZE - NG_COMMS_CRC_LENGTH)); i++) {
		ackMessage.buffer[ackMessage.bufferLength] = messageData->buffer[i];
		ackMessage.bufferLength++;
	}
	const uint16_t crc = nextGenComms_CalculateCRC(ackMessage.buffer, ackMessage.bufferLength);
	ackMessage.buffer[ackMessage.bufferLength] = (uint8_t) ((crc >> 8u) & 0xFFu);
	ackMessage.bufferLength++;
	ackMessage.buffer[ackMessage.bufferLength] = (uint8_t) (crc & 0xFFu);
	ackMessage.bufferLength++;
	/*Escape the message*/
	commsMsg_t escapedAckMessage;
	nextGenComms_EscapeMessage(&ackMessage, &escapedAckMessage);
	driverInterface->sendPacket(&escapedAckMessage);

}

void nextGenComms_SendNackPacket(const nextGenCommsDriverInterface_t *const driverInterface, const uint16_t command,
								 const nextGenCommsAckNackReason_t reason) {

	commsMsg_t nackMessage = {.buffer = {0x00, NACK, 0x00u, 0x03u, (uint8_t) ((command >> 8u) & 0xFFu),
										 (uint8_t) (command & 0xFF), (uint8_t) reason, 0x00u, 0x00u}, .bufferLength =
	NG_COMMS_NACK_MSG_LENGTH};
	const uint16_t crc = nextGenComms_CalculateCRC(nackMessage.buffer, NG_COMMS_NACK_MSG_LENGTH - NG_COMMS_CRC_LENGTH);
	nackMessage.buffer[NG_COMMS_NACK_MSG_LENGTH - 1u] = (uint8_t) (crc & 0xFFu);
	nackMessage.buffer[NG_COMMS_NACK_MSG_LENGTH - 2u] = (uint8_t) ((crc >> 8u) & 0xFFu);
	/*Escape the message*/
	commsMsg_t escapedNackMessage;
	nextGenComms_EscapeMessage(&nackMessage, &escapedNackMessage);
	driverInterface->sendPacket(&escapedNackMessage);

}

/**
 * @brief Replace 'escaped' bytes with correct byte data
 * @param receivedMessage The message to de-escape
 * @param deEscaped The de-escaped message
 * @return true Successful
 * @return false Unsuccessful. Message buffer length invalid or 0
 */
static bool nextGenComms_DeEscapeMessage(const volatile commsMsg_t *receivedMessage, commsMsg_t *deEscaped) {
	bool isMessageLengthOK = !((receivedMessage->bufferLength > NG_MSG_BUFFER_SIZE) ||
							   (receivedMessage->bufferLength == 0u));
	uint16_t messageLength = 0u;
	if (isMessageLengthOK) {
		uint16_t bufferLength = receivedMessage->bufferLength;
		for (uint16_t i = 0u; (i < bufferLength) && (i < NG_MSG_BUFFER_SIZE); i++) {
			static bool prevIsEsc = false;
			if (receivedMessage->buffer[i] == ESC) {
				prevIsEsc = true;
			} else if (prevIsEsc) {
				deEscaped->buffer[messageLength] = receivedMessage->buffer[i] - ESC_OFFSET;
				messageLength++;
				prevIsEsc = false;
			} else {
				deEscaped->buffer[messageLength] = receivedMessage->buffer[i];
				messageLength++;
			}
		}
		if (bufferLength >= NG_MSG_BUFFER_SIZE) {
			messageLength = 0u;
			isMessageLengthOK = false;
		}
	}
	deEscaped->bufferLength = messageLength;
	return isMessageLengthOK;
}

/***
 * @brief Handles a received package
 * @details Checks message for validity, allow to execute... Sends ACK/NACK message
 * @param driverInterface [I] Interface to the driver
 * @param message [I] The message to handle
 * @note Message must be without STX/ETX
 */
void nextGenComms_HandlePackage(const nextGenCommsDriverInterface_t *const driverInterface, const commsMsg_t *message) {
	commsMsg_t deEscapedMessage = {.buffer={0}, .bufferLength=0u};
	if (message->bufferOverflow) {
		nextGenComms_SendNackPacket(driverInterface, 0xFFFFu, NG_NACK_REASON_MessageToLong);
	} else if (nextGenComms_DeEscapeMessage(message, &deEscapedMessage)) {
		const uint8_t messageLength = deEscapedMessage.bufferLength;
		/*Check CRC*/
		const uint16_t crcCalculated = nextGenComms_CalculateCRC(deEscapedMessage.buffer, messageLength - NG_COMMS_CRC_LENGTH);
		const uint16_t crcMessage =
				(uint16_t) deEscapedMessage.buffer[messageLength - 1u] | ((uint16_t) deEscapedMessage.buffer[messageLength - 2u] << 8u);
		if (crcCalculated == crcMessage) {
			const uint16_t commandId = ((uint16_t) deEscapedMessage.buffer[0u] << 8u) | (uint16_t) deEscapedMessage.buffer[1u];

			const uint16_t messageSizeRx = ((uint16_t) deEscapedMessage.buffer[2u] << 8u) | (uint16_t) deEscapedMessage.buffer[3u];

			const size_t messageHandlerIndex = FindCommand(driverInterface->commandTable,
														   driverInterface->commandTableSize, commandId);

			if (messageHandlerIndex != SIZE_MAX) {
				const nextGenCommsCommand_t *commandEntry = &driverInterface->commandTable[messageHandlerIndex];
				/*check if the length byte == to the received message*/
				if (messageSizeRx != (messageLength - 6u)) {
					/*Nack invalid message size byte*/
					nextGenComms_SendNackPacket(driverInterface, commandId, NG_NACK_REASON_InvalidDataByteLength);
				} else if ((messageSizeRx < commandEntry->rxSizeMin) || (messageSizeRx > commandEntry->rxSizeMax)) {
					nextGenComms_SendNackPacket(driverInterface, commandId, NG_NACK_REASON_InvalidMessageLength);
				}
					/*Command 0x0001 is reserved for command enter command 0x0000 is illegal, so if we have command 0x0001 @ first location we handle Enter/Leave Protocol*/
				else if (driverInterface->commandTable[0].command == 0x0001u) {
					if (!driverInterface->isProtocolActive) {
						/*If Protocol is not active only Enter protocol is allowed*/
						if (commandEntry->command == 0x0001u) {
							commsMsg_t response = {.bufferLength = 0U, .bufferOverflow = false, .buffer={0U}};
							nextGenCommsAckNackReason_t Result = commandEntry->handler(&deEscapedMessage.buffer[4u],
																					   messageSizeRx, &response);
							if (Result != NG_NACK_REASON_OK) {
								nextGenComms_SendNackPacket(driverInterface, commandId, Result);
							} else {
								nextGenComms_SendAckPacket(driverInterface, commandId, &response);
							}
						} else {
							nextGenComms_SendNackPacket(driverInterface, commandId, NG_NACK_REASON_InvalidDeviceState);
						}
					} else {
						/*We have a valid message execute the handler and reset the timeout timer*/
						commsMsg_t response = {.bufferLength = 0U, .bufferOverflow = false, .buffer={0U}};
						nextGenCommsAckNackReason_t Result = commandEntry->handler(&deEscapedMessage.buffer[4u], messageSizeRx,
																				   &response);
						if (Result != NG_NACK_REASON_OK) {
							nextGenComms_SendNackPacket(driverInterface, commandId, Result);
						} else {
							nextGenComms_SendAckPacket(driverInterface, commandId, &response);
						}
						if (driverInterface->isProtocolActive) {
							if (driverInterface->timeoutReset != NULL) {
								driverInterface->timeoutReset();
							}
						}
					}
				} else {
					/*We have a valid message execute the handler and reset the timeout timer*/
					commsMsg_t response = {.bufferLength = 0U, .bufferOverflow = false, .buffer={0U}};
					nextGenCommsAckNackReason_t Result = commandEntry->handler(&deEscapedMessage.buffer[4u], messageSizeRx,
																			   &response);
					if (Result != NG_NACK_REASON_OK) {
						nextGenComms_SendNackPacket(driverInterface, commandId, Result);
					} else {
						nextGenComms_SendAckPacket(driverInterface, commandId, &response);
					}
				}
			} else {
				nextGenComms_SendNackPacket(driverInterface, commandId, NG_NACK_REASON_UnknownCommand);
			}

		} else {
			nextGenComms_SendNackPacket(driverInterface, 0xFFFFu, NG_NACK_REASON_CRCError);
		}
	} else {
		nextGenComms_SendNackPacket(driverInterface, 0xFFFFu, NG_NACK_REASON_MessageToLong);
	}
}
/**
 * @brief Checks if the commands in the command table are in ascending order and no doubles are found.
 * @param driverInterface
 * @return true Table in ascending order and no doubles
 * @return false Table \b NOT in ascending order or \b double values are found
 */
bool CheckCommandTable(const nextGenCommsDriverInterface_t *driverInterface) {
	bool result = driverInterface != NULL;
	if (result) {
		result = (driverInterface->commandTable != NULL) && driverInterface;
		if (result) {
			size_t commandTableSize = driverInterface->commandTableSize;
			const nextGenCommsCommand_t *const commandTable = driverInterface->commandTable;
			uint16_t previousCommand = commandTable[0u].command;

			result = (commandTable[0u].rxSizeMin <=
					  commandTable[0u].rxSizeMax); //Check Min/Max rxSize for first command
			for (size_t i = 1u; (i < commandTableSize) && (result == true); i++) {
				uint16_t currentCommand = commandTable[i].command;
				if (currentCommand <= previousCommand) {
					result = false;
				}
				if (commandTable[i].rxSizeMin > commandTable[i].rxSizeMax) {
					result = false;
				}
				previousCommand = currentCommand;
			}
		}
	}
	return result;
}
