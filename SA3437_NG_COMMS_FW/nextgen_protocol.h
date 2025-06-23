#ifndef NG_COMMS_PROTOCOL_H
#define NG_COMMS_PROTOCOL_H

#include <stddef.h>
#include "stdint.h"
#include "stdbool.h"

#define TX_BUFFER_SIZE        133u /**< Size (uint8_t) of the TX byte buffer. This needs to be large enough to handle an XMODEM packet (133bytes).*/
#define STX                   0x02u /**< Start of message*/
#define ETX                   0x03u /**< End of message */

#ifndef NG_MSG_BUFFER_COUNT
#define NG_MSG_BUFFER_COUNT   32u /**<Number of message buffers*/
#endif
#ifndef NG_MSG_BUFFER_SIZE
#define NG_MSG_BUFFER_SIZE    32u /**<Max size of a message*/
#endif


typedef enum {
	NG_NACK_REASON_OK = 0x00u,
	NG_NACK_REASON_CRCError = 0x01u,
	NG_NACK_REASON_InvalidDataByteLength = 0x02u,
	NG_NACK_REASON_InvalidMessageLength = 0x03u,
	NG_NACK_REASON_MessageToLong = 0x04u,
	NG_NACK_REASON_UnknownCommand = 0x05u,
	NG_NACK_REASON_InvalidDeviceState = 0x06u,
	NG_NACK_REASON_InvalidData = 0x07u,
	NG_NACK_REASON_CommandNotImplemented = 0x08u,
	NG_NACK_REASON_TestFailed = 0x09u,
	NG_NACK_REASON_TestNotFinished = 0x0Au,
	NG_NACK_REASON_NVMError = 0x0Bu,
	NG_NACK_REASON_Timeout = 0x0Cu,
	NG_NACK_REASON_BufferOverflow = 0x0Du,
	NG_NACK_REASON_HardwareError = 0x0Eu,
	NG_NACK_REASON_SDKError = 0xFDu,
	NG_NACK_REASON_InternalError = 0xFEu
} nextGenCommsAckNackReason_t;

typedef struct {
	volatile uint16_t bufferLength;            /**<Length of data in buffer*/
	volatile bool bufferOverflow;
	uint8_t buffer[NG_MSG_BUFFER_SIZE];        /**<buffer containing message data*/
} commsMsg_t;

typedef struct {
	commsMsg_t buffer[NG_MSG_BUFFER_COUNT];
	volatile uint16_t writeIndex;
	volatile uint16_t readIndex;
} msgBuffer_t;


typedef nextGenCommsAckNackReason_t (*nextGenCommsHandler_t)(const uint8_t[], const uint16_t, commsMsg_t *);

typedef struct {
	uint16_t command;
	uint16_t rxSizeMin;
	uint16_t rxSizeMax;
	nextGenCommsHandler_t handler;
} nextGenCommsCommand_t;


typedef void (*nextGenProtocolSendPacket_t)(const commsMsg_t *);

typedef void (*nextGenCommsTimoutResetHandler_t)(void);

typedef struct nextGenCommsDriverInterface_s {
	bool isProtocolActive;                                  /**<Used to flag if a protocol is active. Only use by protocols that need enter/exit functionality*/
	const nextGenCommsTimoutResetHandler_t timeoutReset;    /**<Reset protocol timeout timer. Only use by protocols that need enter/exit functionality*/
	const nextGenProtocolSendPacket_t sendPacket;           /**<Handler to send a package over the selected hardware interface.*/
	const size_t commandTableSize;                          /**<Total Size of the command table*/
	const nextGenCommsCommand_t *commandTable;              /**<The command table for the protocol*/
} nextGenCommsDriverInterface_t;


extern nextGenCommsAckNackReason_t nextGenComms_CommandNotImplemented(const uint8_t message[], const uint16_t messageSize, commsMsg_t *resultData);

extern bool CheckCommandTable(const nextGenCommsDriverInterface_t *driverInterface);

extern void nextGenComms_HandlePackage(const nextGenCommsDriverInterface_t *const driverInterface, const commsMsg_t *message);

extern uint16_t nextGenComms_CalculateCRC(const uint8_t buffer[], const uint16_t length);

#endif /*NG_COMMS_PROTOCOL_H*/
