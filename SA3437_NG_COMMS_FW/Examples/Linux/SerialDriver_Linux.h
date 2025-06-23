#ifndef NG_COMMS_SERIALDRIVER_LINUX_H
#define NG_COMMS_SERIALDRIVER_LINUX_H

#include "inttypes.h"
#include "nextgen_protocol.h"
#include <termios.h>


enum Databits_e {
	Databits5 = (uint32_t) (CS5),
	Databits6 = (uint32_t) (CS6),
	Databits7 = (uint32_t) (CS7),
	Databits8 = (uint32_t) (CS8),
};

enum Stopbits_e {
	Stopbits1 = 0x00000000UL,
	Stopbits2 = (uint32_t) (CSTOPB)
};

enum Flowcontrol_e {
	Flowcontrol_None = (uint32_t) (CLOCAL),
	Flowcontrol_XOnXOff = (uint32_t) (IXON | IXOFF),
	Flowcontrol_CrtsCts = (uint32_t) (CRTSCTS)
};

enum Parity_e {
	ParityNone = 0x00000000UL,
	ParityOdd = (uint32_t) (PARODD | PARENB),
	ParityEven = (uint32_t) (PARENB)
};

enum BaudRate_e {
	BR9600 = (uint32_t) (B9600),
	BR115200 = (uint32_t) (B115200)
};

enum UartErrorCode_e {
	UartError_None,
	UartError_PortNotOpen,
	UartError_CannotOpenPort,
	UartError_CannotLockPort
};

extern void commsAppRun(const char *i_UartPort, const nextGenCommsDriverInterface_t *const driverInterface);

extern void uartSendMessage(const commsMsg_t *message);

#endif //NG_COMMS_SERIALDRIVER_LINUX_H
