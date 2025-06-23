#ifndef COMMS_HANDLER_H_
#define COMMS_HANDLER_H_

#include "debug.h"
#include "os.h"
#include <stdbool.h>
#include <stdint.h>
#include "nextgen_protocol.h"


#if defined(FTM_BUILD)
typedef bool (*xModemCallback_t)(const uint8_t packetId, uint8_t data[128u]);
#endif

#ifdef DEBUG_BUILD
#define COMMS_UART_CLI_MSG  (1 << 5u)
#define COMMS_UART_CLI_CMD_EXPAND (1 << 6u)
extern OS_SEM SerialTX_Sema;
void eusart_send_data_blocking(uint8_t string[]);
void FlushDebugMessages(void);
#endif


extern OS_FLAG_GRP CommsEventFlags;
extern uint32_t reset_cause;
extern unsigned int dmaTxChannel;

void debug_out(const char str[], const bool numMode, const uint32_t dataValue);
void debug_printf( const char * format, ... );
void uartCommsTask(void *arg);
void commsAppInit(bool enable);
void commsAppInit_FTMMode(bool enter_mode);
#if defined(FTM_BUILD)
bool xModemModeEnter(xModemCallback_t callback);
void xModemModeExit(void);
#endif
#endif /* COMMS_HANDLER_H_ */

