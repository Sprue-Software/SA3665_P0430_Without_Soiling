#ifndef UARTCLI_H
#define UARTCLI_H

#include <stdbool.h>
#include <stdint.h>

#define CMD_TERMINATOR '\r'
#define UP_ARROW	(0x1E)
#define DOWN_ARROW	(0x1F)

typedef enum {
	UARTCLI_OK,		
	UARTCLI_ERR_NULL_PTR, 
	UARTCLI_ERR_IO,
	UARTCLI_ERR_CMD_NOT_FOUND, 
	UARTCLI_ERR_INVALID_ARGS,  
	UARTCLI_ERR_BUF_FULL,	     
	UARTCLI_IDLE	     
} cliStatus_t;

typedef cliStatus_t (*cmdHandler_t)(int argc, char **argv);
typedef struct 
{
	int8_t * cmd;	     					    /* Command name.                           			*/
	cmdHandler_t handler; 				/* Function pointer to associated function. 		*/
	const int8_t * const helpText;	/* Simple explanation of the command 				*/
	uint8_t maxNoOfParams;				/* Maximum no of arguments including -v (verbose)	*/
} cmdTable_t;

typedef void (*printFuncPtr_t)(const char * format, ...);
typedef struct 
{
	printFuncPtr_t printFunc; 
	cmdTable_t *cmdTbl;		    
	uint16_t cmdCount;		   
} cli_t;


void uartCLITask(void *arg);

bool UARTCLI_IsCliModeActive(void);
void UARTCLI_SetCliMode(bool active);
void UARTCLI_RxData(uint8_t rxByte);
cliStatus_t UARTCLI_ProcessCommand(cli_t *cli);

#endif
