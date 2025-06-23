#include <stdbool.h>
#include <signal.h>
#include <stddef.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/file.h>
#include <stdio.h>
#include <stdlib.h>
#include "pthread.h"
#include "SerialDriver_Linux.h"


pthread_mutex_t g_FlagMutexRx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t g_ThreadFlagRx = PTHREAD_COND_INITIALIZER;



static void SerialPort_RxSignalHandler(int);
static enum UartErrorCode_e SerialPort_Open(const char* i_Port, const enum BaudRate_e i_BaudRate, const enum Databits_e i_Databits, const enum Stopbits_e i_Stopbits, const enum Parity_e i_Parity, const enum Flowcontrol_e i_Flowcontrol);
static void SerialPort_Close(void);


static int g_SerialPortFD = -1L;

static msgBuffer_t receiveMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
static msgBuffer_t transmitMsgBuffer = {.writeIndex = 0u, .readIndex = 0u};
static const nextGenCommsDriverInterface_t* g_DriverInterface;
static volatile bool g_KeepRunning = false;

static void* uartCommsTaskRx()
{
	while(g_KeepRunning)
	{
		pthread_mutex_lock( &g_FlagMutexRx );
		pthread_cond_wait(&g_ThreadFlagRx, &g_FlagMutexRx);
		pthread_mutex_unlock( &g_FlagMutexRx );
		while(receiveMsgBuffer.readIndex != receiveMsgBuffer.writeIndex)
		{
			nextGenComms_HandlePackage(g_DriverInterface, &receiveMsgBuffer.buffer[receiveMsgBuffer.readIndex]);
			receiveMsgBuffer.readIndex = (receiveMsgBuffer.readIndex + 1u) % NG_MSG_BUFFER_COUNT;
		}
	}
}

void uartSendMessage(const commsMsg_t* message)
{
	if(g_SerialPortFD != -1)
	{
		write(g_SerialPortFD, message->buffer, message->bufferLength);
	}
}

void commsAppRun(const char* i_UartPort, const nextGenCommsDriverInterface_t* const driverInterface)
{
	if(SerialPort_Open(i_UartPort, B9600, Databits8, Stopbits1, ParityNone, Flowcontrol_None) != UartError_None)
	{
		printf("[ERROR] unable to open serial port [%s]\n", i_UartPort);
	}
	else
	{
		g_KeepRunning = true;
		pthread_t commsThreadRx;
		g_DriverInterface = driverInterface;
		if(pthread_create(&commsThreadRx, NULL, &uartCommsTaskRx, NULL))
		{
			printf("[ERROR] Unable to create comms RX thread");
		}
		getchar();

		g_KeepRunning = false;
		pthread_join(commsThreadRx, NULL);
		SerialPort_Close();
	}
	exit(0);
}

enum UartErrorCode_e SerialPort_Open(const char* i_Port, const enum BaudRate_e i_BaudRate, const enum Databits_e i_Databits, const enum Stopbits_e i_Stopbits, const enum Parity_e i_Parity, const enum Flowcontrol_e i_Flowcontrol)
{
	enum UartErrorCode_e Result = UartError_None;
	//Ensure port is closed
	SerialPort_Close();

	g_SerialPortFD = open(i_Port, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if(g_SerialPortFD == -1L)
	{
		Result = UartError_CannotOpenPort;
	}
	else
	{
		struct termios Termios;

		tcgetattr(g_SerialPortFD, &Termios);

		Termios.c_cflag  = i_BaudRate;
		Termios.c_cflag |= (tcflag_t)(i_Databits);
		Termios.c_cflag |= (tcflag_t)(i_Stopbits);
		Termios.c_cflag |= (tcflag_t)(i_Parity);
		Termios.c_cflag |= CREAD;

		Termios.c_iflag = IGNPAR | IGNBRK;

		if(i_Flowcontrol == Flowcontrol_XOnXOff)
		{
			Termios.c_iflag |= (tcflag_t)(i_Flowcontrol);
		}
		else
		{
			Termios.c_cflag |= (tcflag_t)(i_Flowcontrol);
		}
		Termios.c_oflag = 0L;
		Termios.c_lflag = 0L;
		Termios.c_cc[VTIME] = 0L;
		Termios.c_cc[VMIN] = 0L;

		tcsetattr(g_SerialPortFD, TCSANOW, &Termios);
		tcflush(g_SerialPortFD, TCOFLUSH);
		tcflush(g_SerialPortFD, TCIFLUSH);


		fcntl(g_SerialPortFD, F_SETOWN, getpid());

		struct sigaction SigAction;
		SigAction.sa_handler = SerialPort_RxSignalHandler;
		sigemptyset(&SigAction.sa_mask);
		SigAction.sa_flags = 0;
		sigaction(SIGIO, &SigAction, NULL);

		fcntl(g_SerialPortFD, F_SETFL, FASYNC);
	}
}

void SerialPort_Close(void)
{
	if(g_SerialPortFD != -1L)
	{
		struct sigaction SigAction;
		sigfillset(&SigAction.sa_mask);
		SigAction.sa_handler = SIG_DFL;
		sigaction(SIGIO, &SigAction, NULL);

		tcflush(g_SerialPortFD, TCOFLUSH);
		tcflush(g_SerialPortFD, TCIFLUSH);
		flock(g_SerialPortFD, LOCK_UN);
		close(g_SerialPortFD);
		g_SerialPortFD = -1L;
	}
}
static void SerialPort_RxSignalHandler(int)
{
	static bool canWriteToRx = false;
	static bool canReceive= true;
	static commsMsg_t* currentRxBuffer = & receiveMsgBuffer.buffer[0u];

	uint8_t tempBuffer[32u] = {};
	static const ssize_t tempBufferSize = sizeof(tempBuffer) / sizeof(tempBuffer[0u]);
	ssize_t bytesRead = 0;
	do
	{
		bytesRead = read(g_SerialPortFD, tempBuffer, tempBufferSize);
		if(bytesRead > 0)
		{
			for(ssize_t bytesProcessed = 0; bytesProcessed < bytesRead; bytesProcessed++)
			{
				uint8_t currentByte = tempBuffer[bytesProcessed];
				if(currentByte == 0x03/*ETX*/)
				{
					if(((receiveMsgBuffer.writeIndex + 1) % NG_MSG_BUFFER_COUNT) != receiveMsgBuffer.readIndex)
					{
						canReceive = true;
						if(canWriteToRx)
						{
							receiveMsgBuffer.writeIndex = ((receiveMsgBuffer.writeIndex + 1u) % NG_MSG_BUFFER_COUNT);
							pthread_mutex_lock( &g_FlagMutexRx );
							pthread_cond_signal(&g_ThreadFlagRx);
							pthread_mutex_unlock( &g_FlagMutexRx );
						}
					}
					else
					{
						canReceive = false;
					}
					canWriteToRx = false;
				} else if (currentByte == 0x02/*STX*/)
				{
					if(canReceive)
					{
						currentRxBuffer = &receiveMsgBuffer.buffer[receiveMsgBuffer.writeIndex];
						currentRxBuffer->bufferLength = 0u;
						currentRxBuffer->bufferOverflow = false;
						canWriteToRx = true;
					}
				}else //Data byte
				{
					if(canWriteToRx)
					{
						if (currentRxBuffer->bufferLength < NG_MSG_BUFFER_SIZE) {
							currentRxBuffer->buffer[currentRxBuffer->bufferLength] = currentByte;
							currentRxBuffer->bufferLength++;
						} else {
							//Not enough room, set overflow flag and UART_RX flag
							canWriteToRx = false;
							currentRxBuffer->bufferOverflow = true;
							pthread_mutex_lock( &g_FlagMutexRx );
							pthread_cond_signal(&g_ThreadFlagRx);
							pthread_mutex_unlock( &g_FlagMutexRx );
						}
					}
				}
			}
		}
	}while(bytesRead == tempBufferSize);
}
