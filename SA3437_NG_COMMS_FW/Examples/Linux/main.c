#include "stdio.h"
#include <stdint.h>

#include "nextgen_protocol.h"
#include "SerialDriver_Linux.h"

#include "Examples/DummyDeviceProtocol.h"

const char c_SerialPort[] = "/dev/ttyUSB0";

nextGenCommsDriverInterface_t dummyDriver =
		{
				.isProtocolActive = false,
				.timeoutReset = NULL,
				.sendPacket = uartSendMessage,
				.commandTableSize = DUMMY_DEVICE_PROTOCOL_COMMAND_TABLE_SIZE,
				.commandTable = DummyDeviceProtocolCommandTable

		};


int32_t main(void) {

	bool commandTableValid = CheckCommandTable(&dummyDriver);
	const char yes[] = "YES";
	const char no[] = "NO";
	printf("FCT Command Table valid ? %s\n", commandTableValid ? yes : no);

	commsAppRun(c_SerialPort, &dummyDriver);

	return 0;
}
