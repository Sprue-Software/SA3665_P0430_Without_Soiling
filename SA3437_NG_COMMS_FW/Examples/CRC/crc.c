#include "crc.h"

#define XMODEM_CRC16_POLY 0x11021u

uint16_t nextGenComms_CalculateCRC(const uint8_t buffer[], const uint8_t length) {
	uint32_t crcResult = 0x00u;
	for (uint8_t i = 0; i < length; i++) {
		crcResult ^= (uint32_t) (buffer[i] << 8u);
		for (uint8_t bitCounter = 0; bitCounter < 8u; bitCounter++) {
			if ((crcResult & 0x8000u) == 0x8000u) {
				crcResult <<= 1u;
				crcResult ^= XMODEM_CRC16_POLY;
			} else {
				crcResult <<= 1u;
			}
		}
	}
	return (uint16_t) crcResult;
}
