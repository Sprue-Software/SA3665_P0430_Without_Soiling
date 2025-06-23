/*
 * eeprom_handler.c
 *
 *  Created on: 6 Apr 2022
 *      Author: uhegde
 */
#include <stddef.h>
#include <stdbool.h>
#include "em_assert.h"
#include "em_i2c.h"
#include "sl_sleeptimer.h"
#include "comms_handler.h"
#include "hal_i2c.h"
#include "eeprom_handler.h"
#include "em_gpio.h"


#define EEPROM_ADDRESS_BYTES 		(2u)
#define EEPROM_WRITELEN_MAX   		(EEPROM_PAGE_SIZE + EEPROM_ADDRESS_BYTES)
#define MAX_RETRYS  				(3u)
#define EEPROM_WRITE_DELAY 			(5u)
#define I2C_TRANSFER_TIMEOUT 		(300000u)

/**
 * Decrease the retry counter and set fault if hits zero
 * @param retry current retry counter
 * @return new retry value
 */
static uint8_t EEPROM_checkForFault(uint8_t retry) {
	uint8_t newRetry = retry;
	newRetry--;
	if (newRetry == 0u) {
		// TODO: check eeprom fault
	    DEBUG_EEPROM("\n report EEP fault", false, 0u);
	}
	return newRetry;
}

/**
 * Check the data written matches the data read back.
 * @param originalData Data written
 * @param readData data read back
 * @param dataLen Length of data
 * @return True if all data matches
 */
static bool EEPROM_VerifyData(uint8_t *originalData, const uint8_t *readData,
		const uint16_t dataLen) {
	bool verifyPass = true;
	uint16_t i = 0u;

	for (i = 0u; i < dataLen; i++) {
		if (*readData++ != *originalData++) {
			verifyPass = false;
		}
	}
	return verifyPass;
}

/**
 * @brief Write bytes to the external EEPROM
 * @pre I2C_HardwareEnable() must be called before using this function
 * @post I2C_HardwareDisable() must be called after this function is used
 * @param data Pointer to location of bytes to be written to EEPROM
 * @param startAddr Address of first location to write to
 * @param dataLen Number of bytes to be written
 * @note The bytes written must not cross a page boundary as the EEPROMs internal address counter will wrap to start of page and not continue to next page
 */
static bool EEPROM_Write(const uint8_t *data, const uint16_t startAddr,
		const uint16_t dataLen, bool incrementSource) {

	bool write_success = false;
	if ((data != NULL) && ((startAddr + dataLen) <= EEPROM_SIZE)) {

		uint16_t currStartAddr = startAddr;
		uint16_t total_bytes_written = 0;
		do
		{
			I2C_TransferReturn_TypeDef ret = i2cTransferInProgress;
			uint8_t i2c_write_data[EEPROM_WRITELEN_MAX] = {0};
			uint16_t cmd_len = 0u;
			uint16_t bytes_to_write;
			uint16_t rem_bytes_in_curr_page;
			rem_bytes_in_curr_page = EEPROM_PAGE_SIZE - (currStartAddr % EEPROM_PAGE_SIZE);
			if (rem_bytes_in_curr_page > dataLen)
			{
				bytes_to_write = dataLen;
			}
			else
			{
				bytes_to_write = rem_bytes_in_curr_page;
			}
			/* Select command to issue */
			i2c_write_data[0u] = (uint8_t)(currStartAddr >> 8u);
			i2c_write_data[1u] = (uint8_t)(currStartAddr & 0x00FFu);

			for (uint8_t cpy_idx = 0u; cpy_idx < bytes_to_write; cpy_idx++, total_bytes_written++)
			{
				uint8_t next_byte;
				if(true == incrementSource)
				{
					next_byte = data[total_bytes_written];
				}
				else 
				{
					next_byte = data[0];
				}
				i2c_write_data[2u + cpy_idx] = next_byte;
			}

			cmd_len = bytes_to_write + EEPROM_ADDRESS_BYTES;
			ret = send_i2c_request(dev_eeprom, I2C_FLAG_WRITE, i2c_write_data,
								   cmd_len, NULL, 0u); /* Send eeprom write command */
			if (ret == i2cTransferDone)
			{
				write_success = true;
			}
			else
			{
				DEBUG_EEPROM("\nWrite fault: ", true, ret);
				break; /*Do not continue write operation*/
			}
			sl_sleeptimer_delay_millisecond(EEPROM_WRITE_DELAY); /*Make sure all data is written before continuing*/
			currStartAddr += bytes_to_write;
		} while (total_bytes_written < dataLen);
	}

	return write_success;
}

/**
 * @brief Read bytes from the external EEPROM
 * @pre I2C_HardwareEnable() must be called before using this function
 * @post I2C_HardwareDisable() must be called after this function is used
 * @param data Pointer to location to save read bytes
 * @param startAddr First address to read from
 * @param dataLen Number of bytes to read
 */
bool EEPROM_Read(uint8_t *data, const uint16_t startAddr,
		const uint16_t dataLen) {
	I2C_TransferReturn_TypeDef ret = i2cTransferInProgress;
	uint8_t i2c_write_data[EEPROM_ADDRESS_BYTES] = {0};
	bool read_success = false;

	if ((data != NULL) && ((startAddr + dataLen) <= EEPROM_SIZE))  {
		/* EEPROM_ADDRESS_BYTES == 2 */
		i2c_write_data[0] = (uint8_t) (startAddr >> 8U);
		i2c_write_data[1] = (uint8_t) (startAddr & 0xFFU);

		ret = send_i2c_request(dev_eeprom, I2C_FLAG_WRITE_READ, i2c_write_data,
				EEPROM_ADDRESS_BYTES, data, dataLen); /* Send eeprom read command */
		if (ret == i2cTransferDone) {
		    read_success = true;
		} else {
			DEBUG_EEPROM("\n Read eep Nok", true, ret);
		}
	}

	return read_success;
}

/**
 * @brief Write bytes to the external EEPROM and verifies that the write was successful.
 * @details If the verification fails, the data is written and read again upto 3 times before a fault is set.
 * @param data Pointer to location of bytes to be written to EEPROM
 * @param startAddr Address of first location to write to
 * @param dataLen Number of bytes to be written
 * @note The bytes written must not cross a page boundary as the EEPROMs internal address counter will wrap to start of page and not continue to next page
 */
bool EEPROM_WriteandVerify(const uint8_t *data, const uint16_t startAddr,
		const uint16_t dataLen) {
	uint8_t retry = MAX_RETRYS;
	uint8_t verifyData[EEPROM_WRITELEN_MAX] = {0};
  bool writeVerify_Success = false;
  bool write_Success = false;
  bool read_Success = false;
	/*In development mode allow writing to read-only data to allow for EEPROM to be cleared etc.*/
	if (startAddr < EEPROM_SIZE) {
		while (retry > 0u) {
			write_Success = EEPROM_Write(data, startAddr, dataLen, true);
			read_Success = EEPROM_Read(verifyData, startAddr, dataLen);
			if (write_Success && read_Success)
			{
				if (EEPROM_VerifyData(verifyData, data, dataLen) == true)
				{
					retry = 0u; /*All good so leave loop*/
					writeVerify_Success = true;
				}
				else
				{
					retry = EEPROM_checkForFault(retry);
				}
			}
			else
			{
				retry = EEPROM_checkForFault(retry);
			}
	}
}

	return writeVerify_Success;
}

/**
 * Write over the full EEPROM or production data
 * @pre I2C_HardwareEnable() must be called before using this function
 * @post I2C_HardwareDisable() must be called after this function is used
 * @param blankValue
 */
bool EEPROM_Blank(uint16_t startAddress) {
	uint8_t zeroBuffer[EEPROM_PAGE_SIZE];
	uint16_t x = 0u;
	bool blank_success = true;

	for (x = 0u; x < EEPROM_PAGE_SIZE; x++) {
		zeroBuffer[x] = 0xFF; 		/*build a buffer of the blank value, the size of a page*/
	}

	x = startAddress;
	while (x < EEPROM_SIZE) {
		if(EEPROM_WriteandVerify(zeroBuffer, x, EEPROM_PAGE_SIZE) == true) {/*write the blanking value to the EEPROM 1 page at a time*/
		x += EEPROM_PAGE_SIZE;
	}
		else {
		   blank_success = false;
		   break;
		}
	}
	return blank_success;
}

/**
 * Write a certain value to a complete section in the EEPROM
 * @pre I2C_HardwareEnable() must be called before using this function
 * @post I2C_HardwareDisable() must be called after this function is used
 * @param blankValue
 */
bool EEPROM_SetSection(const uint16_t startAddr, const uint16_t sectionLength, uint8_t value)
{
	return EEPROM_Write(&value, startAddr, sectionLength, false);
}
