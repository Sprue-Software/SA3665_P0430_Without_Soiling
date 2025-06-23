/*
 * eeprom_handler.h
 *
 *  Created on: 6 Apr 2022
 *      Author: uhegde
 */

#ifndef APP_INC_EEPROM_HANDLER_H_
#define APP_INC_EEPROM_HANDLER_H_

#include "debug.h"
#include <stdint.h>

#define EEPROM_PAGE_SIZE      (32u)
#define EEPROM_SIZE         (4096u)

bool EEPROM_Read(uint8_t *data, const uint16_t startAddr, const uint16_t dataLen);
bool EEPROM_WriteandVerify(const uint8_t *data, const uint16_t startAddr, const uint16_t dataLen);
bool EEPROM_Blank(uint16_t startAddress);
bool EEPROM_SetSection(const uint16_t startAddr, const uint16_t sectionLength, uint8_t value);
#endif /* APP_INC_EEPROM_HANDLER_H_ */
