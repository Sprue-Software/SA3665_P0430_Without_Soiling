/*
 * hal_i2c.h
 *
 *  Created on: 4 Apr 2022
 *      Author: uhegde
 */

#ifndef APP_INC_HAL_I2C_H_
#define APP_INC_HAL_I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include "em_i2c.h"

#define SHT41_X_ADDRESS      (0x88)// (0x82)  sht41 address is 0x88
#define EEPROM_I2C_ADDRESS    (0xA0)

#define SHT4X_I2C_POWERUP   (500u) /* Power up time is less than 500usec */
#define EEPROM_I2C_POWERUP    (100u) /* power up time for eeprom in micro seconds */

typedef enum {
	I2CPower_None, /**<No module has control, ie power is off*/
	I2CPower_EEPROM, /**<EEPROM module has control*/
	I2CPower_SHT41, /**<Temp/Humidity module has control*/
} I2C_powerControl_t;

typedef enum {
	dev_none, dev_temp_humid_sens, dev_eeprom,
} i2c_device_select_t;

void hal_i2c_init(void);
void I2C_BusAcquire(I2C_powerControl_t sender, uint16_t powerupTime);
void I2C_BusRelease(void);
I2C_TransferReturn_TypeDef send_i2c_request(uint8_t dev_type,
		uint8_t flag, uint8_t *writeCmd, uint16_t writeLen, uint8_t *readCmd,
		uint16_t readLen);

#endif /* APP_INC_HAL_I2C_H_ */
