/*
 * hal_i2c.c
 *
 *  Created on: 4 Apr 2022
 *      Author: uhegde
 */
#include "em_gpio.h"
#include "sl_i2cspm_i2c_config.h"
#include "hal_i2c.h"
#include "os.h"
#include "hal_gpio.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "hal_AFE.h"
#include "assistance_light.h"

#include "v3sctrl.h"

static OS_MUTEX I2C_Mutex; /*Semaphore for LDMA transfer complete*/
I2C_powerControl_t I2C_powerOwner;

/**
 * @brief 3VS state prior to turning on
 */
static bool v3s_state = false;

void hal_i2c_init(void) {
	RTOS_ERR err;

	/* Create the i2c mutex.                           */
	OSMutexCreate(&I2C_Mutex, "I2C_Sem", &err);
	/*   Check error code.                                  */
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	I2C_powerOwner = I2CPower_None;
}

void I2C_BusAcquire(I2C_powerControl_t sender, uint16_t powerupTime) {
	RTOS_ERR err;
	OSMutexPend(&I2C_Mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	I2C_powerOwner = sender;							 /*sender is now in control*/

	v3s_state = V3S_ON_LOCK( );

	if (powerupTime > 0u)
	{
		hal_AFE_delay_us(powerupTime);
	}

	/* Pin PA07 is configured to Open-drain with pull-up and filter */
	GPIO_PinModeSet(SL_I2CSPM_I2C_SDA_PORT, SL_I2CSPM_I2C_SDA_PIN,
					gpioModeWiredAnd, 1u); /* WiredAndPullUpFilter takes 40uA */

	/* Pin PA08 is configured to Open-drain with pull-up and filter */
	GPIO_PinModeSet(SL_I2CSPM_I2C_SCL_PORT, SL_I2CSPM_I2C_SCL_PIN,
					gpioModeWiredAnd, 1u); /* WiredANDPullUpFilter takes 40uA */
}

void I2C_BusRelease(void)
{
	RTOS_ERR err;
	/* Pin is configured to Open-drain with pull-up and filter */
	//GPIO_PinModeSet(SL_I2CSPM_I2C_SDA_PORT, SL_I2CSPM_I2C_SDA_PIN, gpioModeWiredAnd, 1u);
	GPIO_PinModeSet(SL_I2CSPM_I2C_SDA_PORT, SL_I2CSPM_I2C_SDA_PIN, gpioModeDisabled, 0u);
	/* Pin is configured to Open-drain with pull-up and filter */
	//GPIO_PinModeSet(SL_I2CSPM_I2C_SCL_PORT, SL_I2CSPM_I2C_SCL_PIN, gpioModeWiredAnd, 1u);
	GPIO_PinModeSet(SL_I2CSPM_I2C_SCL_PORT, SL_I2CSPM_I2C_SCL_PIN, gpioModeDisabled, 0u);
	/*Finally, disable the power pin and clear the power owner*/
	V3S_ON_UNLOCK( v3s_state );
	OSMutexPost(&I2C_Mutex, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	I2C_powerOwner = I2CPower_None;
}

/**
 * desc generic function to send te i2c request
 * @param dev_type  i2c device type
 * @param flag      function type flag
 * @param writeCmd  write command buffer
 * @param writeLen  write command data
 * @param readCmd   read buffer
 * @param readLen   read buffer length
 * @return status of the i2c request
 */
I2C_TransferReturn_TypeDef send_i2c_request(uint8_t dev_type, uint8_t flag,
		uint8_t *writeCmd, uint16_t writeLen, uint8_t *readCmd,
		uint16_t readLen) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;

	if (dev_type == dev_temp_humid_sens) {
		seq.addr = SHT41_X_ADDRESS; /* assign the temp and humidity sensor address */
	} else if (dev_type == dev_eeprom) {
		seq.addr = EEPROM_I2C_ADDRESS; /* assign eeprom address */
	} else {
		/* avoid MISRA violation */
	}

	seq.flags = flag;

	switch (flag) {
	// Send the write command from writeCmd
	case I2C_FLAG_WRITE:
		seq.buf[0].data = writeCmd;
		seq.buf[0].len = writeLen;

		break;

		// Receive data into readCmd of readLen
	case I2C_FLAG_READ:
		seq.buf[0].data = readCmd;
		seq.buf[0].len = readLen;

		break;

		// Send the write command from writeCmd
		// and receive data into readCmd of readLen
	case I2C_FLAG_WRITE_READ:
		seq.buf[0].data = writeCmd;
		seq.buf[0].len = writeLen;

		seq.buf[1].data = readCmd;
		seq.buf[1].len = readLen;

		break;

	default:
		return i2cTransferUsageFault;
		break; /* to avoid MISRA violation */
	}

	// Perform the transfer and return status from the transfer
	ret = I2CSPM_Transfer(I2C0, &seq); /* inbuilt polled timeout is 300000 */

	return ret;
}
