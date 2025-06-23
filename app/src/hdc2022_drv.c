/*
 * HDC2022_logic.c
 *
 *  Created on: 31 Mar 2022
 *      Author: uhegde
 *      NDI : This Driver is modified to work with Sht41 driver
 */

#include "os.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "comms_handler.h"
#include "hal_i2c.h"
#include "sht4x.h"


#define I2CSPM_TASK_STACK_SIZE           (256u)
#define I2CSPM_TASK_PRIO                 (13u)

static uint32_t humidity;
static int32_t temperature=-2000;

/**
 * desc function to measure the temperature and humidity values
 *return true - measurement successful
 *return false - measurement failure
 */
bool SHT41_measure(void) {
  bool status=false;
  status=sht4x_multiple_read(1);
	return status;
}


/**
 * desc function to get the temperature value
 * return temperature signed value, scaled to *100 to retain the accuracy
 * example temperature value of 25.5deg is returned as 2550.
 */
int32_t SHT41_get_temperature(void) {
  temperature=sht_get_temperature();
	return temperature;
}

/**
 * function to get the humidity value
 * @return humidity value in percentage
 */
uint32_t SHT41_get_humidity(void) {

  humidity=sht_get_humidity();
	return humidity;
}
