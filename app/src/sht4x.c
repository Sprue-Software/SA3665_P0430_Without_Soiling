/*
 * temp_humid.c
 *
 *  Created on: 6 october 2022
 *      Author: esambi
 */
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "os.h"
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "hal_i2c.h"
#include "comms_handler.h"
#include "sht4x.h"

/* all measurement commands return T (CRC) RH (CRC) */
#define SHT4X_CMD_MEASURE_HPM 0xFD
#define SHT4X_CMD_MEASURE_LPM 0xE0
#define SHT4X_CMD_READ_SERIAL 0x89
#define SHT4X_CMD_DURATION_USEC 1000
#define SHT4X_CMD_SOFT_RESET  0x94

#define SHT4X_ADDRESS 0x44

#define SHT4X_TX_BUFFER_SIZE              6
#define SHT4X_RXBUFFER_SIZE               6

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32

uint8_t txBuffer[SHT4X_TX_BUFFER_SIZE]= {0u,};
uint8_t txBufferSize = SHT4X_TX_BUFFER_SIZE;
uint8_t rxBuffer[SHT4X_RXBUFFER_SIZE]= {0u,};
uint8_t rxBufferIndex;
int32_t temperature;
int32_t humidity;

static uint8_t sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
static uint16_t sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;
static void sht4x_soft_reset(void);
static int16_t sht4x_measure(void);
void sht4x_enable_low_power_mode(uint8_t enable_low_power_mode);


static void sht4x_soft_reset(void) {
    int16_t ret;
    const uint8_t cmd = SHT4X_CMD_SOFT_RESET;
    uint8_t index=200u;

    do{
		ret = send_i2c_request(dev_temp_humid_sens, I2C_FLAG_WRITE, (uint8_t *)&cmd, 1u, NULL, 0u); /* no read back data here */
		sl_sleeptimer_delay_millisecond(2);
		index--;
    }while((ret != i2cTransferDone)&&(index !=0u));

    if(ret == i2cTransferDone){

    }
    else
    {

    }
}

// Start measurement by writing command to sht41
static int16_t sht4x_measure(void) {
    uint8_t cmd = SHT4X_CMD_MEASURE_HPM;
	return send_i2c_request(dev_temp_humid_sens, I2C_FLAG_WRITE, (uint8_t *)&cmd, 1u, NULL, 0u); /* no read back data here */
}




void sht4x_enable_low_power_mode(uint8_t enable_low_power_mode) {
    if (enable_low_power_mode) {
        sht4x_cmd_measure = SHT4X_CMD_MEASURE_LPM;
        sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_LPM_USEC;
    } else {
        sht4x_cmd_measure = SHT4X_CMD_MEASURE_HPM;
        sht4x_cmd_measure_delay_us = SHT4X_MEASUREMENT_DURATION_USEC;
    }
}




//Read temperature + humidity multiple times based on user input
bool sht4x_multiple_read(int32_t counter)
{
  volatile I2C_TransferReturn_TypeDef ret = i2cTransferInProgress;
  bool trans_done = false; /* default status is transfer not done */
  uint8_t index=200u;
  int32_t t_temp, temp_rh;

  I2C_BusAcquire(I2CPower_SHT41, SHT4X_I2C_POWERUP);
  sht4x_soft_reset();
  sl_sleeptimer_delay_millisecond(1);

  for(int32_t x = 0; x < counter; x++){
	  sht4x_measure();
	  do{// Wait for data to become ready
		  sl_sleeptimer_delay_millisecond(10);
		  ret = send_i2c_request(dev_temp_humid_sens, I2C_FLAG_READ, NULL, 0u, (uint8_t *)&rxBuffer[0], 6u);
		  index--;
	  }while((ret != i2cTransferDone)&&(index !=0u));

	  if(ret == i2cTransferDone)
    {
		  trans_done = true;

		  t_temp = ( rxBuffer[ 0 ] * 256 ) + rxBuffer[ 1 ];

		  /* From datasheet: t (DegC) = -45 + 175 * t_temp/65535 */
      temperature = ( ( 17500 * t_temp ) / 65535 ) - 4500;        /* T is x 100 */

		  temp_rh = ( rxBuffer[ 3 ] * 256 ) + rxBuffer[ 4 ];

		  /* From datasheet: Humidity = -6 + 125 * temp_rh/65535 */
		  humidity = ( ( 125 * temp_rh ) / 65535 ) - 6;

      if( humidity > 100 )
      {
        humidity = 100;
      }
      else if( humidity < 0 )
      {
        humidity = 0;
      }
      else
      {
        // Nothing
      }
    }
	  else
    {
		}

    if(1 < counter)  
    {
      sl_sleeptimer_delay_millisecond(100);
    }
  }
  I2C_BusRelease(); /* Release I2C Bus */
  return trans_done;
}


int16_t sht_get_temperature(void)
{
  return temperature;
}
uint32_t sht_get_humidity(void)
{
   return humidity;
}
