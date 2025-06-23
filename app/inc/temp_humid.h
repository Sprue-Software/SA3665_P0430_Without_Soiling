/*
 * temp_humid.h
 *
 *  Created on: 28 Apr 2022
 *      Author: uhegde
 */

#ifndef APP_INC_TEMP_HUMID_H_
#define APP_INC_TEMP_HUMID_H_

bool temp_humid_measure_bist(bool *bistCheck); /* temp/humidity sensor measurement and BIST */
uint16_t getdayHumidityAvg(void);
void calculateHumidityAvg(void);
int32_t get_TempVal();
uint32_t get_HumidityVal();
void set_ftm_sub_command(uint8_t sub_command);
void set_ftm_simulate_humid(uint32_t ftm_humid_val);
void set_ftm_simulate_temp(uint32_t ftm_temp_val);
void temp_humid_init(void);
#endif /* APP_INC_TEMP_HUMID_H_ */
