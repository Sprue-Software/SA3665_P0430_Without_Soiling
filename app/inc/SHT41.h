/*
 * HDC2022_drv.h
 *
 *  Created on: 1 Apr 2022
 *      Author: uhegde
 */

#ifndef HDC2022_DRV_H_
#define HDC2022_DRV_H_

#include "debug.h"
#include <stdbool.h>
#include <stdint.h>
#include "comms_handler.h"

#ifdef DEBUG_ENABLE_TEMP_HUMID
#define DEBUG_TEMP_HUMID(str, numMode, dataValue) debug_out(str,numMode,dataValue)
#else
#define DEBUG_TEMP_HUMID(str, numMode, dataValue)
#endif



#endif /* HDC2022_DRV_H_ */
