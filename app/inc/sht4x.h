/*
 * Copyright (c) 2020, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *
 * \brief Sensirion SHT driver interface
 *
 * This module provides access to the SHT functionality over a generic I2C
 * interface. It supports measurements without clock stretching only.
 */


#ifdef __cplusplus
extern "C" {
#endif

#include "debug.h"

bool SHT41_measure(void);
int32_t SHT41_get_temperature(void);
int32_t hdc2022_get_temperature_co(void);
uint32_t SHT41_get_humidity(void);

#define STATUS_OK 0
#define STATUS_ERR_BAD_DATA (-1)
#define STATUS_CRC_FAIL (-2)
#define STATUS_UNKNOWN_DEVICE (-3)
#define SHT4X_MEASUREMENT_DURATION_USEC 10000 /* 10ms "high repeatability" */
#define SHT4X_MEASUREMENT_DURATION_LPM_USEC \
    2500 /* 2.5ms "low repeatability"       \
          */

extern bool sht4x_multiple_read(int32_t counter);
extern int16_t sht_get_temperature(void);
extern uint32_t sht_get_humidity(void);


/**
 * Returns the configured SHT4x address.
 *
 * @return SHT4x_ADDRESS
 */
uint8_t sht4x_get_configured_address(void);

#ifdef __cplusplus
}
#endif

