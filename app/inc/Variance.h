/**
 * @file	Variance.h
 * @brief
 * @project SA2888 Gen5 Core Firmware
 * @date	3 Jan 2020
 * @author  Andrew Stillie
 */

#ifndef variance_h
#define variance_h

#include "debug.h"

void DoVariance(void);
void initVariance(void);

/* ************************************ */
/* Constants related to Variance        */
/* ************************************ */

#define	VARIANCE_HISTORY_BUFFER_SIZE 	(64u)
#define	ACTIVATION_COUNT_LIMIT 			(127u)
#define ACCUMULATION_COUNT_LIMIT		(127u)
// Initial Strike Count Threshold = 32
#define PRE_120_HOURS_STRIKE_COUNT		(256u)
#define ZERO_COUNT_LIMIT (24u)
#define STABILISATION_FAULT_LIMIT (0xffu)
#define NIBBLE_BUFFER_LIMIT				(0x3fu)

/* ************************************ */
/* Macros related to Variance        */
/* ************************************ */

#define divideBy128(var1)	var1 >> 7
#define multiplyBy16(var1)	var1 << 4

/* ************************************ */
/* Masks related to Variance        */
/* ************************************ */

#define CLEAR_UPPER_NIBBLE	(0x0fu)
#define CLEAR_LOWER_NIBBLE	(0xf0u)

#define LOWER_NIBBLE 0u
#define UPPER_NIBBLE 1u
#define CLEAR_UPPER_NIBBLE	(0x0fu)
#define CLEAR_LOWER_NIBBLE	(0xf0u)
#define ACCUMULATION_COUNT_LIMIT		(127u)
/*Activation CO level */
#define ACTIVATION_CO_LEVEL				(16u)
#define	ACTIVATION_COUNT_LIMIT 			(127u)

#endif  /* variance_h  */
