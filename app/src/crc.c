/********************************************************************************************************
*
* 									CRC CALCULATOR
*
* Filename			: crc.c
* Version			: V1.00
* Programmer(s)		: AUR
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "em_gpcrc.h"
#include "em_cmu.h"
#include "crc.h"

/********************************************************************************************************
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
********************************************************************************************************/
#define CRC_POLYNOMIAL    (0x00001021)
#define CRC_INIT_VALUE    (0x0000FFFF)
static OS_MUTEX MUT_CalculateCRC;
/********************************************************************************************************
*********************************************************************************************************
*                                              FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

/****************************************************************************************************//**
*                                              CRC_Init()
*
* @brief	Initialize the CRC engine
*
* @note		(1) The following function needs to be called at the time of application initialisation.
********************************************************************************************************/
void CRC_Init(void) {
	CMU_ClockEnable(cmuClock_GPCRC, true);
	GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;

	init.crcPoly 		= CRC_POLYNOMIAL;
	init.initValue 		= CRC_INIT_VALUE;
	init.reverseBits 	= true;
	init.enable 		= true;

	GPCRC_Init(GPCRC, &init);
  CMU_ClockEnable(cmuClock_GPCRC, false);
  char cUartMessageMutexString[] = "UART Msg Mutex";
  RTOS_ERR err;
  OSMutexCreate(&MUT_CalculateCRC, &cUartMessageMutexString[0u], &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
}

/****************************************************************************************************//**
*                                            CRC_Calculate()
*
* @brief	Calculate CRC-16
*
* @param	p_buffer 	pointer to buffer that contains command and application data
*
* @param	ll_status 	link layer status field of the datagram
*
* @param	length		length of the buffer
*
* @return	crc of the provided data
* @note The CRC hardware IP is used by multiple threads, because of this we need to lock the OS scheduler
********************************************************************************************************/
uint16_t CRC_Calculate(uint8_t ll_status, uint8_t buffer[], uint16_t length) {
	uint32_t i;
	uint16_t checksum;
  RTOS_ERR  err;
  OSSchedLock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  OSMutexPend(&MUT_CalculateCRC, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

  CMU_ClockEnable(cmuClock_GPCRC, true);
  GPCRC->CTRL = _GPCRC_CTRL_RESETVALUE;
  GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;

  init.crcPoly 		= CRC_POLYNOMIAL;
  init.initValue 		= CRC_INIT_VALUE;
  init.reverseBits 	= true;
  init.enable 		= true;

  GPCRC_Init(GPCRC, &init);

	GPCRC_Start(GPCRC);

	GPCRC_InputU8(GPCRC, (uint8_t)length);				/* add lsb of the length field */
	GPCRC_InputU8(GPCRC, (uint8_t)(length >> 8));		/* add msb of the length field */
	GPCRC_InputU8(GPCRC, ll_status);					/* add link layer status field */

	for (i = 0; i < length; i++) {						/* add command and data bytes */
		GPCRC_InputU8(GPCRC, buffer[i]);
	}

	checksum = (uint16_t)GPCRC_DataReadBitReversed(GPCRC);			/* return the final CRC value */

	checksum ^= 0xffff;

	CMU_ClockEnable(cmuClock_GPCRC, false);
  OSMutexPost(&MUT_CalculateCRC, OS_OPT_POST_NONE, &err);
  OSSchedUnlock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
	return checksum;
}
/****************************************************************************************************//**
* @brief	Calculate CRC-16
*
* @param	p_buffer 	pointer to the buffer to the data
*
* @param	length		length of the buffer
*
* @return	crc of the provided data

* @note The CRC hardware IP is used by multiple threads, because of this we need to lock the OS scheduler
********************************************************************************************************/
uint16_t CRC_UartCalculate(const uint8_t buffer[], const uint16_t length)
{
  RTOS_ERR  err;
  uint16_t  crcResult;
  OSSchedLock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  OSMutexPend(&MUT_CalculateCRC, 0, OS_OPT_PEND_BLOCKING, NULL, &err);

  CMU_ClockEnable(cmuClock_GPCRC, true);
  GPCRC->CTRL = _GPCRC_CTRL_RESETVALUE;
  GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;

  init.crcPoly 		= CRC_POLYNOMIAL;
  init.initValue 		= 0x0000;
  init.reverseBits 	= true;
  init.enable 		= true;
  GPCRC_Init(GPCRC, &init);

  GPCRC_Start(GPCRC);

  for (uint16_t i = 0; i < length; i++) {						/* add command and data bytes */
    GPCRC_InputU8(GPCRC, buffer[i]);
  }

  crcResult = (uint16_t)GPCRC_DataReadBitReversed(GPCRC);			/* return the final CRC value */

  CMU_ClockEnable(cmuClock_GPCRC, false);
  OSMutexPost(&MUT_CalculateCRC, OS_OPT_POST_NONE, &err);
  OSSchedUnlock(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
  return crcResult;
}
