/**
 @file	  CO_Variance.c
 @project  SA2888 Gen5 Core Firmware
 @author	  Andrew Stillie
 @note:    This Variance algorithm is same as CO-9x & 9B:
 @brief:	  Algorithm to Perform Variance Calculation
 */


#include  <cpu/include/cpu.h>
#include  <kernel/include/os.h>
#include "comms_handler.h"
#include "Variance.h"
#include "timeHandler.h"
#include "acquisition_Co.h"

/* variance Functions*/
uint32_t calculateVar(void);
void writeFaultLeakedSensorFault(void);
uint8_t historyDataIn(uint8_t dataIn);
void clearHistoryBuffer(void);
void calculateAverage(uint8_t rawValue, uint8_t rawValueOld);
uint8_t historyBuffZeroCount(void);
uint32_t getthreshouldhours(void);

/**
 * variance state
 * @brief These are the states associated with the Variance algorithm
 */
enum VarianceStates {
	PreCalibrationState,/**< Pre-Calibration State*/
	ActivationStabilisationState, /**< Activation State */
	AccumulationStabilisationState,/**< Accumulation State */
	VarianceRunningState,/**<  Variance Running state */
	VarianceFaultDetectedState,/**<  Variance Fault detected State */
	VarianceFaultIdleState
};

uint8_t u8historyBuf[64];
/**
 *VarianceStruct
 *@brief: These are the variables associated with the Variance algorithm
 */
static struct {
		uint8_t u8VarianceState; /**< variance State variable*/
		uint8_t u8ActivationCount; /**< Activation Count variable */
		uint8_t u8AccumulationCount; /**<Accumulation  Count variable */
		uint16_t u16CurrentCoVar;
		uint16_t u16CurrentCoAccum;
		uint8_t u8CurrentCoAverage; /**< Co average  */
		uint8_t u8CurrentCoOld;
		uint8_t u8nibbleInFlag; /**< nibble flag */
		uint8_t oneHourVarianceTick; /**<varince after One hour */
		uint8_t u8iterations;
		uint8_t u8highVarStrikeCnt;
		uint8_t u8lowVarStrikeCnt;
		uint16_t u16minimumVar32Hours; /**< minimum variance after 32 hours */
		uint16_t u16maximumVar32Hours; /**< maximum Variance after 32 hours */
		uint8_t u8strikeLoCountThreshold;
		uint8_t u8strikeHiCountThreshold;
		uint8_t u8stabilisationCount; /**< stabilisation  Count variable */
		uint8_t u8zeroCount; /**< Total Number of Zero */
} strctVar;

/**
 * clearHistoryBuffer
 * @brief Clear all elements in u8historyBuf
 */
void clearHistoryBuffer(void)
{
	uint8_t index = VARIANCE_HISTORY_BUFFER_SIZE;

	while (index != 0u){
		index--;
		u8historyBuf[index] = 0u;
	}
}

/**
 * historyBuffZeroCount
 * @brief Count number of zero in the buffer
 * @return  zero counts
 */
uint8_t historyBuffZeroCount(void)
{
	uint8_t bufferPtr;
	uint8_t nibblePtr;
	uint8_t zeroCount;
	bufferPtr = 0u;
	nibblePtr = 0u;
	zeroCount = 0u;
	while (bufferPtr <= NIBBLE_BUFFER_LIMIT){
		if (nibblePtr){
			if ((u8historyBuf[bufferPtr++] & 0x0fu) == 0u){
				zeroCount++;
			}
			nibblePtr = 0u;
		}
		else{
			if ((u8historyBuf[bufferPtr] & 0xf0u) == 0u){
				zeroCount++;
			}
			nibblePtr = 1u;
		}
	}
	return zeroCount;
}

/**
 * historyDataIn
 * @brief add all elements in u8historyBuf
 * @param dataIn  range: 0 - 255
 * @return tempOldest range: 0 - 255  Oldest member of the histor
 */
uint8_t historyDataIn(uint8_t dataIn)
{
	static uint8_t inPtr;

	static uint8_t temp;
	static uint8_t tempHistBuff;
	static uint8_t tempOldest;
	if (strctVar.u8iterations == 0U){
		inPtr = 0U;
	}
	/* *********************************** */
	/* Check to see if InPtr needs to wrap */
	/* *********************************** */
	if (inPtr > NIBBLE_BUFFER_LIMIT){
		inPtr = 0U;
	}
	/* Take temp copy of the array value so that the offset is not calculated each time the value is used. */
	tempHistBuff = u8historyBuf[inPtr];
	/* ************************** */
	/* Check state of nibble flag */
	/* ************************** */
	if (strctVar.u8nibbleInFlag == LOWER_NIBBLE)
	/* ************************** */
	/* Store data to lower nibble */
	/* ************************** */
	{
		/* ******************************************** */
		/* Mask off value to ensure its in lower nibble */
		/* ******************************************** */
		temp = dataIn & CLEAR_UPPER_NIBBLE;
		tempHistBuff = CLEAR_LOWER_NIBBLE & tempHistBuff; /* clear the LSNibble */
		tempHistBuff = tempHistBuff | temp;
		u8historyBuf[inPtr] = tempHistBuff;
		strctVar.u8nibbleInFlag = UPPER_NIBBLE;
		tempOldest = (u8historyBuf[inPtr] >> 4) & CLEAR_UPPER_NIBBLE;
	}
	/* ************************** */
	/* Store data to upper nibble */
	/* ************************** */
	else{
		/* ******************************************** */
		/* Mask off value to ensure its in upper nibble */
		/* ******************************************** */
		temp = (dataIn << 4) & CLEAR_LOWER_NIBBLE;

		tempHistBuff = CLEAR_UPPER_NIBBLE & tempHistBuff; /* clear the LSNibble */
		tempHistBuff = tempHistBuff | temp;
		u8historyBuf[inPtr] = tempHistBuff;
		strctVar.u8nibbleInFlag = LOWER_NIBBLE;
		tempOldest = u8historyBuf[NIBBLE_BUFFER_LIMIT & (inPtr + 1U)] & CLEAR_UPPER_NIBBLE;
		inPtr++;
	}
	return tempOldest;
}

/**
 calculateAverage
 @brief Calculate rolling average using the accumulation method
 @param rawValue  range: 0 - 255 most recent value
 rawValueOld  range: 0 - 255 oldest value
 @return Return Value: none
 */
void calculateAverage(uint8_t rawValue_data, uint8_t rawValueOld_data)
{
	uint8_t rawValue;
	uint8_t rawValueOld;
	rawValue = multiplyBy16(rawValue_data); /* Upscale the CO Value */
	rawValueOld = multiplyBy16(rawValueOld_data); /* Upscale the CO Value */

	if (strctVar.u8iterations == (ACCUMULATION_COUNT_LIMIT + 1)){
		strctVar.u16CurrentCoAccum = (strctVar.u16CurrentCoAccum + (uint32_t)(rawValue)) - (uint32_t)(rawValueOld); /* This old value is in correct. */
		strctVar.u8CurrentCoAverage = divideBy128(strctVar.u16CurrentCoAccum); /* divided by 128. */

	}
	else{
		strctVar.u16CurrentCoAccum = strctVar.u16CurrentCoAccum + (uint32_t)(rawValue);
		strctVar.u8iterations++;
		DEBUG_CO_VAR("\tuCoAccum ++ ", true, strctVar.u16CurrentCoAccum);

	}
}

/**
 * calculateAverage
 * @brief Calculate variance on the 128 nibbles in the 64 byte history buffer
 * @param averageValue  range: 0 - 255  average of the last 128 readings
 * @return variance parameter
 */
uint32_t calculateVar(void)
{

	static int32_t bufferSum;
	static int32_t deltaSrd = 0;
	static int32_t deltaValue = 0;
	static uint8_t index_data;
	static uint8_t temp_data = 0u;
	static uint8_t nibbleFlag = LOWER_NIBBLE;

	bufferSum = 0;
	index_data = 0u;
	/* **************************************************** */
	/* Iterate through the 128 nibbles in the 64 byte array */
	/* **************************************************** */
	while (index_data < VARIANCE_HISTORY_BUFFER_SIZE){

		/* ************************** */
		/* Check state of nibble flag */
		/* ************************** */
		temp_data = u8historyBuf[index_data];

		if (nibbleFlag == LOWER_NIBBLE)
		/* ************************** */
		/* Read data from lower nibble */
		/* ************************** */
		{
			temp_data = multiplyBy16(temp_data & 0x0fu); /* Multiple by 16 to achieve more resolution	*/
			nibbleFlag = UPPER_NIBBLE;

		}
		else
		/* ************************** */
		/* Read data from upper nibble */
		/* ************************** */
		{
			temp_data = temp_data >> 4;
			temp_data = multiplyBy16(temp_data); /* Multiple by 16 to achieve more resolution */

			nibbleFlag = LOWER_NIBBLE;
			index_data++;
		}
		/* ************************** */
		/* Sum the (each value - the average)^2 */
		/* ************************** */

		deltaValue = ((temp_data - strctVar.u8CurrentCoAverage)); /* Calc difference */

		deltaSrd = (deltaValue * deltaValue);

		bufferSum = (bufferSum + deltaSrd);

	}

	bufferSum = divideBy128(bufferSum);
	bufferSum++;
	return ((uint32_t) bufferSum);
}

/**
 * DoVariance
 * @brief This is the main entry point for variance functionality
 * @param averageValue  range: 0 - 255  average of the last 128 readings
 *
 */

void DoVariance(void)
{
	/**
	 * These are the variable associated with CO reading and Variance calculation time
	 */
	static struct {
			uint16_t CurrentCO; /**< Compensated CO values */
			uint32_t variance_start_time; /**< variance Calculation start time */
			uint32_t time; /**< time */
			uint32_t Hours_old;
			uint32_t Hours;
			uint32_t variance_threshold_time;
			uint8_t EolFault;
	} strctCO;
	static uint32_t hour_ct = 0;
	/* read the CO value */
	strctCO.CurrentCO = (uint16_t) getCoAfterCompensation();
	DEBUG_CO_VAR("\t CO", true, strctCO.CurrentCO);
	/* update the strike accordingly */
	strctCO.variance_threshold_time = getthreshouldhours();
	/* Strike Count for First 128 Hours is 4 */
/*	if (strctCO.variance_threshold_time < 128u){
		strctVar.u8strikeHiCountThreshold = 4u;
	}
	else{
		strctVar.u8strikeHiCountThreshold = 255u;
	}*/
	/*Max Variable strike count*/

	strctVar.u8strikeHiCountThreshold = 255u;
/* Test*/
	/*strctVar.u8strikeHiCountThreshold = 2u;*/
	DEBUG_CO_VAR("\t default strike", true, strctCO.variance_threshold_time);

	switch (strctVar.u8VarianceState) {
		/*************************************************************/
		/* Ensure that the  CO level is below 10 for 128 readigs */
		/* If these conditions are not met - reset the timer        */
		/*************************************************************/
		case ActivationStabilisationState:
			/******************/

			/* State Actions */
			/******************/
			/* Check if the Current CO is below the ACTIVATION_CO_LEVEL */
			if (((uint32_t)(strctCO.CurrentCO) <= ACTIVATION_CO_LEVEL)){
				/* Is is below so increment the ActivationCount */
				strctVar.u8ActivationCount++;
				DEBUG_CO_VAR("\tAct ct ", true, strctVar.u8ActivationCount);
			}
			else{
				/* Current CO is a above ACTIVATION_CO_LEVEL */
				/* So reset the ActivationCount */
				strctVar.u8ActivationCount = 0;
			}
			/**************************/
			/* State Exit Conditions */
			/**************************/
			/* ActivationCount has exceeded ACTIVATION_COUNT_LIMIT so move to */
			/* Accumulation Stabilisation State */

			if (strctVar.u8ActivationCount > ACTIVATION_COUNT_LIMIT){
				strctVar.u8VarianceState = AccumulationStabilisationState;
				DEBUG_CO_VAR("\tAccumulationStabilisationState ", true, strctVar.u8ActivationCount);
			}
			if (strctVar.u8stabilisationCount >= (uint8_t)(STABILISATION_FAULT_LIMIT)){
				/* State Actions */
				writeFaultLeakedSensorFault();
				DEBUG_CO_VAR("\tVariance Faults ", false, false);
			}

			break;
			/*************************************************************/
			/* Now that the Raw CO is below 16, allow the Accumulation  */
			/* buffer to fill with 128 minutes worth of data            */
			/*************************************************************/
		case AccumulationStabilisationState:

			/******************/
			/* State Actions */
			/******************/
			/* Check if the Current CO is below the ACTIVATION_CO_LEVEL */
			if (((uint32_t)(strctCO.CurrentCO) < ACTIVATION_CO_LEVEL)){
				/* Is is below so increment the AccumulationCount */
				strctVar.u8AccumulationCount++;
				/* Store the value in the History Buffer */
				strctVar.u8CurrentCoOld = historyDataIn((uint8_t) strctCO.CurrentCO);
				/* Pass the value to the calculateAverage function */
				calculateAverage((uint8_t)(strctCO.CurrentCO), (uint8_t) strctVar.u8CurrentCoOld);
				DEBUG_CO_VAR("\tAccumStable ", true, strctVar.u8AccumulationCount);
			}
			else{
				/**************************/
				/* State Exit Condition  */
				/**************************/
				/* Current CO is a above ACTIVATION_CO_LEVEL */
				/* Reset ActivationCount */
				strctVar.u8AccumulationCount = 0;
				/* Reset ActivationCount */
				strctVar.u8ActivationCount = 0;
				/* Clear the history buffer */
				clearHistoryBuffer();
				initVariance();
				/* Move back to the ActivationStabilisationState */
				strctVar.u8VarianceState = ActivationStabilisationState;
				strctVar.u8stabilisationCount++;
			}

			/**************************/
			/* State Exit Conditions */
			/**************************/
			/* ActivationCount has exceeded ACTIVATION_COUNT_LIMIT so move to */
			/* Variance Running State */
			if (strctVar.u8AccumulationCount > ACCUMULATION_COUNT_LIMIT){
				strctVar.u8VarianceState = VarianceRunningState;
				strctVar.oneHourVarianceTick = false;
				/* Time start for Running state */
				strctCO.variance_start_time = get_currentTime();
			}

			break;
			/*************************************************************/
			/*  Now that the history buffer is full, start performing   */
			/*  the Variance calculation                                */
			/*                                                          */
			/*************************************************************/
		case VarianceRunningState:

			/******************/
			/* State Actions */
			/******************/
			/* Check if the Raw CO is below the ACTIVATION_CO_LEVEL */
			if (((uint32_t)(strctCO.CurrentCO) < ACTIVATION_CO_LEVEL)){
				/* Update the History Buffer */
				strctVar.u8CurrentCoOld = historyDataIn((uint8_t) strctCO.CurrentCO);
				/* Perform the Average Calculation */
				calculateAverage((uint8_t)(strctCO.CurrentCO), (uint8_t)(strctVar.u8CurrentCoOld));

				/* Calculate the current Variance every minute - This might need to change to once every hour */
				strctVar.u16CurrentCoVar = calculateVar();

				DEBUG_CO_VAR("\tVariance every minute ", true, strctVar.u16CurrentCoVar);

				strctVar.u8zeroCount = historyBuffZeroCount();

				DEBUG_CO_VAR("\t  bf Strike Count", true, strctVar.u8highVarStrikeCnt);
				DEBUG_CO_VAR("\t zero", true, strctVar.u8zeroCount);

				/* Calculate Minimum Variance in last 32 hours  */
				if ((strctVar.u16CurrentCoVar < strctVar.u16minimumVar32Hours) && (strctVar.u16CurrentCoVar != 0)){
					strctVar.u16minimumVar32Hours = strctVar.u16CurrentCoVar;
				}

				/* Calculate maximum Variance in last 32 hours  */
				if ((strctVar.u16CurrentCoVar > strctVar.u16maximumVar32Hours) && (strctVar.u16CurrentCoVar != 0xffff)){
					strctVar.u16maximumVar32Hours = strctVar.u16CurrentCoVar;
				}

				/* Check to see if 32 hours has elapsed */

				strctCO.time = get_currentTime();
				strctCO.time = (strctCO.time - strctCO.variance_start_time);
				DEBUG_CO_VAR("\tstrctCO.time ", true, strctCO.time);
				if (strctCO.time < 3600){
					strctCO.Hours = 0U;
					strctCO.Hours_old = 0u;

				}
				else{
					strctCO.Hours = (uint32_t)(strctCO.time / (uint32_t) 3600);
					if (strctCO.Hours > strctCO.Hours_old){
						strctCO.Hours_old = strctCO.Hours;
						strctVar.oneHourVarianceTick = true;
						DEBUG_CO_VAR("\t after one Hours ", false, false);
						DEBUG_CO_VAR("\t  MAX ", true, strctVar.u16maximumVar32Hours);
						DEBUG_CO_VAR("\t  MIN", true, strctVar.u16minimumVar32Hours);
						hour_ct++;

					}

				}
				DEBUG_CO_VAR("\t Hour Count ", true, hour_ct);
				/* If 32 hours has elapsed reset to the minimum variable to 0xffff */
				if ((strctCO.Hours % 32U) == 0U){

					strctVar.u16maximumVar32Hours = 0U;
					strctVar.u16minimumVar32Hours = 0xffff;
				}

				/* Clear that Strike Count at any point the Variance drops below the threshold. */
#ifdef EEPROM
				calibration_verifyData();

				uint16_t u16VarThreshold = calibration_getCOVarianceThreshold();
#endif
				const uint16_t u16VarThreshold = cocal_get_co_var_thresh( ); 			/* Test*/

				if (strctVar.u16CurrentCoVar <= u16VarThreshold){
					DEBUG_CO_VAR("\t drop nt ", false, false);
					/* Variance is less than the threshold: clear the strike count and remain in VarianceRunningState */
					/* This is the change from the previous Product, Observed drop in threshold value for some times */
					/* Clear or increment strike count, when we compare with threshold variance:(i.e every hour)  */
					/*strctVar.u8highVarStrikeCnt = 0u;*/

				}
#ifdef EEPROM
				uint32_t Faults = faults_getFlags();
				/* Verify EOL fault*/

				if (((Faults & (1u << FAULT_ID_EOL)) != 0u)){
					strctCO.EolFault = 1u;
				}
#endif
				/* Check if the hourly tick has occurred */
				/* Adjust this tick rate for testing ???? */
				if ((strctVar.oneHourVarianceTick == true) && (strctCO.EolFault == 0U)){
					/* Clear the hourly tick flag */
					strctVar.oneHourVarianceTick = false;

					DEBUG_CO_VAR("\t After oneHourVarianceTick", true, strctVar.u16CurrentCoVar);
					/* Disable the strike count during the first 128 hours	*/
					/* This is to allow the algorithm to be direct acting directly after calibration */

					/* State Exit Conditions */
					/* Check current Variance against the Threshold */
					if (strctVar.u16CurrentCoVar > u16VarThreshold){
						/* Variance is greater than threshold, so jump to the high Threshold State */
						/* Increment the highVarStrikeCnt */

						strctVar.u8highVarStrikeCnt++;
						/* Measure the number of zero readings in the 128 reading history buffer */
						strctVar.u8zeroCount = historyBuffZeroCount();

						/* Check to see if the zeroCount > ZERO_COUNT_LIMIT */
						if (strctVar.u8zeroCount > ZERO_COUNT_LIMIT){
							/* true clear the strike count  */
							DEBUG_CO_VAR("\t Clear ", false, false);
							strctVar.u8highVarStrikeCnt = 0u;
						}

						/* Strike Count > Limit */
						if (strctVar.u8highVarStrikeCnt >= strctVar.u8strikeHiCountThreshold){
							strctVar.u8VarianceState = VarianceFaultDetectedState;
							DEBUG_CO_VAR("\t Strike Count", true, strctVar.u8highVarStrikeCnt);
						}
					}
					/* Every Hour: If Threshold is less than expected then Clear the strike Count*/
					else{
						DEBUG_CO_VAR("\t Clear var ", false, false);
						strctVar.u8highVarStrikeCnt = 0u;
					}

				}
			}
			else{
				strctVar.u8AccumulationCount = 0U;
				strctVar.u8ActivationCount = 0U;
				clearHistoryBuffer();
				initVariance();
				strctVar.u8VarianceState = ActivationStabilisationState;
				strctVar.u8stabilisationCount = 0U;
			}

			break;

			/*************************************************************/
			/*  The variance threshold has been exceeded.  Log the fault*/
			/*  and move to the idle state.								*/
			/*                                                          */
			/*************************************************************/
		case VarianceFaultDetectedState:
			DEBUG_CO_VAR("\t Variance Fault detected", true, strctCO.CurrentCO);
			/* State Actions */
			writeFaultLeakedSensorFault();
			/* State Exit Conditions */
			strctVar.u8VarianceState = VarianceRunningState;
			/* Idle state. */

			break;

		default:
			DEBUG_CO_VAR("\t Default status ", DEF_NULL, DEF_NULL);

			break;

	}

}

/**
 * writeFaultLeakedSensorFault
 * @brief This is for fault logging
 *
 */
void writeFaultLeakedSensorFault(void)
{
#ifdef EEPROM
	log_CoVarianceEvent(strctVar.u16maximumVar32Hours);
	DEBUG_CO_VAR("\t Fault ", false, false);
	/* Variance Fault */

	faults_changeState(FAULT_ID_CO_VARIANCE, FAULT_ACTION_SETFAULT);
#endif
}

/**
 * initVariance
 * @brief This is the main entry point for variance functionality
 */
void initVariance(void)
{
	strctVar.u8nibbleInFlag = LOWER_NIBBLE;
	strctVar.u16CurrentCoVar = 0u;
	strctVar.u16CurrentCoAccum = 0u;
	strctVar.u8CurrentCoAverage = 0u;
	strctVar.u8CurrentCoOld = 0u;
	strctVar.u8nibbleInFlag = 0u;
	strctVar.u8iterations = 0u;
	strctVar.u8ActivationCount = 0u;
	strctVar.u8AccumulationCount = 0u;
	strctVar.u8VarianceState = ActivationStabilisationState;
	strctVar.oneHourVarianceTick = false;
	strctVar.u8highVarStrikeCnt = 0u;
	strctVar.u8lowVarStrikeCnt = 0u;
	strctVar.u16minimumVar32Hours = 0xffff;
	strctVar.u16maximumVar32Hours = 0u;
	strctVar.u8zeroCount = 0u;
	strctVar.u8strikeHiCountThreshold = 255u;
	strctVar.u8strikeLoCountThreshold = 0u;

	DEBUG_CO_VAR("\t init variance", false, false);
	DEBUG_CO_VAR("\t*********", false, false);
}
/**
 * @brief This is for hours calculation
 */
uint32_t getthreshouldhours(void)
{
	uint32_t current_time;
	current_time = get_currentTime();
	if (current_time < 3600u){
		current_time = 0u;
	}
	else{
		current_time = (uint32_t)(current_time / (uint32_t) 3600U);
	}
	return current_time;
}
