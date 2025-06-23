/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef SPI_COMMS_H_
#define SPI_COMMS_H_

/********************************************************************************************************
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
********************************************************************************************************/

#include <stdint.h>

#include "debug.h"
#include "os.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

#define DEF_SPI_MCU2_TIMED_OUT        (OS_FLAGS)0x01
#define DEF_SPI_MCU2_RETRY_TIMED_OUT  (OS_FLAGS)0x02
#define DEF_SPI_MCU2_TIME_OUT_RECOVER (OS_FLAGS)0x04
#define DEF_SPI_MCU2_20_MIN_TIMEOUT   (OS_FLAGS)0x08

/********************************************************************************************************
*********************************************************************************************************
*                                                EXTERNS
*********************************************************************************************************
********************************************************************************************************/

extern OS_FLAG_GRP		InterMCUSPIComms_CommandsFlag;
extern OS_FLAG_GRP 		InterMCUSPIComms_TimeoutFlag;

extern OS_TCB InterMCUSPIComms_TaskTCB;
extern OS_TCB InterMCUSPIComms_TimeoutTaskTCB;
bool get_SPI_comms_status(void);
#define SPI_COMMS_FAIL_COUNT 10u
/********************************************************************************************************
*********************************************************************************************************
*                                            DATA STRUCTURES
*********************************************************************************************************
********************************************************************************************************/

typedef enum {
  /*00*/SPICmdInvalid = 0u,
  /*01*/SPICmdCurrentValues,
  /*02*/SPICmdLogbookRecord,
  /*03*/SPICmdDemountingLogbook,
  /*04*/SPICmdTimeZone,
  /*05*/SPICmdCountersDates,
  /*06*/SPICmdFwVersion,
  /*07*/SPICmdAlarm,
  /*08*/SPICmdOperatingMode,
  /*09*/SPICmdProductionBLOB,
  /*10*/SPICmdTerminateProduction,
  /*11*/SPICmdRadioTest,
  /*12*/SPICmdRadioLinkError,
  /*13*/SPICmdTrigOCDetection,
  /*14*/SPICmdOCdetectionResult,
  /*15*/SPICmdLaserCalibration,
  /*16*/SPICmdSetDateandTime,
  /*17*/SPICmdRadioDuration,
  /*18*/SPICmdRadioPrivateData,
  /*19*/SPICmdRamFault, /*future use*/
  /*20*/SPICmdEnterEMx, /*future use*/
  /*21*/SPICmdToggleConfigAir,
  /*22*/SPICmdAriringLight,
  /*23*/SPICmdResetCounter,
  /*24*/SPICmdAlarm_Forwarding,
  /*25*/SPICmdSlaveTransfer,
  /*26*/SPICmdSuspend,
  /*27*/TotalSPICommands
} SPICOMMS_COMMANDS;

/* This SPI Command Flags  Can be use by thread to set the Flags */
#define SPI_CMD_Invalid          (0u)
#define SPI_CMD_Current_value    (0x02u)
#define SPI_CMD_Logbook_value    (0x04u)
#define SPI_CMD_demoutingLogbook (0x08u)
#define SPI_CMD_Time_Zone        (0x10u)
#define SPI_CMD_Countersdates    (0x20u )
#define SPI_CMD_Firmware_version  (0x40u)
#define SPI_CMD_Alarm             (0x80u )
#define SPI_CMD_Operating_modes  (0x100u )
#define SPI_CMD_Production_Bob  (0x200u )
#define SPI_CMD_Termination_production  (0x400u )
#define SPI_CMD_Radio_test  (0x800u )
#define SPI_CMD_Radio_Link_error  (0x1000u )
#define SPI_CMD_Trig_Detection  (0x2000u )
#define SPI_CMD_OC_result  (0x4000 )
#define SPI_CMD_Laser_Calibration  (0x8000u )
#define SPI_CMD_Date_Time (0x10000u)
#define SPI_CMD_Radio_Duration  (0x20000u )
#define SPI_CMD_Radio_Private_Data  (0x40000u )
#define SPI_CMD_Ram_Fault  (0x80000u )
#define SPI_CMD_Enter_Em  (0x100000u )
#define SPI_CMD_Toggle_Config_Air  (0x200000u)
#define SPI_CMD_Airing_Light  (0x400000u )
#define SPI_CMD_Reset_Counter  (0x800000u )
#define SPI_CMD_Alarm_forwarding  (0x1000000u)
#define SPI_CMD_Slave_Transfer (0x2000000u )
#define SPI_CMD_Suspend (0x4000000u )

#ifdef DEBUG_BUILD
#define NO_OF_SPI_HISTORY_ENTRIES			(8u)
#define SPI_APP_DATA_LEN              (180u)
#define SPI_RX_DATA_LEN               (20u)
#endif

typedef enum {
	Idle, 									/* Idle */
	MasterInitiated, 						/* Communication inititated by MCU1 */
	SlaveInitiated							/* Communication inititated by MCU2 */
} SPICOMMS_MODE;

typedef struct {
	uint8_t 			BeginFrame;
	uint16_t 			Length;         //The length is the size of the application data + command code
	uint8_t 			LinkLayerStatus;
	uint8_t 			Command;
	uint8_t            *Buffer;
	uint16_t 			CRC;
	uint8_t 			EndFrame;
} SPICOMMS_PACKET;

typedef struct {
	SPICOMMS_MODE 		Mode;
	uint8_t 			Command;
	SPICOMMS_PACKET 	TxPacket;
	SPICOMMS_PACKET 	RxPacket;
} SPICOMMS_DATA_PACKET;

#ifdef DEBUG_BUILD
typedef struct 
{
  uint8_t 			cmd;
  uint8_t       app_data_cmd;
  uint16_t      app_data_len;
  uint8_t       app_data[SPI_APP_DATA_LEN];
  uint8_t       rx_data[SPI_RX_DATA_LEN];
  uint32_t      timestamp;
}INTER_MCU_SPI_COMMS_HISTORY_ELEMENT;

typedef struct 
{
  INTER_MCU_SPI_COMMS_HISTORY_ELEMENT element[NO_OF_SPI_HISTORY_ENTRIES];
  uint8_t writePtr;
  uint8_t totalElements;
}INTER_MCU_SPI_COMMS_HISTORY;
#endif
/********************************************************************************************************
*********************************************************************************************************
*                                               FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

void InterMCU_SPIComms_Init(void);
SPICOMMS_MODE SPIComms_ModeGet(void);
void SPIComms_WriteAFE(uint8_t add, uint8_t data);
uint8_t SPIComms_ReadAFE(uint8_t regAddr, uint8_t pData);
void SPIComms_Send_Data_to_MCU2(uint32_t data_flags);
/****************************************************************************************************//**
*                                             SPIComms_TransmitDataToSlave()
*
* @brief   	Transmit data to slave mcu.
*
* @param	p_packet	pointer to data packets.
********************************************************************************************************/
bool SPIComms_WriteMCU2(SPICOMMS_DATA_PACKET *p_packet);

/****************************************************************************************************//**
*                                             SPIComms_Read()
*
* @brief	Receive data from AFE.
*
* @param	regAddr		address of register to read.
*
* @param	pData	    dummy data.
********************************************************************************************************/
bool SPIComms_ReadMCU2(SPICOMMS_DATA_PACKET *p_packet);

/****************************************************************************************************//**
*                                             SPIComms_Transfer()
*
* @brief   	SPI data transfer between mcu1 and mcu2.
*
* @param	p_packet	pointer to data packets.
********************************************************************************************************/
void SPIComms_Transfer(SPICOMMS_DATA_PACKET *p_packet);

/****************************************************************************************************//**
*                                              SPIComms_AcquireBus()
*
* @brief	Acquire the SPI bus
*
* @note		(1) The following function needs to be called prior to using any SPI Read/Write functionality  
********************************************************************************************************/
void SPIComms_AcquireBus(void);

/****************************************************************************************************//**
*                                              SPIComms_ReleaseBus()
*
* @brief	Release the SPI bus
*
* @note		(1) The following function needs to be called when the bus access is no longer needed 
********************************************************************************************************/
void SPIComms_ReleaseBus(void);


void SPIComms_HandleShortTimeout();
void SPIComms_HandleLongTimeout();
#ifdef DEBUG_BUILD
INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIComms_GetTxPacketElement(uint8_t Index);
INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIComms_GetTxPacketLastElement(void);
uint8_t SPIComms_GetTxPacketNoOfElements(void);
uint8_t SPIComms_GetTxPacketCurrentIndex(void);
#endif

/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* SPI_COMMS_H_ */
