/********************************************************************************************************
*
* 									SPIC COMMUNICATION HANDLER
*
* Filename			: spi_comms.c
* Version			: V1.00
* Programmer(s)		: AUR
* NDI Updated for multiple SPi commands Handling
* Updated by M.F for BUS Acquiring logic
********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
********************************************************************************************************/

#include "spi_comms.h"
#include "hal_gpio.h"
#include "os.h"
#include "telegram.h"
#include "cmsis_gcc.h"
#include "hal_LETimer.h"
#include "hal_BURTCTimer.h"
#include "app.h"
#include "spi_driver.h"
#include "events.h"
#include "em_eusart.h"
#include "os.h"
#include "fault_handler.h"
#include "string.h"
/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

#define DEF_MCU2_TX_RX_BUFFER_SIZE 			  (512u)
#define DEF_AFE_TX_RX_BUFFER_SIZE 			  (2u)
#define DEF_SPI_TIMEOUT_TASK_STK_SIZE 	  (256u)
#define DEF_REPEAT_COUNTER_MAX            (3u)
#define DEF_RETRY_ENABLE 					        (1u)
#define DEF_SPI_Comms_Retry_TIMOUT_ms		  (2000u)
#define DEF_SPI_Comms_TIMOUT_ms		        (1000u)
#define DEF_SPI_COMMS_RECOVERY_TIMEOUT_ms	(1200000u)


/*********************************************** Local typedefs *************************************************/

typedef struct 
{
    uint8_t SPIComms_AFETxBuffer[DEF_AFE_TX_RX_BUFFER_SIZE];
    uint8_t SPIComms_AFERxBuffer[DEF_AFE_TX_RX_BUFFER_SIZE];
    uint8_t SPIComms_MCU2TxBuffer[DEF_MCU2_TX_RX_BUFFER_SIZE];
    uint8_t SPIComms_MCU2RxBuffer[DEF_MCU2_TX_RX_BUFFER_SIZE];
    SPICOMMS_MODE SPIComms_CurrentMode;
    volatile bool SPIComms_Timeout;
    bool spi_comms_status;
    bool spiErrorTimeout;
    volatile bool twenty_min_timer_started;
    int8_t repeat_counter;
}spiContext_t;


/********************************************************************************************************
*********************************************************************************************************
*                                              VARIABLES
*********************************************************************************************************
********************************************************************************************************/

OS_TCB InterMCUSPIComms_TaskTCB;
OS_FLAG_GRP InterMCUSPIComms_CommandsFlag;
static OS_MUTEX InterMCUComms_Mutex;
static CPU_STK InterMCUSPIComms_TaskStack[SPI_COMMS_TASK_STK_SIZE];
static spiContext_t spi_context = {0};

/********************************************************************************************************
*********************************************************************************************************
*                                            PROTOTYPES
*********************************************************************************************************
********************************************************************************************************/

static void InterMCUC_CommsTask(void *p_arg);
//static void InterMCU_TimeoutTask(void *p_arg);
bool SPIComms_WriteMCU2(SPICOMMS_DATA_PACKET *p_packet);
bool SPIComms_ReadMCU2(SPICOMMS_DATA_PACKET *p_packet);
void SPIComms_Transfer(SPICOMMS_DATA_PACKET *p_packet);
uint32_t FLAGS_BIT_INDEX (uint8_t data);


#ifdef DEBUG_BUILD
static INTER_MCU_SPI_COMMS_HISTORY intermcu_spi_comms_history;
void SPIComms_initSPIHistory(void);
#endif

/********************************************************************************************************
*********************************************************************************************************
*                                                FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

/****************************************************************************************************//**
*                                              SPIComms_Init()
*
* @brief	Create SPI Communication related tasks and flags.
*
* @note		(1) The following function needs to be called at the time of application initialisation.
********************************************************************************************************/
void InterMCU_SPIComms_Init(void)
{
	RTOS_ERR err;

	OSFlagCreate(&InterMCUSPIComms_CommandsFlag,
			     "InterMCU SPI Comms Commands Flag",
				 0,
				 &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

	OSMutexCreate(&InterMCUComms_Mutex, "InterMCU SPI Mutex", &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#ifdef DEBUG_BUILD
  	SPIComms_initSPIHistory();
#endif
	OSTaskCreate(&InterMCUSPIComms_TaskTCB,
				 "InterMCU SpiComms Task",
				 InterMCUC_CommsTask,
				 &InterMCUSPIComms_TaskTCB,
				 SPI_COMMS_TASK_PRIO,
				 &InterMCUSPIComms_TaskStack[0],
				 (SPI_COMMS_TASK_STK_SIZE / 10u),
				 SPI_COMMS_TASK_STK_SIZE,
				 0u,
				 0u,
				 DEF_NULL,
				 (OS_OPT_TASK_STK_CLR ),
				 &err);
	EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    spi_context.spi_comms_status = 0;
    spi_context.repeat_counter = 0;
    spi_context.SPIComms_CurrentMode = Idle;
    spi_context.spiErrorTimeout = false;
    spi_context.SPIComms_Timeout = false;
    spi_context.twenty_min_timer_started = false;
}

/****************************************************************************************************//**
 *                                             InterMCUC_CommsTask()
 *
 * @brief   	Task to handle spi data exchange requests with MCU2.
 *
 * @param	p_arg	pointer to arguments.
 ********************************************************************************************************/
static void InterMCUC_CommsTask(void *p_arg)
{
    RTOS_ERR err;
    OS_FLAGS flags_spi;
    const OS_TCB *const ptrToMyTCB = (const OS_TCB *const)p_arg;
    OSTaskRegSet(DEF_NULL, 0, ptrToMyTCB, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    SPICOMMS_DATA_PACKET packet;
    /* Modified for Multiple Flags set by Modules */
    while (true)
    {
        flags_spi = OSFlagPend(&InterMCUSPIComms_CommandsFlag,
                   0xffffffffu,
               0,
               OS_OPT_PEND_FLAG_SET_ANY |
               OS_OPT_PEND_BLOCKING |
               OS_OPT_PEND_FLAG_CONSUME,
               DEF_NULL,
               &err);
               /* If no RTOS Error and SPI Bus not in Fault/Error State only then 
               process received SPI messages, else just consume the message and Ignore it*/
        if ((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) && (false == spi_context.spiErrorTimeout))
        {
            /* To handle Multiple SPI Comms*/
            //LEDBuzz_AcquireBuzzer();
            //SPIComms_AcquireBus ();
            /* SPI MAX Hold Time in 800us for 1 tx, if all flag set then max ~25 milisecond SPI bus will be blocked */
            if ((flags_spi & FLAGS_BIT_INDEX(SPICmdSlaveTransfer))!=0u)
            { /* Check if spi transfer is requested by a local task or mcu2?  */
                packet.Mode = SlaveInitiated;
                SPIComms_Transfer(&packet); /* process request                        */
            } //Slave Communication
            else
            { // Master Communication
                packet.Mode = MasterInitiated;
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdCurrentValues))!=0u)
                {
                    packet.TxPacket.Command = DEF_CURRENT_VALUES;
                    SPIComms_Transfer(&packet); /* process request                        */
                }
                // if ((flags_spi & FLAGS_BIT_INDEX (SPICmdLogbookRecord)) != 0u)
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdLogbookRecord))!=0u)
                {
                    packet.TxPacket.Command = DEF_LOGBOOK_RECORD;
                    SPIComms_Transfer (&packet); /* process request                        */
                }

                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdCountersDates))!=0u)
                {
                    packet.TxPacket.Command = DEF_COUNTERS_AND_DATES;
                    SPIComms_Transfer (&packet); /* process request                        */
                }

                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdFwVersion))!=0u)
                {
                    packet.TxPacket.Command = DEF_FW_VERSION;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdAlarm))!=0u)
                {
                    packet.TxPacket.Command = DEF_ALARM;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdOperatingMode))!=0u)
                {
                    packet.TxPacket.Command = DEF_OPERATING_MODE;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdProductionBLOB))!=0u)
                {
                    packet.TxPacket.Command = DEF_PRODUCTION_BLOB;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdTerminateProduction))!=0u)
                {
                   packet.TxPacket.Command =  DEF_TERMINATE_PRODUCTION;
                   SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdRadioTest))!=0u)
                {
                    packet.TxPacket.Command = DEF_RADIO_TEST;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdRadioLinkError))!=0u)
                {
                     packet.TxPacket.Command =  DEF_ALARM_RADIO_LINK_ERROR;
                     SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdTrigOCDetection))!=0u)
                {
                     packet.TxPacket.Command =  DEF_TRIG_OC_DETECTION;
                     SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdSetDateandTime))!=0u)
                {
                    packet.TxPacket.Command = DEF_DATE_AND_TIME;
                    SPIComms_Transfer (&packet); /* process request                        */
                }

#if 0
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdLaserCalibration))!=0u)
                {
                    packet.TxPacket.Command =  DEF_LASER_CALIBRATION;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
#endif
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdEnterEMx))!=0u)
                {
                    packet.TxPacket.Command = DEF_ENTER_EM;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdToggleConfigAir))!=0u)
                {
                    packet.TxPacket.Command =   DEF_TOGGLE_AIRING_RECOMMENDATION;
                    SPIComms_Transfer (&packet); /* process request                        */
                }

                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdAriringLight))!=0u)
                {
                    packet.TxPacket.Command = DEF_AIRING_LIGHT;
                    SPIComms_Transfer (&packet); /* process request                        */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdDemountingLogbook))!=0u)
                {
                    packet.TxPacket.Command =  DEF_DEMOUNTING_LOGBOOKS;
                    SPIComms_Transfer (&packet); /* process request */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdRadioPrivateData))!=0u)
                {
                    packet.TxPacket.Command =  DEF_RADIO_CONFIG_SWITCH;
                    SPIComms_Transfer (&packet); /* process request */
                }
                if ((flags_spi & FLAGS_BIT_INDEX(SPICmdSuspend))!=0u)
                {
                    //No SPI comms for 20 Minutes. Raise a Major fault.
                    FaultHandler_FaultSet(MCU2SPICommsTimeoutFault);
                    spi_context.spiErrorTimeout = true;
                }
          }
          //LEDBuzz_ReleaseBuzzer();
          //SPIComms_ReleaseBus ();
    } //if error
    else
    {
        DEBUG_SPI("\n\rInterMCU SPI Comms In Error State. No Msg Processed", false, 0);
    }
  }//while
}//end

/****************************************************************************************************//**
*                                               SPIComms_HandleShortTimeout()
*
* @brief	Callback function that is being called by the LE 500ms timeout ISR.
* 
********************************************************************************************************/
void SPIComms_HandleShortTimeout()
{
    spi_context.SPIComms_Timeout = true;
}

/****************************************************************************************************//**
*                                               SPIComms_HandleLongTimeout()
*
* @brief	Callback function that is being called by the LE 20 Min timeout ISR.
* 
********************************************************************************************************/
void SPIComms_HandleLongTimeout()
{
    RTOS_ERR err;
    //Send a Command to SPI Comms task to ignore any more SPI messages to/from MCU2
    (void) OSFlagPost(&InterMCUSPIComms_CommandsFlag, SPI_CMD_Suspend, OS_OPT_POST_FLAG_SET, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/****************************************************************************************************//**
*                                               SPIComms_ModeGet()
*
* @brief	Get the current mode of spi communication.
*
* @return	Current state of the communication i.e. Idle, MasterInitiated, SlaveInitiated.
********************************************************************************************************/
SPICOMMS_MODE SPIComms_ModeGet(void)
{
	return spi_context.SPIComms_CurrentMode;
}

/****************************************************************************************************//**
*                                             SPIComms_Transfer()
*
* @brief   	SPI data transfer between mcu1 and mcu2.
*
* @param	p_packet	pointer to data packets.
********************************************************************************************************/
void SPIComms_Transfer(SPICOMMS_DATA_PACKET *p_packet)
{
    RTOS_ERR err;
    if (p_packet!=NULL)
    {
        p_packet->TxPacket.Buffer = spi_context.SPIComms_MCU2TxBuffer;					/* assign tx and rx buffers*/
        p_packet->RxPacket.Buffer  = spi_context.SPIComms_MCU2RxBuffer;
        spi_context.SPIComms_CurrentMode = p_packet->Mode;										  /* set mode*/
        GPIO_MCU2InterruptDisable();                                            /* disable interrupt              */
        //spi_context.SPIComms_Previous_timeout = spi_context.SPIComms_Timeout;   /* clear timeout flag*/

        switch (spi_context.SPIComms_CurrentMode)
        {
          case MasterInitiated:														                      /* transfer is requested by local module*/
          {
            TELEGRAM_PrepareTransmitPacket(p_packet);								            /* prepare data packet*/
            bool success = false;
            spi_context.repeat_counter = 0;
            while(spi_context.repeat_counter < (int8_t)DEF_REPEAT_COUNTER_MAX)
            {
                LEDBuzz_AcquireBuzzer();
                SPIComms_AcquireBus ();
                success = SPIComms_WriteMCU2(p_packet);								          /* send data 									*/
                if(true == success)
                {
                    success = SPIComms_ReadMCU2(p_packet);                      /* receive response               */
                    if(true == success)
                    {
                        LEDBuzz_ReleaseBuzzer();
                        SPIComms_ReleaseBus ();
                        break;
                    }
                }
                spi_context.repeat_counter++;
                LEDBuzz_ReleaseBuzzer();
                SPIComms_ReleaseBus ();
                DEBUG_SPI("\n\rSP RETRY1 - Master", true, spi_context.repeat_counter);
                OSTimeDly(DEF_SPI_Comms_Retry_TIMOUT_ms, OS_OPT_TIME_DLY, &err);/* Wait 2sec before retrying */
            }
            /* Error Checking  */
            if ((p_packet->RxPacket.LinkLayerStatus!=0u)||(p_packet->RxPacket.BeginFrame!=DEF_BEGIN_FRAME)||(p_packet->RxPacket.EndFrame!=DEF_END_FRAME))
            {
                DEBUG_SPI("\n\rSP FAIL - Master", false, 0);
                spi_context.spi_comms_status=true;
            }
            else
            {
                DEBUG_SPI("\n\rSP Pass - Master", false, 0);
                spi_context.spi_comms_status=false;
            }
         }
         break;

         case SlaveInitiated:														                        /* transfer is requested by mcu2 */
         {
            bool success = false;
            //LEDBuzz_AcquireBuzzer();
            SPIComms_AcquireBus ();
            success = SPIComms_ReadMCU2(p_packet);
            if(true == success)
            {
                TELEGRAM_ProcessReceivedPacket(p_packet);                       /* process data and generate response packet  */
                success = SPIComms_WriteMCU2(p_packet);
            }
            //LEDBuzz_ReleaseBuzzer();
            SPIComms_ReleaseBus ();
            DEBUG_SPI("\n\rSP RETRY1 - SLAVE", true, spi_context.repeat_counter);
            /* Error Checking  */
            if ((p_packet->RxPacket.LinkLayerStatus!=0u)||(p_packet->RxPacket.BeginFrame!=DEF_BEGIN_FRAME)||(p_packet->RxPacket.EndFrame!=DEF_END_FRAME))
            {
                DEBUG_SPI("\n\rSP FAIL - Slave", false, 0);
                spi_context.spi_comms_status=true;
            }
            else
            {
                DEBUG_SPI("\n\rSP Pass - Slave", false, 0);
                spi_context.spi_comms_status=false;
            }
         }
         break;

         default:
         /* Unknown state */
         break;
        }



       if((false == spi_context.twenty_min_timer_started) && (true == spi_context.SPIComms_Timeout))
       {
           BURTCTimer_Start(TMR_FAILED_SPI_COMMS_TIMEOUT_1, one_shot, SPI_FAILURE_TIMEOUT_PERIOD);
           spi_context.twenty_min_timer_started = true;
           DEBUG_SPI("\n\rInterMCU SPI Comms(Tx) failed. Starting 20 Min Timeout Timer", false, 0);
       }

       if((true == spi_context.twenty_min_timer_started) && (false == spi_context.SPIComms_Timeout))
       {
           (void)BURTCTimer_Stop(TMR_FAILED_SPI_COMMS_TIMEOUT_1);
           spi_context.twenty_min_timer_started = false;
           DEBUG_SPI("\n\rInterMCU SPI Comms Recovered. Stopping 20 Min Timeout Timer", false, 0);
       }

      spi_context.SPIComms_CurrentMode = Idle;		          	                  /* set mode to idle 							*/
      GPIO_MCU2InterruptEnable();                                               /* re-enable the interrupt            */
    }
}

/****************************************************************************************************//**
*                                             SPIComms_TransmitDataToSlave()
*
* @brief   	Transmit data to slave mcu.
*
* @param	p_packet	pointer to data packets.
********************************************************************************************************/
bool SPIComms_WriteMCU2(SPICOMMS_DATA_PACKET *p_packet)
{
  bool retVal = true;

#ifdef DEBUG_BUILD
  intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].cmd = p_packet->TxPacket.Command;
  intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].app_data_cmd = p_packet->TxPacket.Buffer[0];
  intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].app_data_len = p_packet->TxPacket.Length-1;
  intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].timestamp = get_currentTime();
	if(intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].app_data_len < SPI_APP_DATA_LEN)
	{
      memcpy(&intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].app_data[0], &p_packet->TxPacket.Buffer[1], p_packet->TxPacket.Length-1);
	}
#endif
  
  spi_context.SPIComms_Timeout = false;	




  LETimer_start(LETIMER_SPI_TIMEOUT, DEF_SPI_Comms_TIMOUT_ms);                  /* start timer  */
  while ((GPIO_MCU2ReadyPinStatusGet() == DEF_MCU2_READY_PIN_LOW) &&
            (spi_context.SPIComms_Timeout == false))
  {     /* wait for the ready pin to go high - because somebody else want to send */
      ;
  }

  EUSART_Enable(EUSART1, eusartEnable);                                         /* Enable TX and RX     */

  GPIO_MCU2CSLowSet();

  while ((GPIO_MCU2ReadyPinStatusGet() != DEF_MCU2_READY_PIN_LOW) &&
             (spi_context.SPIComms_Timeout == false))
   {     /* wait for the ready pin to go low */
       ;
   }

	if (spi_context.SPIComms_Timeout == false)
	{	  /* transmit begin Frame */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.BeginFrame), DEF_LEN_BEGIN_FRAME_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{	  /* transmit Length - LSB and MSB */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.Length), DEF_LEN_LENGTH_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		/* transmit link layer status */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.LinkLayerStatus), DEF_LEN_LINK_LAYER_STATUS_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		/* transmit data */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.Buffer[0]), p_packet->TxPacket.Length);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		/* transmit crc */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.CRC), DEF_LEN_CRC_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		/* transmit end frame */
	    SPIDriver_Transmit((uint8_t*)&(p_packet->TxPacket.EndFrame), DEF_LEN_END_FRAME_FIELD);
	}

	GPIO_MCU2CSHighSet();	

	while ((GPIO_MCU2ReadyPinStatusGet() != DEF_MCU2_READY_PIN_HIGH) &&
	       (spi_context.SPIComms_Timeout == false))
	{		/* wait for the ready pin to go high */
	    ;
	}

	retVal = (spi_context.SPIComms_Timeout == false);
	LETimer_stop(LETIMER_SPI_TIMEOUT);				                                    /* stop the timer 		*/
	EUSART_Enable(EUSART1, eusartDisable);                                        /* Disable TX and RX   */

  return retVal;
}

/****************************************************************************************************//**
*                                             SPIComms_ReceiveDataFromSlave()
*
* @brief	Receive data from slave mcu.
*
* @param	p_packet	pointer to data packets.
********************************************************************************************************/
bool SPIComms_ReadMCU2(SPICOMMS_DATA_PACKET *p_packet)
{

  bool retVal = true;
  p_packet->RxPacket.BeginFrame = 0x00;
  p_packet->RxPacket.Command = 0x00;
  p_packet->RxPacket.Length = 0x00;
  p_packet->RxPacket.CRC = 0x00;
  spi_context.SPIComms_Timeout = false;

  EUSART_Enable(EUSART1, eusartEnable);           														  /* Enable TX and RX     */
	LETimer_start(LETIMER_SPI_TIMEOUT, DEF_SPI_Comms_TIMOUT_ms);						      /* start timer 	*/

	while((GPIO_MCU2ReadyPinStatusGet() != DEF_MCU2_READY_PIN_LOW) &&
	      (spi_context.SPIComms_Timeout == false) )
	{			/* wait for the ready pin to go low */
		;
	}

  if (spi_context.SPIComms_Timeout == false)
  {
	    GPIO_MCU2CSLowSet();	                                                    /* set CS to low to initiate data transfer            */
  }
	
	while ((p_packet->RxPacket.BeginFrame != DEF_BEGIN_FRAME) &&
	       (spi_context.SPIComms_Timeout == false))
	{		 /* receive begin frame */
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.BeginFrame), DEF_LEN_END_FRAME_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		 /* receive length */
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.Length), DEF_LEN_LENGTH_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		 /* receive link layer status */
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.LinkLayerStatus), DEF_LEN_LINK_LAYER_STATUS_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{		 /* receive command and data 									*/
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.Buffer[0]), p_packet->RxPacket.Length);
	}

	if (spi_context.SPIComms_Timeout == false)
	{    /* receive crc 													*/
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.CRC), DEF_LEN_CRC_FIELD);
	}

	if (spi_context.SPIComms_Timeout == false)
	{	   /* receive end of frame 										*/
		SPIDriver_Receive((uint8_t*)&(p_packet->RxPacket.EndFrame), DEF_LEN_END_FRAME_FIELD);
	}

	while ((GPIO_MCU2ReadyPinStatusGet() != DEF_MCU2_READY_PIN_HIGH) &&
	        (spi_context.SPIComms_Timeout == false))
	{		 /* wait for the ready pin to go high 							*/
		;
	}

#ifdef DEBUG_BUILD
	intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].rx_data[0] = p_packet->RxPacket.Length;
	intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].rx_data[1] = p_packet->RxPacket.LinkLayerStatus;
	intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].rx_data[2] = p_packet->RxPacket.CRC & 0xFF;
	intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].rx_data[3] = (p_packet->RxPacket.CRC >> 8) & 0xFF;
	if(p_packet->RxPacket.Length < 16)
	{
		memcpy(&intermcu_spi_comms_history.element[intermcu_spi_comms_history.writePtr].rx_data[4], &p_packet->RxPacket.Buffer[0], p_packet->RxPacket.Length);
	}
	intermcu_spi_comms_history.writePtr = ((intermcu_spi_comms_history.writePtr + 1) % NO_OF_SPI_HISTORY_ENTRIES);
	if(intermcu_spi_comms_history.totalElements < NO_OF_SPI_HISTORY_ENTRIES)
	{
		intermcu_spi_comms_history.totalElements += 1;
	}
#endif

  if (spi_context.SPIComms_Timeout == false)
  {
	    GPIO_MCU2CSHighSet();				
  }

  retVal = (spi_context.SPIComms_Timeout == false);			/* set CS to high to end data transfer 							*/
  LETimer_stop(LETIMER_SPI_TIMEOUT);				                                    /* stop the timer */
	EUSART_Enable(EUSART1, eusartDisable);                                        /* Disable TX and RX */
  return retVal;
}

#ifdef DEBUG_BUILD
INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIComms_GetTxPacketElement(uint8_t Index)
{
  INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *retVal = NULL;
  if(intermcu_spi_comms_history.totalElements < (NO_OF_SPI_HISTORY_ENTRIES - 1))
  {
    if(Index < intermcu_spi_comms_history.writePtr)
    {
      retVal = &intermcu_spi_comms_history.element[Index];
    }
  }
  else
  {
    retVal = &intermcu_spi_comms_history.element[Index];
  }
  return retVal;
}

INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIComms_GetTxPacketLastElement(void)
{
  INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *retVal = NULL;
  if(intermcu_spi_comms_history.totalElements > 0)
  {
    uint8_t index;
    if(0 == intermcu_spi_comms_history.writePtr)
    {
      index = (NO_OF_SPI_HISTORY_ENTRIES - 1);
    }
    else
    {
      index = intermcu_spi_comms_history.writePtr - 1;
    }
    retVal = &intermcu_spi_comms_history.element[index];
  }
  return retVal;
}

uint8_t SPIComms_GetTxPacketNoOfElements(void)
{
  return intermcu_spi_comms_history.totalElements;
}

uint8_t SPIComms_GetTxPacketCurrentIndex(void)
{
  return intermcu_spi_comms_history.writePtr;
}

void SPIComms_initSPIHistory()
{
  memset(((uint8_t *)&intermcu_spi_comms_history), 0, sizeof(intermcu_spi_comms_history));
}
#endif 

/****************************************************************************************************//**
*                                             SPIComms_Write()
*
* @brief	Transmit data to AFE.
*
* @param	add		address of register to write.
*
* @param	data	data to write into the register.
********************************************************************************************************/
void SPIComms_WriteAFE(uint8_t add, uint8_t data) {

	GPIO_AFECSLowSet();
	spi_context.SPIComms_AFETxBuffer[0] = add;
	spi_context.SPIComms_AFETxBuffer[1] = data;
	SPIDriver_Transmit(spi_context.SPIComms_AFETxBuffer, DEF_AFE_TX_RX_BUFFER_SIZE);
	GPIO_AFECSHighSet();
}


uint8_t SPIComms_ReadAFE(uint8_t regAddr, uint8_t pData) {
	uint8_t result = 0;
	GPIO_AFECSLowSet();
	spi_context.SPIComms_AFETxBuffer[0] = regAddr | 0x80u;  											/* Set bit 7: WR									*/
	spi_context.SPIComms_AFETxBuffer[1] = (pData & 0xffu);            						/* Dummy data										*/
	SPIDriver_Transfer(spi_context.SPIComms_AFETxBuffer, spi_context.SPIComms_AFERxBuffer, DEF_AFE_TX_RX_BUFFER_SIZE);
	GPIO_AFECSHighSet();
	result = spi_context.SPIComms_AFERxBuffer[1];  																/* Note: spi_context.SPIComms_AFERxBuffer[0] is echoed regAddr 	*/
	return result;
}

void SPIComms_AcquireBus(void)
{
	RTOS_ERR err;
	OSMutexPend(&InterMCUComms_Mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}


void SPIComms_ReleaseBus(void)
{
	RTOS_ERR err;
	OSMutexPost(&InterMCUComms_Mutex, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}
/****************************************************************************************************//**
*                                             Send data to MCU2()
*
* @brief  This API will Post SPI Events
* @param   SPI Events: Commands
* @return NULL
********************************************************************************************************/
void SPIComms_Send_Data_to_MCU2(uint32_t  data_flags)
{
  RTOS_ERR err;
#if 1
  (void) OSFlagPost(&InterMCUSPIComms_CommandsFlag,           /* post flag to initiate SPI communication  */
                    data_flags,
                    OS_OPT_POST_FLAG_SET,
                    &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#endif
}

/****************************************************************************************************//**
*                                             Get SPI Status()
*
*@brief   This API will provide SPI Communication Status
*@param   None
*@return  true if SPI in fault

********************************************************************************************************/
bool get_SPI_comms_status(void)
{
  bool spi_comms_error_status=false;
  /* if SPI Comms fails & repeat counter is 0 the set the */
  if ((spi_context.spi_comms_status==true) && (spi_context.repeat_counter==0u))
  {
      spi_comms_error_status=true;
  }
  return spi_comms_error_status;
}

