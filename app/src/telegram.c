/********************************************************************************************************
 *
 *                  TELEGRAM HANDLER
 *
 * Filename     : telegram.c
 * Version      : V1.00
 * Programmer(s)    : AUR
 * Update by NDI ( Due to issues in the DeviceLab Tool Not Able to test data integrity )
 * Function Can be optimised , Due to time limits : not changes the core functionalities/handling
  ********************************************************************************************************/

/********************************************************************************************************
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
********************************************************************************************************/

#include <SHT41.h>
#include "telegram.h"
#include "cmsis_gcc.h"
#include "stdbool.h"
#include "string.h"
#include "crc.h"
#include "os.h"
#include "timeHandler.h"
#include "battery_measurement.h"
#include "led_buzzer.h"
#include "data_logging.h"
#include "sht4x.h"
#include "acquisition_Co.h"
#include "acquisition_Heat.h"
#include "acquisition_Smoke.h"
#include "ambient_light.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "hal_switches.h"
#include "hal_BURTCTimer.h"
#include "common_utils.h"
#include "events.h"
#include "diagnostics.h"
#include "sl_power_manager.h"
#include "acquisition_smoke_dust.h"
#include "app.h"
#include "temp_humid.h"
#include "assistance_light.h"
#include "power_control.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

/* Index - command byte */
#define DEF_INDEX_COMMAND                                            (0u)

/* Length of command + data -in bytes */
#define DEF_LEN_STANDARD_RESPONSE                                    (4u)
#define DEF_LEN_CURRENT_VALUES                                       (27u) /* recently Changes the code */
#define DEF_LEN_LOGBOOK_RECORD                                       (17u)
#define DEF_LEN_DEMOUNTING_LOGBOOKS                                  (161u) /* (20*8 bytes)   */
#define DEF_LEN_TIMEZONE_OFFSET                                      (2u)
#define DEF_LEN_COUNTERS_AND_DATES                                   (65u)
#define DEF_LEN_FW_VERSION                                           (6u)
#define DEF_LEN_ALARM                                                (2u)
#define DEF_LEN_OPERATING_MODE                                       (2u)
#define DEF_LEN_PRODUCTION_BLOB                                      (513u)
#define DEF_LEN_TERMINATE_PRODUCTION                                 (1u)   /* Not defined yet  */
#define DEF_LEN_RADIO_TEST                                           (4u)
#define DEF_LEN_ASSISTANCE_LIGHT                                     (2u)
#define DEF_LEN_TRIG_OC_DETECTION_BIST                               (6u)
#define DEF_LEN_TRIG_OC_DETECTION_2                                  (1u)
#define DEF_LEN_TRIG_OC_DETECTION_3                                  (1u)
#define DEF_LEN_TRIG_OC_DETECTION_4                                  (1u)
#define DEF_LEN_TRIG_OC_DETECTION_5                                  (1u)
#define DEF_LEN_RESULT_OC_DETECTION                                  (4u)   /* Not defined yet or tested  */
//#define DEF_LEN_LASER_RX_DATA                                        (481u)   /* Not defined yet or tested   */
#define DEF_LEN_DATE_AND_TIME                                        (5u)
#define DEF_LEN_RADIO_DURATION_COUNTER                               (17u)
#define DEF_LEN_RAM_FAULTS                                           (1u)
#define DEF_LEN_ENERGY_MODE                                          (2u)
#define DEF_LEN_TOGGLE_AIRING_RECOMMENDATION                         (2u)
#define DEF_LEN_AIRING_LIGHT                                         (2u)
#define DEF_LEN_RESET_COUNTER                                        (3u)
#define DEF_LEN_ALARM_FORWARDING                                     (2u)
#define DEF_LEN_RADIO_LINK_ERROR                                     (2u)
#define DEF_CMD_FROM_MCU2_WITH_NO_DATA                               (1u)
#define DEF_LEN_PRIVATE_RADIO_DATA                                   (2u)

/* Index - Current Values = 28 bytes*/
#define DEF_CV_INDEX_CURRENT_DATE_AND_TIME                           (1u) /* 4 bytes      */
#define DEF_CV_INDEX_OPERATING_MODE                                  (5u)   /* 1 byte     */
#define DEF_CV_INDEX_TEMPERATURE                                     (6u)   /* 2 bytes      */
#define DEF_CV_INDEX_HUMIDITY                                        (8u)   /* 1 byte     */
#define DEF_CV_INDEX_BATTERY_VOLTAGE_A                               (9u)   /* 2 bytes      */
#define DEF_CV_INDEX_BATTERY_VOLTAGE_B                               (11u)   /* 2 bytes      */
#define DEF_CV_INDEX_FAULTS                                          (13u)   /* 4 bytes      */
#define DEF_CV_INDEX_STATES                                          (17u)   /* 2 bytes      */
#define DEF_CV_INDEX_CO_VALUE                                        (19u)   /* 2 bytes      */
#define DEF_CV_INDEX_HEAT_LEVEL                                      (21u)   /* 2 bytes      */
#define DEF_CV_INDEX_SMOKE_LEVEL                                     (23u)   /* 1 bytes      */
#define DEF_CV_INDEX_DEGRADED_CHAMBER_LEVEL                          (24u)   /* 1 byte     */
#define DEF_CV_INDEX_SOILING_LEVEL                                   (25u)   /* 1 byte     */
#define DEF_CV_INDEX_BRIGHTNESS                                      (26u)   /* 1 byte     */

/* Index - Logbook record */
#define DEF_LB_EVENT_COUNT                                           (1u)
#define DEF_LB_TIME_STAMP                                            (3u)
#define DEF_LB_OPERATING_STATE                                       (7u)
#define DEF_LB_EVENT_ID                                              (8u)
#define DEF_LB_DATA_0                                                (9u)
#define DEF_LB_DATA_1                                                (13u)
#define DEF_LB_CRC                                                   (17u)

/* Index - Time zone offset */
#define DEF_TZ_OFFSET                                                (1u)

/* Index - Counters & Dates */
/* Use very basic method to find exact location or if any issue with EEPROM */
#define DEF_CD_DAYS_SINCE_COMMISSIONING                             (1u)
#define DEF_CD_DATE_TIME_LAST_USER_BUTTON_PRESS                     (5u)
#define DEF_CD_RESET_COUNTERS_1                                     (9u)
#define DEF_CD_ALARM_COUNTERS_SMOKE                                 (33u)
#define DEF_CD_ALARM_COUNTERS_HEAT                                  (37u)
#define DEF_CD_ALARM_COUNTERS_CO                                    (35u)
#define DEF_CD_ERROR_COUNTERS_1                                     (39u)
#define DEF_CD_USER_EVENT_COUNTERS                                  (63u)

/* Index - FW Versions */
#define DEF_FW_NUMBER                                                (1u)
#define DEF_FW_REV_MAJOR                                             (3u)
#define DEF_FW_REV_MINOR                                             (4u)
#define DEF_FW_REV_BUILD                                             (5u)

/* Index - Alarm */
#define DEF_ALARM_STATE                                              (1u)

/* Index - Operating Mode */
#define DEF_OM_STATE                                                 (1u)

/* Index - BLOB */
#define DEF_BLOB_DATA                                                (1u)

/* Index - Radio Test */
#define DEF_RT_COMMAND                                               (1u)
#define DEF_RT_COMMAND_CH                                            (2u)
#define DEF_RT_COMMAND_POWER                                         (3u)

/* Index - OC Test */
#define DEF_OC_COMMAND_1                                              (1u)
#define DEF_OC_COMMAND_2                                            (2u)
#define DEF_OC_COMMAND_3                                            (3u)
#define DEF_OC_COMMAND_4                                            (4u)
#define DEF_OC_COMMAND_5                                            (5u)

/* Index - Assistance Light */
#define DEF_AS_LIGHT_DURATION                                        (1u)

/* Index - Airing Light */
#define DEF_AI_LIGHT_STATE                                           (1u)

/* Index - Airing Light Recommendation */
#define DEF_AI_LIGHT_RECOMMENDATION                                  (1u)

/* Index Obstacle & Coverage Detection */
#define DEF_OC_OBJECT_DETECTION_RESULT                               (1u)
#define DEF_OC_COVERAGE_DETECTION_RESULT_1                             (2u)
#define DEF_OC_COVERAGE_DETECTION_RESULT_2                     (3u)


/* Index Date Time */
#define DEF_DT_DATE_TIME                                             (1u)

/* Index Radio Duration */
#define DEF_RD_TX_RADIO_4                                            (1u)
#define DEF_RD_TX_TOM                                                (5u)
#define DEF_RD_RX_TOM                                                (9u)
#define DEF_RD_AIRING_LED                                         (13u)

/* Index RAM Faults */
#define DEF_RF_TS                                                    (2u)
#define DEF_RF_OM                                                    (5u)
#define DEF_RF_FAULT                                                 (6u)

/* Index Energy Mode */
#define DEF_EM_MODE                                                (1u)

/* Index Reset Copunter */
#define DEF_RC_SOURCE                          (1u)
#define DEF_RC_CTR                               (2u)

/* Standard Response - Possible Values */
#define DEF_SR_COMMAND_OK                                         (0x00u)
#define DEF_SR_COMMAND_NOT_FOUND                                  (0x01u)
#define DEF_SR_WRONG_LENGTH                                       (0x02u)
#define DEF_SR_PAYLOAD_EMPTY                                      (0x04u)

/* Link Layer Status - Possible Values */
#define DEF_LLS_STATUS_OK                                         (0x00u)
#define DEF_LLS_LENGTH_VIOLATION                                  (0x01u)
#define DEF_LLS_CRC_VIOLATION                                     (0x02u)
#define DEF_LLS_EF_VIOLATION                                      (0x04u)
#define DEF_LLS_BF_VIOLATION                                      (0x08u)
#define DEF_LLS_ERROR_MASK                                        (0x0Fu)

/* Operating Mode - Possible Values */
#define DEF_OM_STANDBY                                            (0x00u)
#define DEF_OM_COMMISIONING                                       (0x01u)
#define DEF_OM_OPERATING                                          (0x02u)
#define DEF_OM_TRANSPORT                                          (0x03u)
#define DEF_OM_FUNCTIONAL_TEST                                    (0x04u)

/* Assistance Light Duration - 0x00 = Off, 180s = On for 3 minutes max */
#define DEF_AS_LIGHT_TURN_OFF                                     (0x00u)
#define DEF_AS_LIGHT_DURATION_MAX                                  (180u)

/* Airing Light States */
#define DEF_AI_LIGHT_OFF                                       (0x00u)
#define DEF_AI_LIGHT_ON                                        (0x01u)
/* production BloB= Techem Secrate data, which include keys etc ..*/
#define DEF_PRODUCTION_BLOB_SIZE                                512u


#define OC_DETECTION 1u
#define OC_DETECTION_BIST 2u
#define OC_DETECTION_DISTANCE 3u
#define OC_SENSOR_CALIBRATION 4u
#define OC_SENSOR_READING  5u

#define LOGBOOK_RECORD_DATA 8u
#define MOUNTING_LOGBOOK_INDEX 8u
#define MOUNTING_LOGBOOK_LOCATIONS 20u
//SK Note: Maybe convert the #define to enum for ALARM

#define DEF_ALARM_STATE_CLEAR     0x00u
#define DEF_ALARM_STATE_SMOKE     0x01u
#define DEF_ALARM_STATE_HEAT      0x02u
#define DEF_ALARM_STATE_CO        0x03u
#define DEF_ALARM_STATE_TEST      0x04u
#define DEF_ALARM_STATE_SILENCE   0x05u

#define DEF_RADIO_PVT_DATA_ON 0x01u
#define DEF_RADIO_PVT_DATA_OFF 0x00u
#define RADIO_PRIVATE_DATA_LOCATION 4u
#define AIRING_FLAG_LOCATION  2u

#define FW_SIZE 2u
#define FW_LC_M 3u
#define FW_LC_MN 4u
#define FW_LC_B  5u
#define SPI_INVALID_VAL_TEMP  0x7fff
#define SPI_INVALID_VAL_16_BYTE  0xFFFFu
#define SPI_INVALID_VAL_8_BYTE  0xFFu
#define SPI_INVALID_VAL_32_BYTE  0xFFFFFFFFu
#define BYTE_4  4u
#define BYTE_2  2u
#define RESET_CTR 12u
#define DEF_RADIO_ERROR 1u
#define Minor_fault_CTR 12u
#define RADIO_LED_BUFF_CTR 16u
#define CTR_DATA 12u

#define EM0_MODE  0x01u
#define EM1_MODE  0x02u
#define EM2_MODE  0x03u

#define MAX_LASER_STRIKE_COUNT      (3u)

/********************************************************************************************************
 *********************************************************************************************************
 *                                             VARIABLES
 *********************************************************************************************************
 ********************************************************************************************************/
  static uint8_t Telegram_LinkLayerStatus; /**< Protocol Link Status */
  static uint8_t Telegram_StandardResponse;/**< Protocol STD response */
  static uint8_t Telegram_CurrentCommand;/**< Protocol Command */
  static uint8_t lasor_sensor_gain=0u;
  static uint8_t lasor_sensor_offset=0u;
  static uint16_t lasor_sensor_distance=0u;
  static RADIO_TEST test_value; /**< Variable for radio test */
  static bool obs_det_ftm_timePeriod = false;
  static bool obs_det_ftm_period_timeover = false;
  dl_fw_rev_data_t fw_data; /**< Variable for FW revision */
  Set_Parameter_for_OC OC_data;
  LASER_TEST laser_result;

  static bool obsFaultSet = false;
  static bool obsHwFaultSet = false;
  static bool covDetectionFlag = false;
  static uint8_t airingConfigStatus = 0U;
  static bool airingLightStatus = true;
  static bool laser_result_validated = false;
  static bool rem_alarm_sil = false;

#ifdef DEBUG_BUILD
  uint8_t laser_data_RX[DEF_LEN_LASER_RX_DATA] ={0};
#endif

 static uint8_t remote_alarm = REM_ALM_END;           /**< Variable for remote Alarm */

  /********************************************************************************************************
 *********************************************************************************************************
 *                                            PROTOTYPES
 *********************************************************************************************************
 ********************************************************************************************************/

static void Telegram_GetApplicationData (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_CheckPacketIntegrity (SPICOMMS_DATA_PACKET *p_packet);
static uint32_t Telegram_GetCommandLength (uint8_t command);
static void Telegram_GetStandardResponse (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetCurrentValues (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetLogbookRecord (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetDemountingLogbooks (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetCountersAndDates (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetFwVersion (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetAlarm (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetOperatingMode (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetProductionBLOB (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetTerminateProduction (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetRadioTest (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetTriggerOCDetection (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetLaser_RX_Data (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetDateAndTime (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetEnergyMode (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetRadioPrivateConfig(SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetToggleAiringRecommendation (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_GetAiringLight (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetTimezoneOffset (SPICOMMS_DATA_PACKET *p_packet);
static bool Telegram_SetAlarm (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetOperatingMode (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetDateTime (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetRadioDurationCounter (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetRamFaults (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetEnergyMode (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetResetCounter (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetAlarm_Forward (SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetRadioPrivateConfig(SPICOMMS_DATA_PACKET *p_packet);
static void Telegram_SetAlarm_Radio_ERROR (SPICOMMS_DATA_PACKET *p_packet);



/********************************************************************************************************
 *********************************************************************************************************
 *                                                FUNCTIONS
 *********************************************************************************************************

/****************************************************************************************************//**
 *                                     TELEGRAM_PrepareTransmitPacket()
 *
 * @brief This API will prepare the packet for SPI transmission
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
void
TELEGRAM_PrepareTransmitPacket (SPICOMMS_DATA_PACKET *p_packet)
{
  if (p_packet != NULL)
    {
      p_packet->TxPacket.BeginFrame = DEF_BEGIN_FRAME; /* start of frame          */
      p_packet->TxPacket.LinkLayerStatus = Telegram_LinkLayerStatus; /* link layer status         */
      Telegram_GetApplicationData (p_packet); /* get data and respective length   */
      p_packet->TxPacket.CRC = CRC_Calculate (
      p_packet->TxPacket.LinkLayerStatus, /* calculate CRC          */
      p_packet->TxPacket.Buffer, p_packet->TxPacket.Length);
      p_packet->TxPacket.EndFrame = DEF_END_FRAME; /* end of frame          */
    }
}

/****************************************************************************************************//**
 *                                      Telegram_GetApplicationData()
 *
 * @brief  This API will read  data from local modules and transmit it to MCU2
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetApplicationData (SPICOMMS_DATA_PACKET *p_packet)
{
  /**< get The command */
  uint8_t command = p_packet->TxPacket.Command;

  switch (command)
    {
    case DEF_STANDARD_RESPONSE: /**< STD response for SPI Command */
      Telegram_GetStandardResponse (p_packet);
      break;

    case DEF_CURRENT_VALUES:/**< Current Value: Every 2 Min TX all system status */
      Telegram_GetCurrentValues (p_packet);
      break;

    case DEF_LOGBOOK_RECORD:/**< on Event Logbook events */
      Telegram_GetLogbookRecord (p_packet);
      break;

    case DEF_DEMOUNTING_LOGBOOKS:/**< on  ADS Switch Operation:  */
      Telegram_GetDemountingLogbooks (p_packet);
      break;

    case DEF_COUNTERS_AND_DATES:/**< Every 24 Hours: Counters  */
      Telegram_GetCountersAndDates (p_packet);
      break;

    case DEF_FW_VERSION:/**< On Bootup: or Need to discuss about FW version  */
      Telegram_GetFwVersion (p_packet);
      break;

    case DEF_ALARM:/**< On Alarm Condition  */
      Telegram_GetAlarm (p_packet);
      break;

    case DEF_OPERATING_MODE:/**< On Operational mode change  */
      Telegram_GetOperatingMode (p_packet);
      break;

    case DEF_PRODUCTION_BLOB:
      Telegram_GetProductionBLOB (p_packet);/**< in Production this command will be use for MCU-2  */
      break;

    case DEF_TERMINATE_PRODUCTION:/**< in Production this command will be use for MCU-2  */
      Telegram_GetTerminateProduction (p_packet);
      break;

    case DEF_RADIO_TEST:/**< in Production this command will be use for MCU-2  */
      Telegram_GetRadioTest (p_packet);
      break;

    case DEF_TRIG_OC_DETECTION:/**< will be use to trigger obs detection */
      Telegram_GetTriggerOCDetection (p_packet);
      break;

    case DEF_DATE_AND_TIME:
      Telegram_GetDateAndTime (p_packet);
      break;

    case DEF_ENTER_EM:/**< MCU-1 Don't support this command but MCU-2 has implementation : Use in the Production */
      Telegram_GetEnergyMode (p_packet);
      break;
    case DEF_RADIO_CONFIG_SWITCH:/** DCR119 */
      Telegram_GetRadioPrivateConfig(p_packet);
      break;
    case DEF_TOGGLE_AIRING_RECOMMENDATION:/**<  Airing recommendation  feature enable or Disable*/
      Telegram_GetToggleAiringRecommendation (p_packet);
      break;

    case DEF_AIRING_LIGHT:/**< Airing recommendation */
      Telegram_GetAiringLight (p_packet);
      break;

#if 0
  case FUNCTIONAL_TEST:
      Telegram_FunctionalTest(p_packet);
     break;
#endif

    default:
      {

        break;
      }
    }
}

/****************************************************************************************************//**
 *                                      Telegram_GetStandardResponse()
 *
 * @brief This API will read the Standard response to transmit in response command received from MCU2
 * @param p_packet  packet received from MCU2
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetStandardResponse (SPICOMMS_DATA_PACKET *p_packet)
{
  if ((p_packet != NULL))
    {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = Telegram_StandardResponse; /* command  */
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND + 1u] = Telegram_CurrentCommand; /* data  */
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND + 2u] = Telegram_StandardResponse;
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND + 3u] = 0x00u; /* future use */
      p_packet->TxPacket.Length = DEF_LEN_STANDARD_RESPONSE; /* length  */
    }
}

/****************************************************************************************************//**
 *                                       Telegram_GetCurrentValues()
 *
 *@brief This API will read current values from other modules
 *@notes  Need to optimise the Function: Use simple way to implement so any bug in data can easily spot
 * @param p_packet  packet to transmit
* @return NULL
 ********************************************************************************************************/
static void
Telegram_GetCurrentValues (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t buff[4] =   { 0u };
  uint8_t filldata = 0u;
  Current_values read_data;
  if ((p_packet != NULL))
    {
      uint32_t fault_val = FaultHandler_GetFaultFlags ();
      /* This will Fill the Structure for Tx data By Calling different Modules */
      get_value_for_SPI (getBehavioural_System_Modes (false), fault_val,&read_data);
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_CURRENT_VALUES; /* command   */
      CommonUtils_Uint32ToUint8_swap(get_currentTime (), buff); /* data */
      /*Current Date & Time: Data Filling depend on protocol Index*/
      for (filldata=0u; filldata < BYTE_4; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_CURRENT_DATE_AND_TIME
              + filldata] = buff[filldata];
        }

       p_packet->TxPacket.Buffer[DEF_CV_INDEX_OPERATING_MODE] = getBehavioural_System_Modes(false);

       CommonUtils_Uint16ToUint8_swap(read_data.temp_val, buff); /* data   */
      /*Current Temperature: Data Filling depend on protocol Index*/
      for (filldata = 0u; filldata < BYTE_2; filldata++)
      {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_TEMPERATURE + filldata] =
              buff[filldata];
      }

      /*Current Humidity: Data Filling depend on protocol Index*/
      p_packet->TxPacket.Buffer[DEF_CV_INDEX_HUMIDITY] = read_data.Humidity_val;
      CommonUtils_Uint16ToUint8_swap (read_data.battA_val, buff); /* data  */
      /*Battery Voltage 1: Data Filling depend on protocol Index*/
      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_BATTERY_VOLTAGE_A + filldata] =
              buff[filldata];
        }
      CommonUtils_Uint16ToUint8_swap (read_data.battB_val, buff); /* data  */
      /*Battery Voltage 2: Data Filling depend on protocol Index*/
      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_BATTERY_VOLTAGE_B + filldata] =
              buff[filldata];
        }
      /*Fault Index: Data Filling depend on protocol Index*/
      CommonUtils_Uint32ToUint8_swap (fault_val, buff); /* data  */
      for (filldata = 0u; filldata < BYTE_4; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_FAULTS + filldata] =
              buff[filldata];
        }

      CommonUtils_Uint16ToUint8_swap (get_system_state_for_MCU2 (), buff); /* data  */
      /*Current State: Data Filling depend on protocol Index*/
      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_STATES + filldata] =
              buff[filldata];
        }

      CommonUtils_Uint16ToUint8_swap (read_data.Co_val, buff); /* data  */
           /*Current Co Value: Data Filling depend on protocol Index*/
      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_CO_VALUE + filldata] =
              buff[filldata];
        }
      /*Current Heat Value: Data Filling depend on protocol Index*/
      CommonUtils_Uint16ToUint8_swap (read_data.Heat_val, buff); /* data  */
      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_HEAT_LEVEL + filldata] =
              buff[filldata];
        }
      /*Current Smoke Value: Data Filling depend on protocol Index*/
      p_packet->TxPacket.Buffer[DEF_CV_INDEX_SMOKE_LEVEL] = read_data.Smoke_val;
      /*Degraded Smoke Chamber Value: Data Filling depend on protocol Index*/
      p_packet->TxPacket.Buffer[DEF_CV_INDEX_DEGRADED_CHAMBER_LEVEL] =
          read_data.Degraded_chamber_val;
      /*Soiling Value: Data Filling depend on protocol Index*/
      p_packet->TxPacket.Buffer[DEF_CV_INDEX_SOILING_LEVEL] = 0u;
      /*Brightness Value: Data Filling depend on protocol Index*/
      p_packet->TxPacket.Buffer[DEF_CV_INDEX_BRIGHTNESS] = read_data.brightness;
      p_packet->TxPacket.Length = DEF_LEN_CURRENT_VALUES; /* length   */

    }
}

/****************************************************************************************************//**
 *                                       Telegram_GetLogbookRecord()
 *
 * @brief This API will read logbook record  ( This shall be the latest logbook records )
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetLogbookRecord (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t buff[4] =
    { 0u };
  uint8_t filldata = 0u;
  uint16_t logbookIndex=0u;
  dl_event_logbook_record_t currentdata;

  if ((p_packet != NULL))
    {
      /* Get index of last entered event logbook record */
      logbookIndex = DataLogging_GetLastMainLogbookIndex( );

      DataLogging_GetEventLogbookRecord( logbookIndex, &currentdata );

      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_LOGBOOK_RECORD; /* command   */

//#define LOGBOOK_test
#ifdef LOGBOOK_test /* test With Hardcoded Values*/
      currentdata.Timestamp=0x313235;
      currentdata.OperatingMode=0x02;
      currentdata.ID=0x30;
      currentdata.Count=0x5553;
      currentdata.Data[0]=0x34;
      currentdata.Data[1]=0x34;
      currentdata.Data[2]=0x34;
      currentdata.Data[3]=0x34;
      currentdata.Data[4]=0x35;
 #endif

      /* Logbook Count */
      CommonUtils_Uint16ToUint8_swap (currentdata.Count, buff); /* data  */

      for (filldata = 0u; filldata < BYTE_2; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_LB_EVENT_COUNT + filldata] =
              buff[filldata];
        }
      /* TimeStamp  */
      CommonUtils_Uint32ToUint8_swap (currentdata.Timestamp, buff); /* data  */

      for (filldata = 0u; filldata < BYTE_4; filldata++)
        {
          p_packet->TxPacket.Buffer[DEF_LB_TIME_STAMP + filldata] =
              buff[filldata];
        }
      /* Operating Mode  */
      p_packet->TxPacket.Buffer[DEF_LB_OPERATING_STATE] =
          currentdata.OperatingMode;
      /* Current ID  */
      p_packet->TxPacket.Buffer[DEF_LB_EVENT_ID] = currentdata.ID;
      /* This will copy 8 Bytes*/
      for (uint8_t ctr = 0u; ctr < LOGBOOK_RECORD_DATA; ctr++)
        {
          (p_packet->TxPacket.Buffer[DEF_LB_DATA_0 + ctr]) =
              currentdata.Data[ctr];
        }
      p_packet->TxPacket.Length = DEF_LEN_LOGBOOK_RECORD; /* length */
    }
}

/****************************************************************************************************//**
 *                                     Telegram_GetDemountingLogbooks()
 *
 * @brief This API will read the de mounting logbooks
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetDemountingLogbooks ( SPICOMMS_DATA_PACKET * p_packet )
{
  uint8_t buf[ sizeof( dl_demounting_logbook_record_t ) ] = { 0u }; 

  dl_demounting_logbook_record_t record = { 0 };

  uint8_t packet_location = 0u;

  if( ( p_packet != NULL ) )
  {
    const uint16_t newest_index = DataLogging_GetLastDemountingLogbookIndex( );

    p_packet->TxPacket.Buffer[ DEF_INDEX_COMMAND ] = DEF_DEMOUNTING_LOGBOOKS;

    /* get The 20 records every time when we receive request*/
    for( int i = 0, j = newest_index; i < MOUNTING_LOGBOOK_LOCATIONS; i++ )
    {
      DataLogging_GetDemountingLogbookRecord( j, &record );

      CommonUtils_Uint16ToUint8_swap( record.Count, buf );

      CommonUtils_Uint32ToUint8_swap( record.Timestamp, &buf[ 2 ] );

      buf[ 6 ] = record.OperatingMode;

      buf[ 7 ] = record.ID;

      const int k = 1 + ( i * sizeof( dl_demounting_logbook_record_t ) );

      memcpy( &p_packet->TxPacket.Buffer[ k ], buf, sizeof( dl_demounting_logbook_record_t ) );

      if( j == 0 )
      {
        j = MOUNTING_LOGBOOK_LOCATIONS - 1;
      }
      else
      {
        j--;
      }
    }

    p_packet->TxPacket.Length = DEF_LEN_DEMOUNTING_LOGBOOKS;
  }

}

/****************************************************************************************************//**
 *                                         Telegram_GetCountersAndDates()
 *
 * @brief This API will read the  counters,dates
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetCountersAndDates (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t buf[ 4 ] = { 0u };

  if( p_packet != NULL )
  {
    /* Command */
    p_packet->TxPacket.Buffer[ DEF_INDEX_COMMAND ] = DEF_COUNTERS_AND_DATES;

    /* Date of Commissioning */
    const uint32_t first_activation = get_commissioning_time();//DataLogging_GetFirstActivation( );   // 0x12345678

    CommonUtils_Uint32ToUint8_swap( first_activation, buf );

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_DAYS_SINCE_COMMISSIONING ], buf, sizeof( first_activation ) );

    /* Latest use Bist Time stamp */
    const uint32_t user_bist_time = get_userBistTest_time(); //DataLogging_GetLatestUserBistTimestamp( );    // 0x87654321

    CommonUtils_Uint32ToUint8_swap( user_bist_time, buf );

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_DATE_TIME_LAST_USER_BUTTON_PRESS ], buf, sizeof( user_bist_time ) );

    /* Total reset  Counters: 12 reason    */
    for( uint16_t i = 0, j = 0; i < RESET_CTR; i++, j += sizeof( uint16_t ) )
    {
      const uint16_t reset_reason_count = DataLogging_GetResetReasonCount( i );   // i

      CommonUtils_Uint16ToUint8_swap( reset_reason_count, buf );

      p_packet->TxPacket.Buffer[ j + DEF_CD_RESET_COUNTERS_1     ] = buf[ 0 ];

      p_packet->TxPacket.Buffer[ j + DEF_CD_RESET_COUNTERS_1 + 1 ] = buf[ 1 ];
    }

    /* Smoke */
    const uint16_t count_smoke  = DataLogging_GetSmokeLocalEventCount( );   // 0x1111

    CommonUtils_Uint16ToUint8_swap( count_smoke, buf );

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_ALARM_COUNTERS_SMOKE ], buf, sizeof( count_smoke ) );

    /* CO */
    const uint16_t count_Co     = DataLogging_GetCOLocalEventCount( );    // 0x2222

    CommonUtils_Uint16ToUint8_swap( count_Co, buf );

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_ALARM_COUNTERS_CO ], buf, sizeof( count_Co ) );

    /* Heat */
    const uint16_t count_heat   = DataLogging_GetHeatLocalEventCount( );    // 0x3333

    CommonUtils_Uint16ToUint8_swap( count_heat, buf );

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_ALARM_COUNTERS_HEAT ], buf, sizeof( count_heat ) );

    /* Error counters */
    for( uint16_t i = 0, j = 0; i < Minor_fault_CTR; i++, j += sizeof( uint16_t ) )
    {
      const uint16_t minor_fault_count = DataLogging_GetMinorFaultCount( i );    // i

      CommonUtils_Uint16ToUint8_swap( minor_fault_count, buf );

      p_packet->TxPacket.Buffer[ j + DEF_CD_ERROR_COUNTERS_1     ] = buf[ 0 ];

      p_packet->TxPacket.Buffer[ j + DEF_CD_ERROR_COUNTERS_1 + 1 ] = buf[ 1 ];
    }

    /* Total test count */
    uint16_t bistcnt    = DataLogging_GetUserTestCount( );
    uint16_t extbistcnt = DataLogging_GetUserExtTestCount( );



    const uint16_t total_ct   = bistcnt + extbistcnt;     // 0x1234

    CommonUtils_Uint16ToUint8_swap( total_ct, buf) ;

    memcpy( &p_packet->TxPacket.Buffer[ DEF_CD_USER_EVENT_COUNTERS ], buf, sizeof( total_ct ) );

    /* Packet length */
    p_packet->TxPacket.Length = DEF_LEN_COUNTERS_AND_DATES;
  }
}

/****************************************************************************************************//**
 *                                          Telegram_GetFwHwVersions()
 *
 * @brief This API will read FW and HW versions
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetFwVersion (SPICOMMS_DATA_PACKET *p_packet)
{
  //uint8_t buff[2] = { 0u };
  if (p_packet != NULL)
    {
  p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_FW_VERSION; /* command   */
  p_packet->TxPacket.Buffer[DEF_FW_NUMBER] =    (uint8_t)FW_SA_NUM_2;
  p_packet->TxPacket.Buffer[DEF_FW_NUMBER+1u] = (uint8_t)FW_SA_NUM_1;
  p_packet->TxPacket.Buffer[DEF_FW_REV_MAJOR] = (uint8_t)FW_MAJOR_REV; /* data   */
  p_packet->TxPacket.Buffer[DEF_FW_REV_MINOR] = (uint8_t)FW_MINOR_REV;
  p_packet->TxPacket.Buffer[DEF_FW_REV_BUILD]=  (uint8_t)FW_BUILD_REV;
  p_packet->TxPacket.Length = DEF_LEN_FW_VERSION; /* length   */
    }
}

/****************************************************************************************************//**
 *                                            Telegram_GetAlarm()
 *
 * @brief This API will read  Alarm status
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetAlarm (SPICOMMS_DATA_PACKET *p_packet)
{
  behaviour_state_enum_operational_States data;
  p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_ALARM; /* command  */

  if (p_packet != NULL)
    {
   //#define FW_ALARM
#ifdef FW_ALARM
      data=state_CO_Alarm_Silence;
#else
      data = getBehavioural_Operational_State ();
#endif
      if (data == state_Smoke_Alarm)
        {
          p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_SMOKE; /* data   */
        }
      else if (data == state_Heat_Alarm)
        {
          p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_HEAT; /* data   */
        }
      else if (data == state_CO_Alarm)
        {
          p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_CO; /* data   */
        }
      else if ((data == state_CO_Alarm_Silence)
          || (data == state_Smoke_Alarm_Silence)
          || (data == state_Heat_Alarm_Silence))
        {
          p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_SILENCE; /* data   */
        }
      else
        {
          if(get_rem_alarm_silence_to_muc2() == true)
          {
              set_rem_alarm_silence_to_mcu2(false);
              p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_SILENCE; /* data   */
          }
          else
          {
              p_packet->TxPacket.Buffer[DEF_ALARM_STATE] = DEF_ALARM_STATE_CLEAR; /* data   */
          }
        }
      p_packet->TxPacket.Length = DEF_LEN_ALARM; /* length  */
    }
}

/****************************************************************************************************//**
 *                                         Telegram_GetOperatingMode()
 *
 * @brief This API will read the operating mode to send to MCU2
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetOperatingMode (SPICOMMS_DATA_PACKET *p_packet)
{
  behaviour_state_enum_System_modes mode_sys;
  /* techem has different interpretation for mode ( Transport= 3, 4= Functional test mode)*/
  mode_sys= getBehavioural_System_Modes (false);

  if (p_packet != NULL)
  {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_OPERATING_MODE; /* command   */
      p_packet->TxPacket.Buffer[DEF_OM_STATE] = (uint8_t) mode_sys;
      p_packet->TxPacket.Length = DEF_LEN_OPERATING_MODE; /* length   */
   }
}

/****************************************************************************************************//**
 *                                         Telegram_GetDateAndTime()
 *
 * @brief This API will read the System Time
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetDateAndTime (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t buff[4]={0u};
  uint32_t time_val = 0u;
  time_val = get_currentTime();

  if (p_packet != NULL)
  {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_DATE_AND_TIME; /* command   */
      CommonUtils_Uint32ToUint8_swap (time_val, buff);                  /* data   */
      for (uint8_t filldata = 0u; filldata < BYTE_4; filldata++)
      {
          p_packet->TxPacket.Buffer[DEF_CV_INDEX_CURRENT_DATE_AND_TIME + filldata] = buff[filldata];
      }
      p_packet->TxPacket.Length = DEF_LEN_DATE_AND_TIME; /* length   */
  }
}
/****************************************************************************************************//**
 *                                        Telegram_GetProductionBLOB()
 *
 * @brief This API will read  production blob to send to MCU2
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetProductionBLOB (SPICOMMS_DATA_PACKET *p_packet)
{
#ifdef PRODUCTION
  uint8_t production_data[DEF_PRODUCTION_BLOB_SIZE] =
    { 0 };
#endif
  if (p_packet != NULL)
    {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_PRODUCTION_BLOB; /* command  */
#ifdef PRODUCTION
      /* This functionality Can Not be tested in development*/
      (void) memcpy (&p_packet->TxPacket.Buffer[DEF_BLOB_DATA],
                     &production_data, DEF_PRODUCTION_BLOB_SIZE); /* data   */
#endif
      p_packet->TxPacket.Length = DEF_LEN_PRODUCTION_BLOB; /* length  */
    }
}

/****************************************************************************************************//**
 *                                      Telegram_GetTerminateProduction()
 *
 * @brief This API will t terminate production
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetTerminateProduction (SPICOMMS_DATA_PACKET *p_packet)
{
  if (p_packet != NULL)
  {
      /* This functionality Can Not be tested in development*/
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_TERMINATE_PRODUCTION; /* command   */
      p_packet->TxPacket.Length = DEF_LEN_TERMINATE_PRODUCTION; /* length   */

   }
}

/****************************************************************************************************//**
 *                                          Telegram_GetRadioTest()
 *
* @brief This API will read radio test
* @param p_packet  packet to transmit
* @return NULL
 ********************************************************************************************************/
void
Telegram_GetRadioTest (SPICOMMS_DATA_PACKET *p_packet)
{
  RADIO_TEST value;
  if (p_packet != NULL)
    {
      value = get_radio_test ();
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_RADIO_TEST; /* command   */
      p_packet->TxPacket.Buffer[DEF_RT_COMMAND] = value; /* data  */
      p_packet->TxPacket.Buffer[DEF_RT_COMMAND_CH] = 0x00u; /* Channel Not Use/Applicable  */
      p_packet->TxPacket.Buffer[DEF_RT_COMMAND_POWER] = 0x00u; /* Power Not use/Applicable  */
      p_packet->TxPacket.Length = DEF_LEN_RADIO_TEST; /* length   */
    }
}

/****************************************************************************************************//**
 *                                    Telegram_GetTriggerOCDetection()
 *
 * @brief This Function will Trigger Obstacle and Coverage Detection
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetTriggerOCDetection (SPICOMMS_DATA_PACKET *p_packet)
{
  Set_Parameter_for_OC OC_data_MCU2;
  if (p_packet != NULL)
    {
       p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_TRIG_OC_DETECTION; /* command  */
      /* New Data As Per techem requirements */
      get_OC_Parameter (&OC_data_MCU2);
      if (OC_data_MCU2.val !=0u)
        {
          p_packet->TxPacket.Buffer[DEF_OC_COMMAND_1] = (uint8_t) OC_data_MCU2.val; /* Command  */
          p_packet->TxPacket.Buffer[DEF_OC_COMMAND_2] = OC_data_MCU2.parameter_1; /* Para 1 */
          p_packet->TxPacket.Buffer[DEF_OC_COMMAND_3] = OC_data_MCU2.parameter_2; /* Para 2  */
          p_packet->TxPacket.Buffer[DEF_OC_COMMAND_4] = OC_data_MCU2.parameter_3; /* Para 3  */
          p_packet->TxPacket.Buffer[DEF_OC_COMMAND_5] = OC_data_MCU2.parameter_4; /* Para 4 */
          p_packet->TxPacket.Length = DEF_LEN_TRIG_OC_DETECTION_BIST; /* length  */
        }
  }
}

/****************************************************************************************************//**
 *                                      Telegram_GetLaserCalibration()
 * @brief This function will  get laser calibration data
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetLaser_RX_Data (SPICOMMS_DATA_PACKET *p_packet)
{
  /* To Do: no data format agreed yet *//* data   */
#ifdef DEBUG_BUILD
  for (uint16_t laser_data=1u;laser_data<DEF_LEN_LASER_RX_DATA;laser_data++)
  {
    laser_data_RX[laser_data] = p_packet->RxPacket.Buffer[laser_data];
  }
  set_obs_det_ftm_timeover(true);

#endif



}

/****************************************************************************************************//**
 *                                        Telegram_GetDEnergyMode()
 *
 * @brief This function will read energy mode to send info to MCU2
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetEnergyMode (SPICOMMS_DATA_PACKET *p_packet)
{
  if (p_packet != NULL)
    {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_ENTER_EM; /* command   */
      p_packet->TxPacket.Buffer[DEF_EM_MODE] = 0x01u; /* data   */
      p_packet->TxPacket.Length = DEF_LEN_ENERGY_MODE; /* length  */
    }
}


static void
Telegram_GetRadioPrivateConfig(SPICOMMS_DATA_PACKET *p_packet)
{
  uint32_t radio_data = 0u;
  radio_data = DataLogging_GetFeatureconfigurationFlags();
  if (p_packet != NULL)
  {
     p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_RADIO_CONFIG_SWITCH; /* command  */

     if (((radio_data & (1u << RADIO_PRIVATE_DATA_LOCATION))) != 0u)
     {
         p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] = DEF_RADIO_PVT_DATA_ON; /* Radio Pvt data on   */
     }
     else
     {
         p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] = DEF_RADIO_PVT_DATA_OFF;/* Radio Pvt data off   */
     }

     p_packet->TxPacket.Length = DEF_LEN_TOGGLE_AIRING_RECOMMENDATION; /* length   */
  }
}


/****************************************************************************************************//**
 *                                 Telegram_GetToggleAiringRecommendation()
 *
 * @brief This function will read airing light recommendation data
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetToggleAiringRecommendation (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t airing_data = get_AiringConfig();

  if (p_packet != NULL)
  {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_TOGGLE_AIRING_RECOMMENDATION; /* command  */

      if (airing_data == (uint8_t)AIRING_FLAG_ON)
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] =  AIRING_FLAG_ON; /* Airing recommendation set   */
      }
      else if(airing_data == (uint8_t)AIRING_FLAG_OFF)
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] = AIRING_FLAG_OFF;/* Airing recommendation clear   */
      }
	  else if(airing_data == (uint8_t)RADIOPRIV_DATA_CONFIGMODE)
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] = RADIOPRIV_DATA_CONFIGMODE;/* Radio Private data config mode */
      }
      else
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_RECOMMENDATION] = AIRING_FLAG_TOGGLE;/* Airing recommendation toggle   */
      }

      p_packet->TxPacket.Length = DEF_LEN_TOGGLE_AIRING_RECOMMENDATION; /* length   */
    }
}

/****************************************************************************************************//**
 *                                 Telegram_SetRadioPrivateConfig()
 *
 * @brief This API will Set private radio data config
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_SetRadioPrivateConfig(SPICOMMS_DATA_PACKET *p_packet)
{
  uint32_t radio_data = 0u;
  uint8_t radio_config = 0u;
  DEBUG_TELEGRAM("\n privateDataSwitch SPI", false, 0u);
  radio_data = DataLogging_GetFeatureconfigurationFlags ();
  if (p_packet != NULL)
    {
        radio_config = p_packet->RxPacket.Buffer[1u];

        if ((radio_config == 1u)) /*set */
        {
            DEBUG_TELEGRAM("\n privateDataSwitch ON", false, 0u);
            radio_data = (radio_data | (1u << RADIO_PRIVATE_DATA_LOCATION)); /* on */
        }
        else
        {
            DEBUG_TELEGRAM("\n privateDataSwitch OFF", false, 0u);
            radio_data = (radio_data & (~(1u << RADIO_PRIVATE_DATA_LOCATION))); /* off   */
        }

        DataLogging_SetFeatureConfigurationFlags (radio_data);
     }
}

/****************************************************************************************************//**
 *                                        Telegram_GetAiringLight()
 *
 * @brief This API will read the  airing light data
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_GetAiringLight (SPICOMMS_DATA_PACKET *p_packet)
{

  if (p_packet != NULL)
  {
      p_packet->TxPacket.Buffer[DEF_INDEX_COMMAND] = DEF_AIRING_LIGHT; /* command   */

      if (airingLightStatus == true)
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_STATE] = DEF_AI_LIGHT_ON; /* data  */
          airingLightStatus = false;
      }
      else
      {
          p_packet->TxPacket.Buffer[DEF_AI_LIGHT_STATE] = DEF_AI_LIGHT_OFF;
          airingLightStatus = true;
      }

      p_packet->TxPacket.Length = DEF_LEN_AIRING_LIGHT; /* length   */
  }
}

/****************************************************************************************************//**
 *                                        Telegram_SetTimezoneOffset()
 *
 * @brief This API will Set timezone offset received from MCU2
 * @param p_packet  packet to transmit
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_SetTimezoneOffset (SPICOMMS_DATA_PACKET *p_packet)
{

  (void) &p_packet;
#ifdef FUTURE_USE
  uint32_t offset;

  offset = *(uint32_t*)&(p_packet->RxPacket.Buffer[DEF_TZ_OFFSET]); /* data   */
#endif
  /* to do: call function to update time zone */
}

/****************************************************************************************************//**
 *                                          Telegram_SetFwVersion()
 *
 * @brief This API will read  FW and HW versions & set
 * @param p_packet  packet to transmit
 * @return null
 ********************************************************************************************************/
static void
Telegram_SetFwVersion (SPICOMMS_DATA_PACKET *p_packet)
{

  dl_fw_rev_data_t pstr_fw_rev;
  uint8_t buff[FW_SIZE] =
    { 0 };
  if (p_packet != NULL)
    {
      buff[0] = p_packet->RxPacket.Buffer[DEF_FW_NUMBER];
      buff[1] = p_packet->RxPacket.Buffer[DEF_FW_NUMBER + 1u];
      pstr_fw_rev.FwNumber = CommonUtils_Uint8ToUint16 (buff);
      pstr_fw_rev.FwRevMajor = p_packet->RxPacket.Buffer[FW_LC_M];
      pstr_fw_rev.FwRevMinor = p_packet->RxPacket.Buffer[FW_LC_MN];
      pstr_fw_rev.FwRevBuild = p_packet->RxPacket.Buffer[FW_LC_B];
      DataLogging_SetFw (&pstr_fw_rev);
      const bool eeprom_ok = data_logging_is_eeprom_ok( );
      if( eeprom_ok )
      {
           DataLogging_SetCRC( );
      }
    }
}

/****************************************************************************************************//**
 *                                          Telegram_SetAlarm()
 *
 * @brief   This API will   read alarm data received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return NULL
 ********************************************************************************************************/
static bool
Telegram_SetAlarm( SPICOMMS_DATA_PACKET * p_packet )
{
  bool ok = false;

  if( p_packet != NULL )
  {
		ok = set_remote_alarm_status_MCU_2(p_packet->RxPacket.Buffer[ DEF_ALARM_STATE ]);
  }

  return( ok );
}
/****************************************************************************************************//**
 *                                          trigger_remote_alarm()
 *
 * @brief   This API will   Set alarm data received from MCU2
 * We have to use global flag remote alarm because As soon As we trigger Alarm AFE took control of SPI BUS
 * and  device unable to send the MCU-2 response .
 * @param p_packet  pointer to packet containing the received data
 * @return NULL
 ********************************************************************************************************/

void trigger_remote_alarm( void )
{
//  if(getBehavioural_Operational_State() != state_Remote_Alarm)
//  {
      RTOS_ERR err;
      OSFlagPost( &Event_Flags_SubGroup[ 0 ], ( uint32_t )EVENT_REMOTE_ALARM_0, OS_OPT_POST_FLAG_SET, &err );
      APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
//}
}

/****************************************************************************************************//**
 *                                       Telegram_SetOperatingMode()
 *
 * @brief This API will Set operating mode received from MCU2
 * @param p_packet  pointer to packet containing the received data
 *@return null
 ********************************************************************************************************/
static void
Telegram_SetOperatingMode (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t operating_mode;
  if (p_packet != NULL)
    {
      operating_mode = p_packet->RxPacket.Buffer[DEF_OM_STATE]; /* data     */
      setBehavioural_System_Modes ( (behaviour_state_enum_System_modes) operating_mode);
      setBehavioural_Operational_State (state_Idle); /* This will reset the System State to default*/
      if((operating_mode == Transport_Mode) || (operating_mode == Standby_Mode))
      {
          (void)BURTCTimer_Stop(TMR_Demount_too_long_event_0);
          if((FaultHandler_GetFaultFlags() & DEF_DEM_TOO_LONG_FAULT) != 0U)
          {
              FaultHandler_FaultClear(DemountedTooLongFault);
              DataLogging_SetMinorFault(FaultDemountedTooLong, false);
          }
      }

    }
}

/****************************************************************************************************//**
 *                                      Telegram_SetResultObsCovDet()
 *
 * @brief This API will  Set coverage and obstacle detection result received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_SetResultObsCovDet (SPICOMMS_DATA_PACKET *p_packet)
{
    uint8_t cov_det_result_1 = 0u;
    uint8_t cov_det_result_2 = 0u;
    uint8_t obs_cov_BIST_result = 0u;
    static uint8_t laserHwStrikeCount = 0u;

    if (p_packet != NULL)
    {
      obs_cov_BIST_result = p_packet->RxPacket.Buffer[DEF_OC_OBJECT_DETECTION_RESULT]; /* data     */
      cov_det_result_1 = p_packet->RxPacket.Buffer[DEF_OC_COVERAGE_DETECTION_RESULT_1];
      cov_det_result_2 =   p_packet->RxPacket.Buffer[DEF_OC_COVERAGE_DETECTION_RESULT_2];

      //@notes  Below code handling may change in future and need better handling
      //cov_det_result_1 =00 Always except error
      /* If Obstacle detection BIST result */
      if ((obs_cov_BIST_result==LASER_DITECTION) ||(obs_cov_BIST_result==LASER_BIST))
      {
          /* Below Conditions are proposed by Techem and may be change in future */
          if ((cov_det_result_1 == 0u) && (cov_det_result_2 == 0u))
          {
              DEBUG_TELEGRAM("\nNO DETECTION", false, 0u);
              laser_result = LASER_NO_DETETCION;
              if(covDetectionFlag == true)
              {
                  FaultHandler_FaultClear(CoverageDetectedFault);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_COVERAGE_DET_END, NULL );
                  covDetectionFlag = false;
                  DataLogging_SetMinorFault(FaultCoverageDetected, covDetectionFlag);
              }

              if(obsFaultSet == true)  //reset
              {
                  FaultHandler_FaultClear(ObstacleDetectedFault);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_END, NULL );
                  obsFaultSet = false;
                  DataLogging_SetMinorFault(FaultObstacleDetected, obsFaultSet);
              }

          }
          if ((cov_det_result_1 == 0u) && (cov_det_result_2 == 1u))
          {
              DEBUG_TELEGRAM("\nLASER_OBSTACLE_DETETCION", false, 0u);
              laser_result = LASER_OBSTACLE_DETETCION;
              if(obsFaultSet == false)
              {
                  FaultHandler_FaultSet(ObstacleDetectedFault);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_START, NULL );
                  obsFaultSet = true;
                  DataLogging_SetMinorFault(FaultObstacleDetected, obsFaultSet);
              }

          }
          if ((cov_det_result_1 == 0u) && (cov_det_result_2 == 2u))
          {
              DEBUG_TELEGRAM("\nLASER_COVERAGE_DETETCION", false, 0u);
              laser_result = LASER_COVERAGE_DETETCION;
              if(covDetectionFlag == false)
              {
                  FaultHandler_FaultSet(CoverageDetectedFault);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_COVERAGE_DET_START, NULL );
                  covDetectionFlag = true;
                  DataLogging_SetMinorFault(FaultCoverageDetected, covDetectionFlag);
              }
          }

          if ((cov_det_result_1 == 0u) && (cov_det_result_2 == 0xffu))
          {
              DEBUG_TELEGRAM("\nLASER_SENSOR_FAILUARE", false, 0u);
              laser_result = LASER_SENSOR_FAILUARE;
          }

          if(getBehavioural_System_Modes(false) != Commisioning_Mode)
          {
              if(laser_result == LASER_SENSOR_FAILUARE)
              {
                  if(obsHwFaultSet == false)
                  {
                      laserHwStrikeCount++;
                      if(laserHwStrikeCount >= MAX_LASER_STRIKE_COUNT)
                      {
                          FaultHandler_FaultSet(ObstacleDetectionHwFault);
                          DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_HW_ERR_START, NULL );
                          obsHwFaultSet = true;
                      }
                  }
              }
              else
              {
                  laserHwStrikeCount = 0u;
                  if(obsHwFaultSet == true)
                  {
                      FaultHandler_FaultClear(ObstacleDetectionHwFault);
                      DataLogging_SetEventLogbookRecord( DEF_LBE_OBSTACLE_DET_HW_ERR_END, NULL );
                      obsHwFaultSet = false;
                  }
              }
          }

      }
      /* This will read the Distance*/
      if (obs_cov_BIST_result==LASER_DISTANCE)
      {
          lasor_sensor_distance = ((cov_det_result_2 << 8u ) | cov_det_result_1);
          DEBUG_TELEGRAM("\nLaser distance", true, lasor_sensor_distance);
      }
      /* This will read the Gain & offset : Not Sure What MCU-1 will do with this*/
      if (obs_cov_BIST_result==LASER_GAIN_OFFSET)
      {
          lasor_sensor_gain= cov_det_result_1;
          lasor_sensor_offset=cov_det_result_2;
      }

      set_laser_status_validated(true);

      if(get_obs_det_ftm_timeover() == false)
      {
          set_obs_det_ftm_timeover(true);
      }

      if(get_obs_det_ftm_period_timeover() == false)
      {
          set_obs_det_ftm_period_timeover(true);
      }
   }

}



/****************************************************************************************************//**
 *                                        Get the laser Status()
 *
 * @brief   This function will return laser status
 * @param None
 * @return different result as per the LASER_TEST enum
 ********************************************************************************************************/

LASER_TEST get_laser_status(void)
{
     return laser_result;
}

/****************************************************************************************************//**
 *                                        Get the distance()
 *
 * @brief   This function will return distance
 * @param None
 * @return None
 ********************************************************************************************************/

uint16_t get_laser_distance(void)
{
     return lasor_sensor_distance;
}

/****************************************************************************************************//**
 *                                        Set the distance()
 *
 * @brief   This function will return distance
 * @param None
 * @return None
 ********************************************************************************************************/
void set_laser_distance(uint16_t distance)
{
  lasor_sensor_distance = distance;
}

/****************************************************************************************************//**
 *                                        Telegram_SetDateTime()
 *
 * @brief This API will Set date and time received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return None
 ********************************************************************************************************/
static void
Telegram_SetDateTime (SPICOMMS_DATA_PACKET *p_packet)
{
  uint32_t date_time = 0U;
  uint8_t buff[4] = { 0 };
  if (p_packet != NULL)
  {
      buff[0] = p_packet->RxPacket.Buffer[DEF_DT_DATE_TIME + 0u]; /* data     */
      buff[1] = p_packet->RxPacket.Buffer[DEF_DT_DATE_TIME + 1u]; /* data     */
      buff[2] = p_packet->RxPacket.Buffer[DEF_DT_DATE_TIME + 2u]; /* data     */
      buff[3] = p_packet->RxPacket.Buffer[DEF_DT_DATE_TIME + 3u]; /* data     */
      date_time = CommonUtils_Uint8ToUint32 (buff);
      /* Data Integrity need to check */
      time_setCurrentTime (date_time);
  }
}

/****************************************************************************************************//**
 *                                   Telegram_SetRadioDurationCounter()
 *
 * @brief     Set radio duration counter data received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return null
 ********************************************************************************************************/
static void
Telegram_SetRadioDurationCounter (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t buff[4u] =
    { 0u };
  uint8_t filldata = 0u;

  uint8_t counter[RADIO_LED_BUFF_CTR] =
    { 0u };
  uint32_t total_airing_duration = 0u;
  if (p_packet != NULL)
    {
      for (uint8_t ctr = 0u; ctr < CTR_DATA; ctr++)
        {
          counter[ctr] = p_packet->RxPacket.Buffer[DEF_RD_TX_RADIO_4 + ctr];
        }
     for (; filldata < BYTE_4; filldata++)
    {
      buff[filldata]=p_packet->RxPacket.Buffer[DEF_RD_AIRING_LED + filldata] ;
    }
     /* Total Airing Duration */
     total_airing_duration = CommonUtils_Uint8ToUint32 (buff);
     /* Total Radio Duration logged */
     DataLogging_SetRadioDurationCounter (counter);
     /* Total Airing Duration logged */
     DataLogging_SetAirRecTotalDuration (total_airing_duration);
    }
}

/****************************************************************************************************//**
 *                                        Telegram_SetRamFaults()
 *
 * @brief This API will  Set RAM faults received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return null
 ********************************************************************************************************/
static void
Telegram_SetRamFaults (SPICOMMS_DATA_PACKET *p_packet)
{
  (void) (&p_packet); //unused
    FaultHandler_FaultSet (MCU2RAMFault);
}

/****************************************************************************************************//**
 *                                        Telegram_SetAlarm_Forward_ERROR()
 *
 * @brief This API will Set Radio Link Fault
 * @param p_packet  pointer to packet containing the received data
 * @return NULL
 ********************************************************************************************************/
static void
Telegram_SetAlarm_Radio_ERROR (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t error_data=0u;
  if (p_packet != NULL)
  {
      error_data=p_packet->RxPacket.Buffer[DEF_RADIO_ERROR];
      if (error_data==DEF_RADIO_ERROR)
      {
          FaultHandler_FaultSet (RadioFault); /* Minor fault set */
      }
      else
      {
          FaultHandler_FaultClear (RadioFault); /* Minor fault to clear */
      }
  }
}

/****************************************************************************************************//**
 *                                        Telegram_SetAlarm_Forward()
 *
 * @brief     Set Alarm forwarding feature
 * @param p_packet  pointer to packet containing the received data
 * @return null
 ********************************************************************************************************/
static void
Telegram_SetAlarm_Forward (SPICOMMS_DATA_PACKET *p_packet)
{
  uint32_t set_alarm_forwarding;
  uint32_t get_alarm_forwarding;


  get_alarm_forwarding = DataLogging_GetFeatureconfigurationFlags ();
  if (p_packet != NULL)
    {
      set_alarm_forwarding = p_packet->RxPacket.Buffer[1u]; /* data      */
      if (set_alarm_forwarding == 1u)
        {
          get_alarm_forwarding = (get_alarm_forwarding | (1u << 3u)); //Set The flag
        }
      else
        {
          get_alarm_forwarding = (get_alarm_forwarding & ~(1u << 3u)); //Clear The flag
        }
      /*Set The New Configuration*/
      DataLogging_SetFeatureConfigurationFlags (get_alarm_forwarding);

    }
}

/****************************************************************************************************//**
 *                                  Not Supported       Telegram_SetEnergyMode()
 *
 * @brief  This API will Set energy mode received from MCU2
 * @param p_packet  pointer to packet containing the received data
 * @return null
 ********************************************************************************************************/
static void
Telegram_SetEnergyMode( SPICOMMS_DATA_PACKET * p_packet )
{
  const uint8_t mode = p_packet->RxPacket.Buffer[ 1u ];

  switch( mode )
  {
    case EM0_MODE:
    {
      pc_enter_em0_and_stop( );
      break;
    }
    case EM1_MODE:
    {
      pc_enter_em1_and_stop( );
      break;
    }
    case EM2_MODE:
    {
      pc_enter_em2_and_stop( );
      break;
    }
    default:
    {
      break;
    }
  }
}

/****************************************************************************************************//**
 *                                        Telegram_SetResetCounter()
 *
 * @brief This API will Set reset counter received from MCU2
  * @param p_packet  pointer to packet containing the received data
  * @return NULL
 ********************************************************************************************************/
static void
Telegram_SetResetCounter (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t reset_source = 0u;
  uint8_t reset_counter = 0u;
  uint8_t logbook_data_reset[2] =
    { 0u };
  if (p_packet != NULL)
    {
      reset_source = p_packet->RxPacket.Buffer[DEF_RC_SOURCE];
      reset_counter = p_packet->RxPacket.Buffer[DEF_RC_CTR];
      /* Reset Source ,reset Counters */
      logbook_data_reset[0] = reset_source;
      logbook_data_reset[1] = reset_counter;
      DataLogging_SetEventLogbookRecord (DEF_LBE_RESET, logbook_data_reset);
    }
}

/****************************************************************************************************//**
 *                                    TELEGRAM_ProcessReceivedPacket()
 *
 * @brief This API will process the received packet from MCU2
 * * @param p_packet  pointer to packet containing the received data
 * This Function Can be optimised , Due to time limits : not changes the core functionalities
 ********************************************************************************************************/
void
TELEGRAM_ProcessReceivedPacket (SPICOMMS_DATA_PACKET *p_packet)
{
  if ((p_packet->TxPacket.Buffer == NULL)
      || (p_packet->RxPacket.Buffer == NULL))
    { /* null pointer check           */
      return;
    }


   Telegram_CurrentCommand = p_packet->RxPacket.Buffer[0];
  Telegram_CheckPacketIntegrity (p_packet); /* run packet integrity checks        */

  if ((Telegram_StandardResponse == DEF_SR_COMMAND_OK)
      && (Telegram_LinkLayerStatus == DEF_LLS_STATUS_OK))
    {
    //  DEBUG_TELEGRAM ("CMD",true,Telegram_CurrentCommand);
      switch (Telegram_CurrentCommand)
        { /* check received command             */

        case DEF_CURRENT_VALUES: /* Current Values*/
          p_packet->TxPacket.Command = DEF_CURRENT_VALUES; /* process command and send requested data  */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_COUNTERS_AND_DATES:
          p_packet->TxPacket.Command = DEF_COUNTERS_AND_DATES; /* process command and send requested data  */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_DEMOUNTING_LOGBOOKS:
          p_packet->TxPacket.Command = DEF_DEMOUNTING_LOGBOOKS; /* process command and send requested data  */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_TIMEZONE_OFFSET:
          Telegram_SetTimezoneOffset (p_packet); /* process command               */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response         */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_FW_VERSION:
          Telegram_SetFwVersion (p_packet); /* process command              */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          SPIComms_Send_Data_to_MCU2(SPI_CMD_Firmware_version);    /* Send MCU1 version along with MCU2 */
          break;

        case DEF_ALARM:
        {
          const bool ok = Telegram_SetAlarm( p_packet );          /* Process command                    */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE;     /* Send standard response             */
          TELEGRAM_PrepareTransmitPacket( p_packet );             /* Create packet                      */
          if( ok )
          {
            trigger_remote_alarm( );                              /* This wiil trigger the Remote Alarm */
          }

          break;
        }

        case DEF_OPERATING_MODE:
          Telegram_SetOperatingMode (p_packet); /* process command              */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_RESULT_OC_DETECTION:
          Telegram_SetResultObsCovDet (p_packet); /* process command              */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_LASER_DATA_RX:
        Telegram_GetLaser_RX_Data(p_packet); /* process command              */
        p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
         TELEGRAM_PrepareTransmitPacket (p_packet);
        break;

        case DEF_DATE_AND_TIME:
          Telegram_SetDateTime (p_packet); /* process command               */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_RADIO_DURATION_COUNTER:
          Telegram_SetRadioDurationCounter (p_packet); /* process command               */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_RAM_FAULTS:
          Telegram_SetRamFaults (p_packet); /* process command              */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_ENTER_EM:
          Telegram_SetEnergyMode (p_packet);
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;

        case DEF_RESET_COUNTER:
          Telegram_SetResetCounter (p_packet);
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_ALARM_FORWARDING:
          Telegram_SetAlarm_Forward (p_packet); /* process command               */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_ALARM_RADIO_LINK_ERROR:
          Telegram_SetAlarm_Radio_ERROR (p_packet); /* process command               */
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
          TELEGRAM_PrepareTransmitPacket (p_packet);
          break;
        case DEF_RADIO_CONFIG_SWITCH:
          DEBUG_TELEGRAM("Commmand SPI from MCU2 privateDataSwitch", false, 0u);
          Telegram_SetRadioPrivateConfig(p_packet);
          p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE;
          TELEGRAM_PrepareTransmitPacket (p_packet);
        break;

        default:
          {
            break;
          }
        }
    }
  else
    {
      p_packet->TxPacket.Command = DEF_STANDARD_RESPONSE; /* send standard response           */
      TELEGRAM_PrepareTransmitPacket (p_packet);
    }
}

/****************************************************************************************************//**
 *                                       Telegram_CheckPacketIntegrity()
 *
 * @brief     Check integrity of the packet received from MCU2
 *
 * @param p_packet  pointer to packet containing the received data
 ********************************************************************************************************/
static void
Telegram_CheckPacketIntegrity (SPICOMMS_DATA_PACKET *p_packet)
{
  uint8_t command;
  uint32_t length;
  uint16_t crc;

  Telegram_StandardResponse = DEF_SR_COMMAND_OK;
  Telegram_LinkLayerStatus = DEF_LLS_STATUS_OK;

  command = p_packet->RxPacket.Buffer[0u]; /* check for the validity of command  */
  if (command > TotalSPICommands)
    {
      Telegram_StandardResponse |= DEF_SR_COMMAND_NOT_FOUND;
    }

  if (p_packet->RxPacket.Length == 0u)
    { /* check payload size           */
      Telegram_StandardResponse |= DEF_SR_PAYLOAD_EMPTY;
    }

  length = Telegram_GetCommandLength (command); /* check length of the application data */
  if (length != p_packet->RxPacket.Length)
    {
      Telegram_StandardResponse |= DEF_SR_WRONG_LENGTH;
      Telegram_LinkLayerStatus |= DEF_LLS_LENGTH_VIOLATION;
    }

  crc = CRC_Calculate (p_packet->RxPacket.LinkLayerStatus, /* check crc violation           */
                       p_packet->RxPacket.Buffer, p_packet->RxPacket.Length);
  if (crc != p_packet->RxPacket.CRC)
    {
      Telegram_LinkLayerStatus |= DEF_LLS_CRC_VIOLATION;
    }

  if (p_packet->RxPacket.BeginFrame != DEF_BEGIN_FRAME)
    { /* check start of frame violation     */
      Telegram_LinkLayerStatus |= DEF_LLS_BF_VIOLATION;
    }

  if (p_packet->RxPacket.EndFrame != DEF_END_FRAME)
    { /* check end of frame violation     */
      Telegram_LinkLayerStatus |= DEF_LLS_EF_VIOLATION;
    }
}

/****************************************************************************************************//**
 *                                       Telegram_GetCommandLength()
 *
 * @brief     Get length of data associated with the command
 * @param p_packet  pointer to packet containing the received data
 * @return  length of the data
 ********************************************************************************************************/
static uint32_t Telegram_GetCommandLength (uint8_t command)
{
  uint32_t length = 0u;

  switch (command)
    {
    case DEF_STANDARD_RESPONSE:
      length = DEF_LEN_STANDARD_RESPONSE;
      break;

    case DEF_CURRENT_VALUES:
      length= DEF_CMD_FROM_MCU2_WITH_NO_DATA;
      break;

    case DEF_LOGBOOK_RECORD:
      length= DEF_CMD_FROM_MCU2_WITH_NO_DATA;
      break;

    case DEF_DEMOUNTING_LOGBOOKS:
      length= DEF_CMD_FROM_MCU2_WITH_NO_DATA;
      break;

    case DEF_TIMEZONE_OFFSET:
      length = DEF_LEN_TIMEZONE_OFFSET;
      break;

    case DEF_COUNTERS_AND_DATES:
      length= DEF_CMD_FROM_MCU2_WITH_NO_DATA;
      break;

    case DEF_FW_VERSION:
      length = DEF_LEN_FW_VERSION;
      break;

    case DEF_ALARM:
      length = DEF_LEN_ALARM;
      break;

    case DEF_OPERATING_MODE:
      length = DEF_LEN_OPERATING_MODE;
      break;

    case DEF_PRODUCTION_BLOB:
      length = DEF_LEN_PRODUCTION_BLOB;
      break;

    case DEF_TERMINATE_PRODUCTION:
      length = DEF_LEN_TERMINATE_PRODUCTION;
      break;

    case DEF_RADIO_TEST:
      length = DEF_LEN_RADIO_TEST;
      break;
    case DEF_AIRING_LIGHT:
      length = DEF_LEN_AIRING_LIGHT;
      break;

    case DEF_RESULT_OC_DETECTION:
      length = DEF_LEN_RESULT_OC_DETECTION;
      break;

    case DEF_LASER_DATA_RX:
      length = DEF_LEN_LASER_RX_DATA;
      break;

    case DEF_DATE_AND_TIME:
      length = DEF_LEN_DATE_AND_TIME;
      break;

    case DEF_RADIO_DURATION_COUNTER:
      length = DEF_LEN_RADIO_DURATION_COUNTER;
      break;

    case DEF_RAM_FAULTS:
      length = DEF_LEN_RAM_FAULTS;
      break;

    case DEF_ENTER_EM:
      length = DEF_LEN_ENERGY_MODE;
      break;

    case DEF_RESET_COUNTER:
      length = DEF_LEN_RESET_COUNTER;
      break;
    case DEF_ALARM_FORWARDING:
      length = DEF_LEN_ALARM_FORWARDING;
      break;
    case DEF_ALARM_RADIO_LINK_ERROR:
         length = DEF_LEN_RADIO_LINK_ERROR;
         break;
    case DEF_RADIO_CONFIG_SWITCH:
        length = DEF_LEN_PRIVATE_RADIO_DATA;
        break;
    default:
      {
        break;
      }
    }
  return length;
}


/****************************************************************************************************//**
 *                                       get_radio_test()
 *
 * @brief     Get the Radio test form FTM
 * @param void
 * @return Readio test val enum
 ********************************************************************************************************/
RADIO_TEST
get_radio_test (void)
{
  
  return test_value;
}

/****************************************************************************************************//**
 *                                      Sget_radio_test()
 *
 * @brief     Set the Radio test form FTM
 * @param set the Radio Test
 * @return None
 ********************************************************************************************************/
void
set_radio_test (RADIO_TEST rd_val)
{
  test_value = rd_val;
}

/****************************************************************************************************//**
 *                                       get_system_state_for_MCU2()
 *
 * @brief     Get the System State
 * @param void
 * @return None
 ********************************************************************************************************/
uint16_t
get_system_state_for_MCU2 (void)
{
  uint16_t SPI_SYSTEM_data_read;
  /* Just get latest Update before sending information*/
  SPI_SYSTEM_data_read = update_system_state_for_MCU2 ();
  return SPI_SYSTEM_data_read;
}


/****************************************************************************************************//**
 *                                       Set_system_state_for_MCU2()
 *
 * @brief     Set the state
 * Combination of state can be possible (e.g Dark/Light with alarm )
 * @param set the state
 * @return None
 ********************************************************************************************************/
uint16_t
update_system_state_for_MCU2 (void)
{

  Ads_state_t mounting_status;
  bool set_clear = false;
  uint8_t smoke_feature = 0u;
  uint16_t SPI_SYSTEM_data = 0u;

  if((super_smoke_detected() == true) || smoke_detected() == true)
  {
      SPI_SYSTEM_data = SPI_SYSTEM_data | (1u << (uint8_t) SPI_Smoke_Alarm_State);
  }
  else
  {
      SPI_SYSTEM_data = (SPI_SYSTEM_data & ~(1u << (uint8_t) SPI_Smoke_Alarm_State));
  }

  if((getHeatState() == Heat_super) || (getHeatState() == Heat_high))
  {
      SPI_SYSTEM_data = SPI_SYSTEM_data | (1u << (uint8_t) SPI_Heat_Alarm_State);
  }
  else
  {
      SPI_SYSTEM_data = (SPI_SYSTEM_data & ~(1u << (uint8_t) SPI_Heat_Alarm_State));
  }

  if((getCoState() == co_super) || (getCoState() == co_high))
  {
      SPI_SYSTEM_data = SPI_SYSTEM_data | (1u << (uint8_t) SPI_CO_Alarm_State);
  }
  else
  {
      SPI_SYSTEM_data = (SPI_SYSTEM_data & ~(1u << (uint8_t) SPI_CO_Alarm_State));
  }

  mounting_status = hal_get_ads_state();

  if (mounting_status == Ads_onBase)
  {
    SPI_SYSTEM_data = SPI_SYSTEM_data | (1u << (uint16_t) SPI_Demounted_State);
  }
  else if (mounting_status == Ads_offBase)
  {
    SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data & ~(1u << (uint16_t) SPI_Demounted_State));
  }
  else
  {
    /* LDRA*/
  }

  set_clear = GetAssistanceLightStatus();

  if (set_clear == true)
  {
    SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data | (1u << (uint16_t) SPI_Assistance_Light_State));
  }
  else
  {
      SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data & ~(1u << (uint16_t) SPI_Assistance_Light_State));
  }

  set_clear = get_Ambient_Light_Status();

  if (set_clear == true)
  {
      SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data | (1u << (uint16_t) SPI_Ambient_Light_Darkness_State));
  }
  else
  {
      SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data & ~(1u << (uint16_t) SPI_Ambient_Light_Darkness_State));
  }

  smoke_feature = DataLogging_GetDeviceConfig();

  if (smoke_feature == 0u)
  {
      SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data | (1u << (uint16_t) SPI_Device_Configuration_State_Smoke));
  }
  else
  {
      SPI_SYSTEM_data = (uint16_t) (SPI_SYSTEM_data & ~(1u << (uint16_t) SPI_Device_Configuration_State_Smoke));
  }

  return SPI_SYSTEM_data;
}


/**
 * @brief    This Function will inform MCU_2 about the Action
 *   Alarm
 */
#if 0
void
inform_MCU2_Alarm(void)
{
  RTOS_ERR err;
  OSFlagPost (&InterMCUSPIComms_CommandsFlag, /* post flag to initiate SPI communication  */
              SPICmdAlarm,
              OS_OPT_POST_FLAG_SET,
              &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

#endif

/**
 * @brief get the Remote Alarm State
 * @param  none
 * @return
 */
uint8_t get_remote_alarm_status_MCU_2( void )
{
  return remote_alarm;
}

bool set_remote_alarm_status_MCU_2( uint8_t alarm )
{
  bool ok = false;

  if( ( alarm >= (uint8_t)REM_ALM_END ) && ( alarm < (uint8_t)REM_ALM_MAX ) )
  {
    remote_alarm = alarm;

    ok = true;
  }

  return( ok );
}

void clr_remote_alarm_status_MCU_2( void )
{
  remote_alarm = REM_ALM_END;
}

/**
 * @brief get the Current Values & fill the Global Structure
 * @param  none
 * @return
 */

void
get_value_for_SPI (behaviour_state_enum_System_modes data_mode,
                   uint32_t fault_val , Current_values *tx_data)
{

  int16_t heatVal = 0;

  if ((data_mode == Operational_Mode) || (data_mode == Functional_Test_Mode))
  {
      /* Operational Mode with Data without fault*/
       if ((fault_val == DEF_NO_FAULT))
       {
          tx_data->temp_val = (int16_t) SHT41_get_temperature ();
          tx_data->Humidity_val = (uint8_t) SHT41_get_humidity ();
          tx_data->battA_val = (uint16_t) get_battery_A_Voltage ();
          tx_data->battB_val = (uint16_t) get_battery_B_Voltage ();
          if(data_mode ==  Functional_Test_Mode)
          {
              tx_data->temp_val = (int16_t) get_TempVal();
              tx_data->Humidity_val = (uint8_t) get_HumidityVal();
              tx_data->battA_val = (uint16_t) get_ftm_Batt_A_Sim_Vol ();
              tx_data->battB_val = (uint16_t) get_ftm_Batt_B_Sim_Vol ();
          }
          tx_data->Co_val = (uint16_t) getCoAfterCompensation ();
          heatVal = (int16_t)getHeatAfterCompensation ();
          if(heatVal < 0)
          {
              tx_data->Heat_val = SPI_INVALID_VAL_16_BYTE;
          }
          else
          {
              tx_data->Heat_val = (uint16_t)heatVal;
          }

          tx_data->Smoke_val = smoke_get_percentage();
          tx_data->Degraded_chamber_val = smoke_dust_get_correction_factor( );
          tx_data->Soiling_val = 0u;
          tx_data->Coverage_Status = 0x19; // Future use
          tx_data->Obs_Status = 0x1A; // Future use
          /*Check the Ambient light Bist */
          const bool perform_measurement = false;
          if (ambient_light_get_BIST ( perform_measurement ) == false)
          {
              tx_data->brightness = SPI_INVALID_VAL_8_BYTE;
          }
          else
          {
              tx_data->brightness = ambient_light_get_status ();
          }

       }
       else /* Invalid data in operational mode */
       {
          /* Default Temperature in case of HTU20D chip failure */
          if ((fault_val & DEF_TEMP_SENSOR_HW_FAULT) != 0u)
          {

              tx_data->temp_val = SPI_INVALID_VAL_TEMP;
          }
          else
          {
              if(data_mode ==  Functional_Test_Mode)
              {
                  tx_data->temp_val = (int16_t) get_TempVal();
              }
              else
              {
                  tx_data->temp_val = (int16_t) SHT41_get_temperature ();
              }
          }
          /* Default Humidity failure */
          if ((fault_val & DEF_HUMIDITY_SENSOR_HW_FAULT) != 0u)
          {
              tx_data->Humidity_val = SPI_INVALID_VAL_8_BYTE;
          }
          else
          {
              if(data_mode ==  Functional_Test_Mode)
              {
                  tx_data->Humidity_val = (uint8_t) get_HumidityVal();
              }
              else
              {
                  tx_data->Humidity_val = (uint8_t) SHT41_get_humidity ();
              }
          }
          /* Default Battery failure */
          if ((fault_val & DEF_BATTERY_FAULT) != 0u)
          {

              tx_data->battA_val = (uint16_t) get_battery_A_Voltage ();
              tx_data->battB_val = (uint16_t) get_battery_B_Voltage ();
          }
          else
          {
              if(data_mode ==  Functional_Test_Mode)
              {
                  tx_data->battA_val = (uint16_t) get_ftm_Batt_A_Sim_Vol ();
                  tx_data->battB_val = (uint16_t) get_ftm_Batt_B_Sim_Vol ();
              }
              else
              {
                  tx_data->battA_val = (uint16_t) get_battery_A_Voltage ();
                  tx_data->battB_val = (uint16_t) get_battery_B_Voltage ();
              }
          }

          /* Default Co failure */
          if ((fault_val & DEF_CO_SENSOR_HW_FAULT) != 0u)
          {

              tx_data->Co_val = SPI_INVALID_VAL_16_BYTE;
          }
          else
          {
              tx_data->Co_val = (uint16_t) getCoAfterCompensation ();
          }
          /* Default Smoke failure */
          if ((fault_val & DEF_SMOKE_CHAMBER_HW_FAULT) != 0u)
          {

              tx_data->Smoke_val = SPI_INVALID_VAL_8_BYTE;
          }
          else
          {
              tx_data->Smoke_val = smoke_get_percentage ();
          }
          /* Default Smoke failure */
          if ((fault_val & DEF_HEAT_SENSOR_HW_FAULT) != 0u)
          {
              tx_data->Heat_val = SPI_INVALID_VAL_16_BYTE;
          }
          else
          {
              heatVal = (int16_t)getHeatAfterCompensation ();
              if(heatVal < 0)
              {
                  tx_data->Heat_val = SPI_INVALID_VAL_16_BYTE;
              }
              else
              {
                  tx_data->Heat_val = (uint16_t)heatVal;
              }
          }
          /* Default Smoke degraded chamber failure */
          if ((fault_val & DEF_DEG_SMOKE_CHAMBER_FAULT) != 0u)
          {
              tx_data->Degraded_chamber_val = 100U;
          }
          else
          {
              tx_data->Degraded_chamber_val = smoke_dust_get_correction_factor( );
          }
          /* soiling feature is deprecated */
          tx_data->Soiling_val = 0u;
		  
          /* Default coverage failure */
          if ((fault_val & DEF_COVERAGE_DET_FAULT) != 0u)
          {
              tx_data->Coverage_Status = SPI_INVALID_VAL_8_BYTE;
              tx_data->Obs_Status = SPI_INVALID_VAL_8_BYTE;
          }
          else
          {
              tx_data->Coverage_Status = 0x19; // Future use
              tx_data->Obs_Status = 0x1A;      // Future use
          }
          /* Ambeint light */
          const bool perform_measurement = false;
          if (ambient_light_get_BIST (perform_measurement) == false)
          {
              tx_data->brightness = SPI_INVALID_VAL_8_BYTE;
          }
          else
          {
              tx_data->brightness = ambient_light_get_status ();
          } /*Check the Ambient light Bist */

        }
    }
    /*  Transport or Commissioning  Mode with Data without fault*/
    else
    {
        if ((data_mode == Transport_Mode) || (data_mode == Commisioning_Mode)
          || (data_mode == Standby_Mode))
        {
          tx_data->temp_val = SPI_INVALID_VAL_TEMP;
          tx_data->Humidity_val = SPI_INVALID_VAL_8_BYTE;
          tx_data->battA_val = SPI_INVALID_VAL_16_BYTE;
          tx_data->battB_val = SPI_INVALID_VAL_16_BYTE;
          tx_data->Co_val = SPI_INVALID_VAL_16_BYTE;
          tx_data->Smoke_val = SPI_INVALID_VAL_8_BYTE;
          tx_data->Heat_val = SPI_INVALID_VAL_16_BYTE;
          tx_data->Degraded_chamber_val = SPI_INVALID_VAL_8_BYTE;
          tx_data->Soiling_val = SPI_INVALID_VAL_8_BYTE;
          tx_data->Coverage_Status = SPI_INVALID_VAL_8_BYTE;
          tx_data->Obs_Status = SPI_INVALID_VAL_8_BYTE;
          tx_data->brightness = SPI_INVALID_VAL_8_BYTE;
        }
    }




}



/****************************************************************************************************//**
*                                             Set parameter for OC ()
*
*@brief   This API will set  Obstacle detection command
*@param   PARA 1 : oc test command as per techem , Para 2 to 4 :data
*@return  NONE

********************************************************************************************************/
void Set_OC_Parameter(OC_TEST command ,uint8_t para_1,uint8_t para_2,uint8_t para_3,uint8_t para_4)
{

 OC_data.val=command; /* Command */
 OC_data.parameter_1=para_1;
 OC_data.parameter_2=para_2;
 OC_data.parameter_3=para_3;
 OC_data.parameter_4=para_4;
}

/****************************************************************************************************//**
*                                             Get parameter for OC ()
*
*@brief   This API will provide  Obstacle detection Status
*@param   Struct Obstacle detection command and parameters
*@return  NONE

********************************************************************************************************/
void get_OC_Parameter(Set_Parameter_for_OC *OC_data_rt)
{


if (OC_data_rt !=NULL)
  {
    OC_data_rt->val=OC_data.val;  /* Command */
    OC_data_rt->parameter_1=OC_data.parameter_1;
    OC_data_rt->parameter_2=OC_data.parameter_2;
    OC_data_rt->parameter_3=OC_data.parameter_3;
    OC_data_rt->parameter_4=OC_data.parameter_4;
}

}

/****************************************************************************************************//**
*                                             set_obs_det_ftm_timeover ()
*
*@brief   This API set the FTM get results valid
*@param   bool: true or false
*@return  NONE

********************************************************************************************************/
void set_obs_det_ftm_timeover(bool val)
{
  obs_det_ftm_timePeriod = val;
}

/****************************************************************************************************//**
*                                             get_obs_det_ftm_timeover ()
*
*@brief   This API validates obs detection results for FTM
*@param   n/a
*@return  true = success, false = failure

********************************************************************************************************/
bool get_obs_det_ftm_timeover(void)
{
  return obs_det_ftm_timePeriod;
}

/****************************************************************************************************//**
*                                             set_obs_det_ftm_period_timeover ()
*
*@brief   This API validates obs detection periodic results for FTM
*@param   bool: true or false
*@return  NONE

********************************************************************************************************/
void set_obs_det_ftm_period_timeover(bool val)
{
  obs_det_ftm_period_timeover = val;
}

/****************************************************************************************************//**
*                                             get_obs_det_ftm_period_timeover ()
*
*@brief   This API validates obs detection results for FTM
*@param   n/a
*@return  true = success, false = failure

********************************************************************************************************/
bool get_obs_det_ftm_period_timeover(void)
{
  return obs_det_ftm_period_timeover;
}

/****************************************************************************************************//**
*                                             set_AiringConfig
*
*@brief   This API sets the airing configuration status
*@param   uint8_t:status
*@return  NONE

********************************************************************************************************/
void set_AiringConfig(uint8_t status)
{
  airingConfigStatus = status;
}

/****************************************************************************************************//**
*                                             get_AiringConfig
*
*@brief   This API gets the airing configuration status
*@param   None
*@return  uint8_t:airingConfigStatus

********************************************************************************************************/
uint8_t get_AiringConfig(void)
{
   return airingConfigStatus;
}

/****************************************************************************************************//**
*                                             set_laser_status_validated
*
*@brief   This API sets the laser results status confirm MCU2 did call laser function.
*@param   bool:status
*@return  NONE

********************************************************************************************************/

void set_laser_status_validated(bool status)
{
  laser_result_validated = status;
}

/****************************************************************************************************//**
*                                            get_laser_status_validated
*
*@brief   This API gets the laser results status confirm MCU2 did call laser function.
*@param   None
*@return  uint8_t:airingConfigStatus

********************************************************************************************************/
bool get_laser_status_validated(void)
{
  return laser_result_validated;
}

/****************************************************************************************************//**
*                                             set_rem_alarm_silence_to_mcu2
*
*@brief   This API send 0x05 param on alarm cmd 0x07 for short press test button action  .
*@param   bool:status
*@return  NONE

********************************************************************************************************/
void set_rem_alarm_silence_to_mcu2(bool status)
{
  rem_alarm_sil = status;
}

/****************************************************************************************************//**
*                                           get_rem_alarm_silence_to_muc2
*
*@brief   This API get status to send 0x05 param on alarm cmd 0x07 for short press test button action
*@param   None
*@return  bool:rem_alarm_sil

********************************************************************************************************/
bool get_rem_alarm_silence_to_muc2(void)
{
  return rem_alarm_sil;
}


