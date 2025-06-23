/********************************************************************************************************
 ********************************************************************************************************
 *                                               MODULE
 ********************************************************************************************************
 *******************************************************************************************************/

#ifndef _TELEGRAM_H_
#define _TELEGRAM_H_

/********************************************************************************************************
*********************************************************************************************************
*                                               INCLUDES
*********************************************************************************************************
********************************************************************************************************/

#include "stdint.h"
#include "spi_comms.h"
#include "system_events.h"

/********************************************************************************************************
*********************************************************************************************************
*                                                DEFINES
*********************************************************************************************************
********************************************************************************************************/

#define DEF_BEGIN_FRAME                        (0xBF)		/* Data Frame - Start and End characters */
#define DEF_END_FRAME                          (0xEF)

#define DEF_LEN_BEGIN_FRAME_FIELD                (1u)		/* Data Frame - Lengths - in bytes 		*/
#define DEF_LEN_LENGTH_FIELD                     (2u)
#define DEF_LEN_LINK_LAYER_STATUS_FIELD          (1u)
#define DEF_LEN_COMMAND_FIELD                    (1u)
#define DEF_LEN_CRC_FIELD                        (2u)
#define DEF_LEN_END_FRAME_FIELD                  (1u)

#define DEF_STANDARD_RESPONSE                    (0u)		/* Commands 							*/
#define DEF_CURRENT_VALUES                       (1u)
#define DEF_LOGBOOK_RECORD                       (2u)
#define DEF_DEMOUNTING_LOGBOOKS                  (3u)
#define DEF_TIMEZONE_OFFSET                      (4u)
#define DEF_COUNTERS_AND_DATES                   (5u)
#define DEF_FW_VERSION                           (6u)
#define DEF_ALARM                                (7u)
#define DEF_OPERATING_MODE                       (8u)
#define DEF_PRODUCTION_BLOB                      (9u)
#define DEF_TERMINATE_PRODUCTION                (10u)
#define DEF_RADIO_TEST                          (11u)
#define DEF_ALARM_RADIO_LINK_ERROR              (12u)
#define DEF_TRIG_OC_DETECTION                   (13u)
#define DEF_RESULT_OC_DETECTION                 (14u)
#define DEF_LASER_DATA_RX                       (15u)
#define DEF_DATE_AND_TIME 		                  (16u)
#define DEF_RADIO_DURATION_COUNTER              (17u)
#define DEF_RADIO_CONFIG_SWITCH                 (18u) /* for DCR Radio Switch use*/
#define DEF_RAM_FAULTS                          (19u)
#define DEF_ENTER_EM 		                        (20u)
#define DEF_TOGGLE_AIRING_RECOMMENDATION        (21u)
#define DEF_AIRING_LIGHT                   		  (22u)
#define DEF_RESET_COUNTER                  		  (23u)
#define DEF_ALARM_FORWARDING                    (24u)


#define LASER_DISTANCE 3u
#define LASER_DITECTION 1u
#define LASER_BIST 2u
#define LASER_GAIN_OFFSET 4u

#define AIRING_FLAG_OFF                         (0x00u)
#define AIRING_FLAG_ON                          (0x01u)
#define AIRING_FLAG_TOGGLE                      (0x02u)
#define RADIOPRIV_DATA_CONFIGMODE               (0x03u)

#define DEF_LEN_LASER_RX_DATA                   (321u)   /* Not defined yet or tested   */

typedef enum
{
 Send_CW= 1,
 Send_Modulated,
 Send_OMS_S,
 Send_OMS_A,
 Send_TOM,
 Continuous_Rx,
 Tx_Power_Down,
 END_OF_RADIO_TEST
} RADIO_TEST;

/**
 * These are the Laser Module  States
 */
typedef enum
{
 LASER_NO_DETETCION= 0,  //0x00 00 - No Detection
 LASER_OBSTACLE_DETETCION,//0x00 01 - Obstacle Detection
 LASER_COVERAGE_DETETCION,//0x00 02 - Coverage Detection
 LASER_END_OF_DETECTION,//0x00 03 - End Detection (Obstacle/Coverage)
 LASER_SENSOR_FAILUARE,//0xFF FF ? Sensor Failure
 END_OF_LASER_TEST
} LASER_TEST;


/**
 * These are the variousSystem States
 */
typedef enum {
  SPI_Smoke_Alarm_State=0,
  SPI_Heat_Alarm_State,
  SPI_CO_Alarm_State,
  SPI_Demounted_State,
  SPI_Assistance_Light_State,
  SPI_Ambient_Light_Darkness_State,
  SPI_Device_Configuration_State_Smoke=6,
} SPI_TX_SYS_State;


/* OC detection */

typedef enum
{
 OC_detection= 1,
 OC_BIST,
 OC_Distance,
 OC_Calibration,
 OC_Reading,
 END_OF_OC_TEST
} OC_TEST;

typedef enum
{
  REM_ALM_END,
  REM_ALM_SMOKE,
  REM_ALM_HEAT,
  REM_ALM_CO,
  REM_ALM_TEST,
  REM_ALM_MAX
}
REM_ALM;

typedef struct
{
  OC_TEST val;
  uint8_t parameter_1;
  uint8_t parameter_2;
  uint8_t parameter_3;
  uint8_t parameter_4;
}Set_Parameter_for_OC;

/* Strut for system Values , This values need to be fill */
typedef struct
{
  int16_t temp_val;  /**< Current Temp Values */
  uint16_t battA_val; /**< Current Battery A Voltage */
  uint16_t battB_val;/**< Current Battery B Voltage */
  uint16_t Co_val;/**< Current Co Value */
  uint16_t Heat_val;/**< Current heat value */
  uint8_t Smoke_val;/**< Current Smoke value */
  uint8_t Degraded_chamber_val;/**< Degraded Chamber value */
  uint8_t Soiling_val;/**< Soiling  value */
  uint8_t Coverage_Status;/**< Coverage Status */
  uint8_t Obs_Status;/**< OBstacle status: This req May be obsolete soon */
  uint8_t brightness;/**< Brightness Status:This req May be obsolete soon  */
  uint8_t Humidity_val;/**< Humidity Value */
} Current_values;

/********************************************************************************************************
*********************************************************************************************************
*                                               FUNCTIONS
*********************************************************************************************************
********************************************************************************************************/

void TELEGRAM_Init(void);
void TELEGRAM_PrepareTransmitPacket(SPICOMMS_DATA_PACKET *p_packet);
void TELEGRAM_ProcessReceivedPacket(SPICOMMS_DATA_PACKET *p_packet);
uint16_t get_the_address(bool logbook_or_demount_log );
RADIO_TEST get_radio_test(void);
void set_radio_test(RADIO_TEST rd_val);
uint16_t update_system_state_for_MCU2 (void);
uint16_t get_system_state_for_MCU2 (void);
void inform_MCU2_current_value(void);
void inform_MCU2_Alarm(void);
uint8_t get_remote_alarm_status_MCU_2( void );
bool set_remote_alarm_status_MCU_2( uint8_t alarm );
void clr_remote_alarm_status_MCU_2( void );
void Set_OC_Parameter(OC_TEST command ,uint8_t para_1,uint8_t para_2,uint8_t para_3,uint8_t para_4);
void get_OC_Parameter(Set_Parameter_for_OC *OC_data_rt);
LASER_TEST get_laser_status();
uint16_t get_laser_distance(void);
void trigger_remote_alarm(void);
void set_laser_distance(uint16_t distance);
void get_value_for_SPI (behaviour_state_enum_System_modes data_mode, uint32_t fault, Current_values *tx_data_val);
void set_obs_det_ftm_timeover(bool val);
bool get_obs_det_ftm_timeover(void);
void set_obs_det_ftm_period_timeover(bool val);
bool get_obs_det_ftm_period_timeover(void);
void set_AiringConfig(uint8_t status);
uint8_t get_AiringConfig(void);
void set_laser_status_validated(bool status);
bool get_laser_status_validated(void);
void set_rem_alarm_silence_to_mcu2(bool status);
bool get_rem_alarm_silence_to_muc2(void);

/********************************************************************************************************
*********************************************************************************************************
*                                               MODULE END
*********************************************************************************************************
********************************************************************************************************/

#endif /* _TELEGRAM_H_ */
