#include "uartCLI.h"
#include "comms_handler.h"
#include "CO_Calibration.h"
#include "nextgen_protocol.h"
#include "os.h"
#include "em_eusart.h"
#include "dmadrv.h"
#include <string.h>
#include "system_events.h"
#include <stdarg.h>
#include "fault_handler.h"
#include "data_logging.h"
#include "core_cm33.h"
#include "stdlib.h"
#include "acquisition_Smoke.h"
#include "acquisition_Heat.h"
#include "telegram.h"
#include "spi_comms.h"
#include "diagnostics.h"
#include "eeprom_handler.h"
#include "hal_i2c.h"
#include "hal_switches.h"
#include "acquisition_Co.h"
#include "assistance_light.h"
#include "ctune.h"
#include "user_info_page.h"
#include "production.h"

#ifdef DEBUG_BUILD
/*********************************************** Local DEFINES *************************************************/

#define CLI_BUFFER_LEN          (128u)
#define MAX_RSP_LEN             (256u)
#define MAX_EEPROM_READ_BUF_LEN (128u)
/*********************************************** Local Typedefs *************************************************/
typedef struct 
{
    const char *const StringVal;
    uint32_t bitMap;
}systemFaultStrToBitMap_t;

/******************************************* Static Function Declaration ***************************************/
static const uint8_t minorFaultStartIdx = DegradedSmokeChamberFault;
static void WriteFormatted ( const char * format, ... );
static bool isVerbose(int argc, char **argv);
static bool UARTCLI_ProcessShortcut(uint8_t rxdByte, uint16_t bytesCountRxd, uint16_t *expandedCmdLen);
static cliStatus_t CLI_ExitCli(int argc, char **argv);
static cliStatus_t CLI_HelpFunc(int argc, char **argv);
static cliStatus_t CLI_ResetCpu(int argc, char **argv);
static cliStatus_t CLI_GetBehaviorialSystemMode(int argc, char **argv);
static cliStatus_t CLI_GetCurrentSystemFault(int argc, char **argv);
static cliStatus_t CLI_SetCurrentSystemFault(int argc, char **argv);
static cliStatus_t CLI_ClrCurrentSystemFault(int argc, char **argv);
static cliStatus_t CLI_set_smoke_cleanair(int argc, char **argv);
static cliStatus_t CLI_set_smoke_cal(int argc, char **argv);
static cliStatus_t CLI_start_heat_simulation(int argc, char **argv);
static cliStatus_t CLI_inject_heat_value(int argc, char **argv);
// static cliStatus_t CLI_get_last_spi_msg(int argc, char **argv);
static cliStatus_t CLI_get_spi_msg(int argc, char **argv);
static cliStatus_t CLI_dump_all_spi_msg(int argc, char **argv);
static cliStatus_t CLI_get_battery_calib(int argc, char **argv);
static cliStatus_t CLI_set_battery_calib(int argc, char **argv);
static cliStatus_t CLI_start_smoke_simulation(int argc, char **argv);
static cliStatus_t CLI_get_super_smoke_threshold(int argc, char **argv);
static cliStatus_t CLI_get_smoke_threshold(int argc, char **argv);
static cliStatus_t CLI_inject_smoke_value(int argc, char **argv);
static cliStatus_t CLI_inject_smoke_threshold(int argc, char **argv);
static cliStatus_t CLI_get_Temperature(int argc, char **argv);
static cliStatus_t CLI_ReadAllSensors(int argc, char **argv);
static cliStatus_t CLI_eeprom_erase(int argc, char **argv);
static cliStatus_t CLI_get_co_calib(int argc, char **argv);
static cliStatus_t CLI_set_co_calib(int argc, char **argv);
static cliStatus_t CLI_set_co_calib_tables(int argc, char **argv);
static cliStatus_t CLI_set_co_raw_reading(int argc, char **argv);
static cliStatus_t CLI_read_eeprom_data(int argc, char **argv);
static cliStatus_t CLI_get_eeprom_index(int argc, char **argv);
static cliStatus_t CLI_GetBehaviorialOperationalState(int argc, char **argv);
static cliStatus_t CLI_get_current_value(int argc, char **argv);
static cliStatus_t CLI_SetBehaviorialSystemMode(int argc, char **argv);
static cliStatus_t CLI_SetMountEvent(int argc, char **argv);
static cliStatus_t CLI_SetDeMountEvent(int argc, char **argv);
static cliStatus_t CLI_SendFwVersionToMcu2(int argc, char **argv);
static cliStatus_t CLI_set_assistance_light(int argc, char **argv);
static cliStatus_t CLI_set_smoke_dust_test_values(int argc, char **argv);
static cliStatus_t CLI_set_smoke_dust_test_mode(int argc, char **argv);
static cliStatus_t CLI_do_buzzer_test(int argc, char **argv);
static cliStatus_t CLI_get_operating_state(int argc, char **argv);
static cliStatus_t CLI_set_smoke_dust_set_call_count(int argc, char **argv);
static cliStatus_t CLI_print_demounting_logbook(int argc, char **argv);
static cliStatus_t CLI_set_abuf_calib( int argc, char **argv );
static cliStatus_t CLI_get_abuf_calib( int argc, char **argv );
static cliStatus_t CLI_print_event_logbook(int argc, char **argv);
static cliStatus_t CLI_set_event(int argc, char **argv);
static cliStatus_t CLI_set_eeprom_crc(int argc, char **argv);
static cliStatus_t CLI_set_ctune(int argc, char **argv);
static cliStatus_t CLI_dump_flash(int argc, char **argv);
static cliStatus_t CLI_erase_flash(int argc, char **argv);
static cliStatus_t CLI_set_production(int argc, char **argv);
static cliStatus_t CLI_get_RawLaserData(int argc, char **argv);

/*********************************************** Static Variables *************************************************/

extern uint8_t laser_data_RX[DEF_LEN_LASER_RX_DATA];
uint16_t raw_laser_data[161u] = {0};

static uint8_t CliBuffer[CLI_BUFFER_LEN] = {0};
static uint8_t CmdBuffer[CLI_BUFFER_LEN] = {0};
static uint8_t SendResp[MAX_RSP_LEN] = {0};
static uint8_t eepromReadBytes[MAX_EEPROM_READ_BUF_LEN] = {0};
static bool cli_mode_active = false;
static const uint8_t cliPrompt[] = ">> "; /* CLI prompt displayed to the user */
static const uint8_t cliUnrecog[] = "Command not recognised\r\n";
static uint8_t *cliBuffPtr = CliBuffer;
static volatile bool cmdPending = false;
static const cmdTable_t cliCommandTbl[] = 
{
    {
        .cmd            = "help",
        .handler        = CLI_HelpFunc,
        .helpText       = "List supported commands with short description",
        .maxNoOfParams     = 1
    },
    {
        .cmd            = "exitcli",
        .handler        = CLI_ExitCli,
        .helpText       = "Exit the CLI mode and enter NG mode",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "reset",
        .handler        = CLI_ResetCpu,
        .helpText       = "Software(Warm) Resets the CPU",
        .maxNoOfParams     = 0
    },
    {
        .cmd            = "get-system-mode",
        .handler        = CLI_GetBehaviorialSystemMode,
        .helpText       = "Return the Current System Mode. -v for verbose response",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "get-system-state",
        .handler        = CLI_GetBehaviorialOperationalState,
        .helpText       = "Return the Current System State. -v for verbose response",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "get-system-fault",
        .handler        = CLI_GetCurrentSystemFault,
        .helpText       = "Return the Current System Fault. -v for verbose response",
        .maxNoOfParams     = 1
    },
    {
        .cmd            = "set-system-fault",
        .handler        = CLI_SetCurrentSystemFault,
        .helpText       = "Sets one or more system faults. -h for list of allowed system faults. Comma separated values\r\n        for setting more than one system faults i.e set-system-fault 1,2,6",
        .maxNoOfParams     = 1
    },
    {
        .cmd            = "clr-system-fault",
        .handler        = CLI_ClrCurrentSystemFault,
        .helpText       = "Clears one or more system faults. -h for list of allowed system faults. Comma separated values\r\n        for clearing more than one system faults i.e clr-system-fault 0,1,4",
        .maxNoOfParams     = 1
    },
    {
        .cmd            = "unused",
        .handler        = 0,
        .helpText       = "Get soiling calibration values from EEPROM.",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "get-soiling-data",
        .handler        = 0,
        .helpText       = "Get soiling live data values from EEPROM.",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "set-soiling-calib",
        .handler        = 0,
        .helpText       = "Set soiling calibration values to EEPROM.",
        .maxNoOfParams  = 7
    },
    {
        .cmd            = "clr-soiling-sim",
        .handler        = 0,
        .helpText       = "Clear soiling adc simulation value for a given sensor.",
        .maxNoOfParams  = 2
    },
    {
        .cmd            = "set-soiling-sim",
        .handler        = 0,
        .helpText       = "Set soiling adc simulation value for a given sensor.",
        .maxNoOfParams  = 2
    },
    {
        .cmd            = "run-soiling",
        .handler        = 0,
        .helpText       = "Run soiling tests using configured sensors and display millivolts",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "set-smoke-cleanair-threshold",
        .handler        = CLI_set_smoke_cleanair,
        .helpText       = "Set smoke reflection threshold value to EEPROM.",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "set-smoke-cal-threshold",
        .handler        = CLI_set_smoke_cal,
        .helpText       = "Set smoke calibration threshold value to EEPROM.",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "start-heat-simulation",
        .handler        = CLI_start_heat_simulation,
        .helpText       = "Enters/Exits Simulated Heat mode. Pass 1 to enter and 0 to exit",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "inject-heat",
        .handler        = CLI_inject_heat_value,
        .helpText       = "Injects Simulated Heat values for testing. Pass heat in degree C scaled by 10",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "get-mcu2-spi-msg",
        .handler        = CLI_get_spi_msg,
        .helpText       = "Get one of the last 5 SPI message sent to MCU2. Pass the index (1 - 5) interested in",
        .maxNoOfParams  = 3
    },
    {
        .cmd            = "dump-all-mcu2-spi-msg",
        .handler        = CLI_dump_all_spi_msg,
        .helpText       = "Dump all the last 5 SPI messages that were sent to MCU2",
        .maxNoOfParams  = 3
    },
    {
         .cmd            = "get-battery-calib",
         .handler        = CLI_get_battery_calib,
         .helpText       = "Get battery calibration values from EEPROM.",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-battery-calib",
         .handler        = CLI_set_battery_calib,
         .helpText       = "Set battery calibration values to EEPROM.",
         .maxNoOfParams  = 4
    },
    {
        .cmd            = "start-smoke-simulation",
        .handler        = CLI_start_smoke_simulation,
        .helpText       = "Enters/Exits Simulated Smoke mode. Pass 1 to enter and 0 to exit",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "get-super-smoke-thresh",
        .handler        = CLI_get_super_smoke_threshold,
        .helpText       = "Gets the current super smoke threshold",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "get-smoke-thresh",
        .handler        = CLI_get_smoke_threshold,
        .helpText       = "Gets the current smoke threshold",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "inject-smoke",
        .handler        = CLI_inject_smoke_value,
        .helpText       = "Inject simulated smoke value",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "inject-smoke-thresh",
        .handler        = CLI_inject_smoke_threshold,
        .helpText       = "Inject simulated smoke threshold",
        .maxNoOfParams  = 1
    },
    {
        .cmd            = "Get-current-Temperature",
        .handler        = CLI_get_Temperature,
        .helpText       = "Gets current Temperature ",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "Read-all-CO-calib-sensors",
        .handler        = CLI_ReadAllSensors,
        .helpText       = "Gets Temperature,Humidity and CO",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "read-eeprom-data",
        .handler        = CLI_read_eeprom_data,
        .helpText       = "Read a chunk of EEPROM data. i.e read-eeprom-data 0(start Addr) 1024(Length in bytes). -v to organize the dump in 16 columns",
        .maxNoOfParams  = 3
    },
    {
        .cmd            = "eeprom-erase",
        .handler        = CLI_eeprom_erase,
        .helpText       = "Erase EEPROM with 0xFF's",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "get-co-calib",
        .handler        = CLI_get_co_calib,
        .helpText       = "Get co calibration values from EEPROM.",
        .maxNoOfParams  = 0
    },
    {
        .cmd            = "set-co-calib",
        .handler        = CLI_set_co_calib,
        .helpText       = "Set co calibration values to EEPROM.",
        .maxNoOfParams  = 4
    },
    {
        .cmd            = "set-co-calib-tables",
        .handler        = CLI_set_co_calib_tables,
        .helpText       = "Set/Program co calibration tables to internal FLASH.",
        .maxNoOfParams  = 5
    },
    {
        .cmd            = "set-co-raw-reading",
        .handler        = CLI_set_co_raw_reading,
        .helpText       = "Set raw co READING.",
        .maxNoOfParams  = 2
    },
    {
        .cmd            = "get-curr-value",
        .handler        = CLI_get_current_value,
        .helpText       = "Get current system values. -v for verbose response",
        .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-system-mode",
         .handler        = CLI_SetBehaviorialSystemMode,
         .helpText       = "Sets System Mode.",
         .maxNoOfParams  = 2
    },
    {
         .cmd            = "set-mount-event",
         .handler        = CLI_SetMountEvent,
         .helpText       = "Simulate mount event and sends demount logbook record to mcu2.",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "set-demount-event",
         .handler        = CLI_SetDeMountEvent,
         .helpText       = "Simulate demount event and sends demount logbook record to mcu2",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "send-fwversion",
         .handler        = CLI_SendFwVersionToMcu2,
         .helpText       = "Sends Mcu1 firmware version to Mcu2",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "set-assistance-light",
         .handler        = CLI_set_assistance_light,
         .helpText       = "Turns assistance light on or off",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-smoke-dust-test-values",
         .handler        = CLI_set_smoke_dust_test_values,
         .helpText       = "Set smoke dust correction test values",
         .maxNoOfParams  = 2
    },
    {
         .cmd            = "set-smoke-dust-test-mode",
         .handler        = CLI_set_smoke_dust_test_mode,
         .helpText       = "Enable or disables smoke dust test mode",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-smoke-dust-call-count",
         .handler        = CLI_set_smoke_dust_set_call_count,
         .helpText       = "Set smoke dust call count",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "do-buzzer-test",
         .handler        = CLI_do_buzzer_test,
         .helpText       = "Peforms a buzzer test",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "get-operating-state",
         .handler        = CLI_get_operating_state,
         .helpText       = "Get operating state",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "print-demounting-logbook",
         .handler        = CLI_print_demounting_logbook,
         .helpText       = "Print demounting logbook entries",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-abuf-calib",
         .handler        = CLI_set_abuf_calib,
         .helpText       = "Set abuf calibration",
         .maxNoOfParams  = 3
    },
    {
         .cmd            = "get-abuf-calib",
         .handler        = CLI_get_abuf_calib,
         .helpText       = "Get abuf calibration",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "print-event-logbook",
         .handler        = CLI_print_event_logbook,
         .helpText       = "Print event logbook entries",
         .maxNoOfParams  = 1
    },
    {
         .cmd            = "set-event",
         .handler        = CLI_set_event,
         .helpText       = "Set event",
         .maxNoOfParams  = 2
    },
    {
         .cmd            = "set-eeprom-crc",
         .handler        = CLI_set_eeprom_crc,
         .helpText       = "Calculate and set eeprom crc",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "set-ctune",
         .handler        = CLI_set_ctune,
         .helpText       = "Display/set ctune value",
         .maxNoOfParams  = 3
    },
    {
         .cmd            = "dump-flash",
         .handler        = CLI_dump_flash,
         .helpText       = "Dump contents of FLASH",
         .maxNoOfParams  = 3
    },
    {
         .cmd            = "erase-flash",
         .handler        = CLI_erase_flash,
         .helpText       = "Erase information page of FLASH",
         .maxNoOfParams  = 0
    },
    {
         .cmd            = "set-prod",
         .handler        = CLI_set_production,
         .helpText       = "Set/clr production",
         .maxNoOfParams  = 2
    },
    {
         .cmd            = "get-laser-raw-data",
         .handler        = CLI_get_RawLaserData,
         .helpText       = "Laser-raw-data",
         .maxNoOfParams  = 1
    },
};

/*@Note: Take care to sync the strings if values added or removed */
static const systemFaultStrToBitMap_t faultStringMap[] = 
{
    {"Smoke Chamber Diode Hw Fault"             ,DEF_SMOKE_CHAMBER_HW_FAULT },
    {"Obstacle Detection Hw Fault"              ,DEF_OBSTACLE_DET_HW_FAUT   },
    //{"Soiling Detection Hw Fault"               ,DEF_SOILING_DET_HW_FAULT   },
    {"Demounting Detection Hw Fault"            ,DEF_DEMOUNTING_DET_HW_FAULT},
    {"CO Senser Hw Fault"                       ,DEF_CO_SENSOR_HW_FAULT     },
    {"Temperature Sensor Hw Fault"              ,DEF_TEMP_SENSOR_HW_FAULT   },
    {"Humidity Sensor Hw Fault"                 ,DEF_HUMIDITY_SENSOR_HW_FAULT},
    {"Heat Sensor Hw Fault"                     ,DEF_HEAT_SENSOR_HW_FAULT   },
    {"Buzzer HwFault"                           ,DEF_BUZER_HW_FAULT         },
    {"Battery Fault"                            ,DEF_BATTERY_FAULT          },
    {"CO End Of Life Fault"                     ,DEF_CO_EOL_FAULT           },
    {"MCU1 RAM Fault"                           ,DEF_MCU1_RAM_FAULT         },
    {"MCU2 SPI COMMS Timeout Fault"             ,DEF_MCU2_SPI_COMMS_TIMEOUT },
    {"Radio Fault"                              ,DEF_RADIO_FAULT            },
    {"MCU2 RAM Fault"                           ,DEF_MCU2_RAM_FAULT         },
    {"Degraded Smoke Chamber Fault"             ,DEF_DEG_SMOKE_CHAMBER_FAULT},
    {"Obstacle Detected Fault"                  ,DEF_OBSTACLE_DET_FAULT     },
    //{"Soiling Detected Fault"                   ,DEF_SOILING_DET_FAULT      },
    {"Coverage Detected Fault"                  ,DEF_COVERAGE_DET_FAULT     },
    {"Temp Sensor Out Of Bounds Fault"          ,DEF_TEMP_SENSOR_OOB_FAULT  },
    {"Humidity Sensor Out Of Bounds Fault"      ,DEF_HUMIDITY_SENSOR_OOB_FAULT},
    {"Test Button Fault"                        ,DEF_TEST_BUTTON_FAULT        },
    {"Demounted Too Long Fault"                 ,DEF_DEM_TOO_LONG_FAULT       },
    {"EEPROM Calib Data Corruption Fault"       ,DEF_EEPROM_CAL_DATA_CORRUPT_FAULT}
};

static const char *spiCmdString[] = 
{
    "SPI Cmd Invalid",
    "SPI Cmd Current Values",
    "SPI Cmd Log book Record",
    "SPI Cmd Demounting Log book",
    "SPI Cmd Time Zone",
    "SPI Cmd Counters Dates",
    "SPI Cmd Fw Version",
    "SPI Cmd Alarm",
    "SPI Cmd Operating Mode",
    "SPI Cmd Production BLOB",
    "SPI Cmd Terminate Production",
    "SPI Cmd Radio Test",
    "SPI Cmd Radio Link Error",
    "SPI Cmd Trig OC Detection",
    "SPI Cmd OC detection Result",
    "SPI Cmd Laser Calibration",
    "SPI Cmd Set Dateand Time",
    "SPI Cmd Radio Duration",
    "SPI Cmd Funtional Test", 
    "SPI Cmd Ram Fault", 
    "SPI Cmd Enter EMx", 
    "SPI Cmd Toggle Config Air",
    "SPI Cmd AriringLight",
    "SPI Cmd ResetCounter",
    "SPI Cmd Alarm_Forwarding",
    "SPI Cmd SlaveTransfer"
};

static const char *AlarmTypeString[] = 
{
    "Alarm End",
    "Smoke Alarm",
    "Heat Alarm",
    "CO Alarm",
    "Test Alarm",
    "Alarm Muted"
};

static const char *LinkErrStatus[] = 
{
    "No Link Error",
    "Length Violation",
    "CRC Violation",
    "EF Violation",
    "BF Violation",
};

typedef enum
{
    NoLinkError = 0x00,
    LengthViolation = 0x01,
    CRCViolation = 0x02,
    EFViolation = 0x04,
    BFViolation = 0x08,
}LinkError_t;

/*********************************************** Global Variables *************************************************/
cli_t cmdLineIf;

/**************************************************Static Function Definitions ************************************/

/***** CLI Command Handlers *******/

static int get_next_arg( int argc, char **argv, int arg )
{
  arg++;

  for( ; arg < argc; arg++ )
  {
    if( *argv[ arg ] != '-' )
    {
      break;
    }
  }

  return( ( arg >= argc ? 0 : arg ) );
}

static bool isOption( int argc, char **argv, const char * opt )
{
  bool found = false;

  if( argc > 1 )
  {
    const size_t opt_len = strlen( opt );
      
    for( int i = 1; ( i < argc ) && ( found == false ); i++ )
    {
      const int match = strncmp( argv[ i ], opt, opt_len );

      found = ( match == 0 );
    }
  }

  return( found );
}

static bool isVerbose(int argc, char **argv)
{
  return( isOption( argc, argv, "-v" ) );
}

static cliStatus_t CLI_ExitCli(int argc, char **argv)
{
    RTOS_ERR err;
    cmdLineIf.printFunc("Exiting CLI mode and entering NG mode. \r\n You can now Open NG Commander for FTM/Calibration related commands \n\r");
    bool dmaActive;
    while(true)
    {
        DMADRV_TransferActive(dmaTxChannel, &dmaActive);
        if (!dmaActive) 
        {
            break;
        }
        OSTimeDly(2, OS_OPT_TIME_DLY, &err);
    }
    //Wait For the TX fifo to be flushed before 
    OSTimeDly(50, OS_OPT_TIME_DLY, &err);
    EUSART_Enable(EUSART0, false);
    EUSART_IntClear(EUSART0, 0xFFFFFFFFu);
    EUSART0->STARTFRAMECFG = STX;
    EUSART0->SIGFRAMECFG = ETX;
    EUSART_Enable(EUSART0, eusartEnable);
    UARTCLI_SetCliMode(false);
    SetAssistanceLightStatus(false);
    cmdLineIf.printFunc("Command executed successfully\r\n");
    return UARTCLI_OK;
}

static cliStatus_t CLI_HelpFunc(int argc, char **argv)
{
    RTOS_ERR err;
    bool cmdFound = false;
    if(argc > 1)    /*Help on a specific command, then print the help string for that command*/
    {
        if(!isVerbose(argc, argv))
        {
            for(uint16_t idx = 0; idx < cmdLineIf.cmdCount; idx++) 
            {
                if(strcmp(argv[1], cmdLineIf.cmdTbl[idx].cmd) == 0) 
                {
                    /* Found a match, print it's help string. */
                    cmdLineIf.printFunc("%d %s\r\n", (idx+1), cmdLineIf.cmdTbl[idx].helpText);
                    /* Very basic protection against Number Of arguments passed*/
                    cmdFound = true;
                }
            }
            if(false == cmdFound)
            {
                cmdLineIf.printFunc("Command not supported\r\n");
            }
        }
        else
        {
            cmdLineIf.printFunc("Following are the supported commands: \r\n");
            for(uint16_t idx = 0; idx < cmdLineIf.cmdCount; idx++)
            {
                cmdLineIf.printFunc("%d %s\r\n", (idx+1), cmdLineIf.cmdTbl[idx].cmd);
                cmdLineIf.printFunc("        ");
                cmdLineIf.printFunc("%s \r\n", cmdLineIf.cmdTbl[idx].helpText);
                OSTimeDly(1, OS_OPT_TIME_DLY, &err);
            }
        }
    }
    else /*No command passed, display the list of supported commands with the help string"*/
    {
        cmdLineIf.printFunc("Following are the supported commands: \r\n");
        for(uint16_t idx = 0; idx < cmdLineIf.cmdCount; idx++)
        {
            cmdLineIf.printFunc("%d %s\r\n", (idx+1), cmdLineIf.cmdTbl[idx].cmd);
            OSTimeDly(1, OS_OPT_TIME_DLY, &err);
        }
    }
    cmdLineIf.printFunc("Command executed successfully\r\n");
    return UARTCLI_OK;
}

static cliStatus_t CLI_ResetCpu(int argc, char **argv)
{
    NVIC_SystemReset();
}

static cliStatus_t CLI_GetBehaviorialSystemMode(int argc, char **argv)
{
    /*@Note: Take care to sync the strings if values added or removed */
    const char *const verbose_string[] = 
    {
        "Standby Mode",
        "Commisioning Mode",
        "Operational Mode",
        "Functional Test Mode",
        "Transport Mode,"
        "Shutdown Mode",
    };
    behaviour_state_enum_System_modes current_system_mode = getBehavioural_System_Modes(false);
    if(isVerbose(argc, argv))
    {
        cmdLineIf.printFunc("%s\r\n", verbose_string[current_system_mode]);
        cmdLineIf.printFunc("Command executed successfully\r\n");
    }
    else
    {
        cmdLineIf.printFunc("%d\r\n", current_system_mode);
    }
    return UARTCLI_OK;
}


static cliStatus_t CLI_get_co_calib(int argc, char **argv)
{
    const uint16_t  na_per_ppm                   = data_logging_get_na_per_ppm( );
    const uint16_t  COCal                        = DataLogging_GetCOCal( );
    const uint8_t   calib_temperature            = DataLogging_GetCOCalibTemperature( );
    const uint8_t   calib_humidity               = DataLogging_GetCOCalibHumidity( );

    cmdLineIf.printFunc("CONAPPM     = %u\r\n", na_per_ppm );
    cmdLineIf.printFunc("COCAL       = %u\r\n", COCal );
    cmdLineIf.printFunc("CALTEMP     = %u\r\n", calib_temperature );
    cmdLineIf.printFunc("CALHUM      = %u\r\n", calib_humidity );

    return( UARTCLI_OK );
}

static cliStatus_t CLI_set_co_calib(int argc, char **argv)
{
    if(argc < 5 )
    {
      cmdLineIf.printFunc("Set CO calibration values into EEPROM\r\n" );
      cmdLineIf.printFunc("set-co-calib <naperppm> <cocal> <varthresh> <shortcir> <opencir> <calibtemp> <calibhum> \r\n" );
      cmdLineIf.printFunc("where:\r\n" );
      cmdLineIf.printFunc("    conappm: CO Calibration - na per ppm\r\n" );
      cmdLineIf.printFunc("      cocal: CO Calibration - CO CAL\r\n" );
      cmdLineIf.printFunc("  calibtemp: Calibration temperarure code\r\n" );
      cmdLineIf.printFunc("   calibhum: Calibration humidity percentage\r\n" );
      cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
    }
    else
    {
      const uint16_t  na_per_ppm                   = atoi( argv[ 1 ] );
      const uint16_t  COCal                        = atoi( argv[ 2 ] );
      const uint8_t   calib_temperature            = atoi( argv[ 3 ] );
      const uint8_t   calib_humidity               = atoi( argv[ 4 ] );

      data_logging_set_na_per_ppm( na_per_ppm );
      DataLogging_SetCOCal( COCal );
      DataLogging_SetCOCalibTemperature( calib_temperature );
      DataLogging_SetCOCalibHumidity( calib_humidity );
      
      DataLogging_SetCRC( );

      CLI_get_co_calib( 0, NULL );

      cocal_read_eeprom_values( );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_set_co_calib_tables( int argc, char **argv )
{
  bool ok = true;

  const bool opt_help   = isOption( argc, argv, "-?" );

  if( opt_help || ( argc < 2 ) )
  {
    cmdLineIf.printFunc("Set CO calibration tables into internal FLASH\r\n" );
    cmdLineIf.printFunc("set-co-calib-tables {opts} [hexstring]>\r\n" );
    cmdLineIf.printFunc("Add the given [hexstring] to the internal programming buffer. When\r\n" );
    cmdLineIf.printFunc("the buffer is complete is can be programmed to the internal FLASH.\r\n" );
    cmdLineIf.printFunc("The [hexstring] is hex pairs e.g 1AFFE7A5...... Keep calling until\r\n" );
    cmdLineIf.printFunc("the buffer is ready for progamming. The last two bytes must be the\r\n" );
    cmdLineIf.printFunc("CRC. Programming will not occur if the CRC is incorrect. The first\r\n" );
    cmdLineIf.printFunc("call should have the reset option with or without a payload string\r\n" );
    cmdLineIf.printFunc("followed by the data before programming.\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  -r: Reset internal programming buffer\r\n" );
    cmdLineIf.printFunc("  -p: Program internal buffer to FLASH\r\n" );
    cmdLineIf.printFunc("  -?: Print this help information\r\n" );
  }
  else
  {
    const bool opt_prog   = isOption( argc, argv, "-p" );
    const bool opt_reset  = isOption( argc, argv, "-r" );

    /* Get first argument ignoring any option parameters */
    const int arg = get_next_arg( argc, argv, 0 );

    /* Is reset option string specified without any data? */
    if( opt_reset && ( arg <= 0 ) )
    {
      /* Yes, just reset programming buffer */
      ( void )cocal_update_prog_buf_ascii( NULL );
    }

    /* Is there any payload data? */
    if( arg > 0 )
    {
      /* Add data to internal programming buffer */
      ok = cocal_update_prog_buf_ascii( argv[ arg ] );
    }

    /* Ready to program? */
    if( ok && opt_prog )
    {
      /* Yes, program to internal flash */
      ok = cocal_program_calib( );
    }

    /* Inform caller */
    if( ok )
    {
      cmdLineIf.printFunc( "OK\r\n" );
    }
    else
    {
      cmdLineIf.printFunc( "FAILED\r\n" );
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_co_raw_reading( int argc, char **argv )
{
  if(argc < 2 )
  {
    cmdLineIf.printFunc("Set the raw co reading to be used\r\n" );
    cmdLineIf.printFunc("set-co-raw-reading {opts} <reading>\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  -sp: Turn simulation on for the ppm co reading, otherwise off\r\n" );
    cmdLineIf.printFunc("  -sr: Turn simulation on for the raw co reading, otherwise off\r\n" );
    cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
  }
  else
  {
    const int arg = get_next_arg( argc, argv, 0 );

    if( arg > 0 )
    {
      const bool opt_simulate_ppm = isOption( argc, argv, "-sp" );
      const bool opt_simulate_raw = isOption( argc, argv, "-sr" );

      const uint16_t reading = atoi( argv[ arg ] );
      
      if( opt_simulate_ppm )
      {
        acqco_simulated_ppm_co_reading( reading );
      }
      else if( opt_simulate_raw )
      {
        acqco_simulated_raw_co_reading( reading );
      }
      else
      {
        acqco_simulate_raw_co_reading( false );
      }
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_assistance_light( int argc, char **argv )
{
  if(argc < 2 )
  {
    cmdLineIf.printFunc("Turns assistance light on or off\r\n" );
    cmdLineIf.printFunc("set-assistance-light <onoff>\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  0: Turns light off\r\n" );
    cmdLineIf.printFunc("  1: Turns light on\r\n" );
    cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
  }
  else
  {
    const int arg = get_next_arg( argc, argv, 0 );

    if( arg > 0 )
    {
      const uint16_t onoff = atoi( argv[ arg ] );

      if( onoff )
      {
        GPIO_TurnAssistanceLEDon( );
      }
      else
      {
        GPIO_TurnAssistanceLEDoff( );
      }
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_smoke_dust_test_values(int argc, char **argv)
{
  if( argc < 3 )
  {
    cmdLineIf.printFunc("Set smoke dust correction values\r\n" );
    cmdLineIf.printFunc("set-smoke-dust-corr-values <raw> <ref>\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("       raw: RAW Smoke value\r\n" );
    cmdLineIf.printFunc("    thresh: Cal reflection threshold\r\n" );
    cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
  }
  else
  {
    const uint32_t raw       = ( uint32_t )atoi( argv[ 1 ] );
    const uint32_t thresh    = ( uint32_t )atoi( argv[ 3 ] );

    cmdLineIf.printFunc("Raw           = %lu\r\n", raw );
    cmdLineIf.printFunc("Thresh        = %lu\r\n", thresh );

    smoke_dust_set_raw_test_value( raw );
    smoke_dust_set_calref_test_value( thresh );
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_smoke_dust_test_mode(int argc, char **argv)
{
  /* Get first argument ignoring any option parameters */
  const int arg = get_next_arg( argc, argv, 0 );

  if( arg > 0 )
  {
    const int mode = atoi( argv[ arg ] );

    const bool test_mode = ( mode != 0 ? true : false );

    smoke_dust_set_test_mode( test_mode );
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_smoke_dust_set_call_count(int argc, char **argv)
{
  /* Get first argument ignoring any option parameters */
  const int arg = get_next_arg( argc, argv, 0 );

  if( arg > 0 )
  {
    const int count = atoi( argv[ arg ] );

    //smoke_dust_set_call_count( ( uint32_t )count );
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_print_demounting_logbook( int argc, char **argv )
{
  dl_demounting_logbook_record_t record = { 0 };

  for( int i = 0; i < DEF_LEN_DEMOUNTING_LOGBOOK_RECORDS_MAX; i++ )
  {
    DataLogging_GetDemountingLogbookRecord( i, &record );

    if( record.Count != 0xFFFF )
    {
      cmdLineIf.printFunc( "Index      : %d\r\n", i );
      cmdLineIf.printFunc( "Event Id   : %d\r\n", record.Count );
      cmdLineIf.printFunc( "Timestamp  : %lu\r\n", record.Timestamp );
      cmdLineIf.printFunc( "Mode       : %d\r\n", record.OperatingMode );
      cmdLineIf.printFunc( "Event Type : %d\r\n", record.ID );
      cmdLineIf.printFunc( "\n\r" );
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_print_event_logbook( int argc, char **argv )
{
  dl_event_logbook_record_t record = { 0 };

  for( int i = 0; i < DEF_LEN_EVENT_LOGBOOK_RECORDS_MAX; i++ )
  {
    DataLogging_GetEventLogbookRecord( i, &record );

    if( ( record.Timestamp != 0ul ) && ( record.Timestamp != 0xFFFFFFFF ) )
    {
      cmdLineIf.printFunc( "Index      : %d\r\n", i );
      cmdLineIf.printFunc( "Event Id   : %d\r\n", record.Count );
      cmdLineIf.printFunc( "Timestamp  : %lu\r\n", record.Timestamp );
      cmdLineIf.printFunc( "Mode       : %d\r\n", record.OperatingMode );
      cmdLineIf.printFunc( "Event Type : %d\r\n", record.ID );
      cmdLineIf.printFunc( "Data       : " );

      for( int j = 0; j < 8; j++ )
      {
        cmdLineIf.printFunc( "0x%02X ", record.Data[ j ] );
      }

      cmdLineIf.printFunc( "\n\r" );
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_event( int argc, char **argv )
{
  if( argc < 3 )
  {
    cmdLineIf.printFunc("Set event\r\n" );
    cmdLineIf.printFunc("set-event <event> <data>\r\n" );
    cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
  }
  else
  {
    const uint8_t data[ 8 ] = { 0 };

    const uint16_t event = atoi( argv[ 1 ] );

    strncpy( data, argv[ 2 ], sizeof( data ) );

    if( event == DEF_LBE_REMOTE_ALARM_TEST )
    {
      set_remote_alarm_status_MCU_2( REM_ALM_TEST );
      trigger_remote_alarm( );
    }
    else if( event == DEF_LBE_REMOTE_ALARM_SILENCE )
    {
      set_remote_alarm_status_MCU_2( REM_ALM_END );
      trigger_remote_alarm( );
    }
    else if( event == DEF_LBE_SMOKE_REMOTE_ALARM )
    {
      set_remote_alarm_status_MCU_2( REM_ALM_SMOKE );
      trigger_remote_alarm( );
    }
    else if( event == DEF_LBE_HEAT_REMOTE_ALARM )
    {
      set_remote_alarm_status_MCU_2( REM_ALM_HEAT );
      trigger_remote_alarm( );
    }
    else if( event == DEF_LBE_CO_REMOTE_ALARM )
    {
      set_remote_alarm_status_MCU_2( REM_ALM_CO );
      trigger_remote_alarm( );
    }
    else if( event == DEF_LBE_REMOTE_ALARM_RECEIVED )
    {
      /* Not supported */
    }
    else
    {
      DataLogging_SetEventLogbookRecord( event, data );

      SPIComms_Send_Data_to_MCU2( SPI_CMD_Logbook_value );
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_ctune( int argc, char **argv )
{
  bool ok = false;

  uint32_t value = 0ul;

  const bool opt_help = isOption( argc, argv, "-?" );

  if( opt_help )
  {
    cmdLineIf.printFunc("Set ctune value\r\n" );
    cmdLineIf.printFunc("set-ctune {opts} <[value]>\r\n" );
    cmdLineIf.printFunc("Set and/or display tuning value. When no arguments are provided\r\n" );
    cmdLineIf.printFunc("the current ctune value is read from FLASH and displayed.\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  -s: Save ctune to FLASH\r\n" );
    cmdLineIf.printFunc("  -t: Tune oscillator\r\n" );
    cmdLineIf.printFunc("  -?: Print this help information\r\n" );
  }
  else if( argc < 2 )
  {
    ok = ct_get_tuning_value( &value );

    if( ok )
    {
      cmdLineIf.printFunc("ctune = %d [0x%X]\r\n", value, value );
    }
  }
  else
  {
    const int arg = get_next_arg( argc, argv, 0 );

    if( arg > 0 )
    {
      value = atoi( argv[ arg ] );

      const bool opt_save = isOption( argc, argv, "-s" );
      const bool opt_tune = isOption( argc, argv, "-t" );

      if( opt_save )
      {
        ok = ct_set_tuning_value( value );

        if( ok )
        {
          uint32_t value2;

          ok = ct_get_tuning_value( &value2 );

          if( ok && ( value == value2 ) )
          {
            cmdLineIf.printFunc("ctune = %d [0x%X]\r\n", value, value );

            cmdLineIf.printFunc("Saved to FLASH\r\n" );
          }
          else
          {
            cmdLineIf.printFunc("Error saving to FLASH\r\n" );
          }
        }
      }

      if( opt_tune )
      {
        ok = ct_tune( value );

        if( ok )
        {
          cmdLineIf.printFunc("ctune = %d [0x%X]\r\n", value, value );

          cmdLineIf.printFunc("Oscillator tuned\r\n" );
        }
        else
        {
          cmdLineIf.printFunc("Error tuning Oscillator\r\n" );
        }
      }
    }

  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_dump_flash( int argc, char **argv )
{
  bool ok = false;

  uint32_t address  = USERDATA_BASE;
  uint32_t count    = 256ul;
  
  const bool opt_help = isOption( argc, argv, "-?" );

  if( opt_help )
  {
    cmdLineIf.printFunc("Dump FLASH\r\n" );
    cmdLineIf.printFunc("dump-flash {[address]} {[count]}\r\n" );
    cmdLineIf.printFunc("Displays the contents of the internal flash from address.\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  address: Address of internal FLASH [0x%X]\r\n", address );
    cmdLineIf.printFunc("    count: Number of bytes to display [%d]\r\n", count );
    cmdLineIf.printFunc("       -?: Print this help information\r\n" );
  }
  else if( argc == 2 )
  {
    address = strtol( argv[ 1 ], NULL, 16 );
  }
  else if( argc == 3 )
  {
    address = strtol( argv[ 1 ], NULL, 16 );
    count   = atoi( argv[ 2 ] );
  }
  else
  {
  }

  const uint8_t * ptr = ( uint8_t * )address;

  for( uint32_t i = 0; i < count; )
  {
    cmdLineIf.printFunc("%8X ", address + i );

    for( int j = 0; ( j < 16 ) && ( i < count ); j++, i++ )
    {
      uint8_t byte = ptr[ i ];

      cmdLineIf.printFunc("%02X ", byte );
    }

    cmdLineIf.printFunc("\r\n" );
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_erase_flash( int argc, char **argv )
{
  const bool opt_help = isOption( argc, argv, "-?" );

  if( opt_help )
  {
    cmdLineIf.printFunc("Erase FLASH\r\n" );
    cmdLineIf.printFunc("erase-flash {[address]} {[count]}\r\n" );
    cmdLineIf.printFunc("Erase information page in internal FLASH.\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  -?: Print this help information\r\n" );
  }
  else
  {
    const bool ok = uip_erase( );

    if( ok )
    {
      cmdLineIf.printFunc("Information page erased\r\n" );
    }
    else
    {
      cmdLineIf.printFunc("Error, information cannot be erased\r\n" );
    }
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_set_abuf_calib( int argc, char **argv )
{
    if( argc < 3 )
    {
      cmdLineIf.printFunc("Set ABUF caclibration values\r\n" );
      cmdLineIf.printFunc("set-abuf-calib <offset> <gain>\r\n" );
      cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
    }
    else
    {
      const uint16_t offset   = atoi( argv[ 1 ] );
      const uint16_t gain     = atoi( argv[ 2 ] );
      
      dl_abuf_cfg_data_t cfg;

      cfg.offset  = offset;
      cfg.gain    = gain;

      DataLogging_SetAbufConfig( &cfg );

      DataLogging_SetCRC( );

      CLI_get_abuf_calib( 0, NULL );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_get_abuf_calib( int argc, char **argv )
{
  dl_abuf_cfg_data_t cfg;

  DataLogging_GetAbufConfig( &cfg );

  cmdLineIf.printFunc("OFFSET = %d\r\n", cfg.offset );
  cmdLineIf.printFunc("GAIN   = %d\r\n", cfg.gain );

  return UARTCLI_OK;
}

static cliStatus_t CLI_do_buzzer_test( int argc, char **argv )
{
    const uint8_t buzzbist = runBuzzerBist( );

    if( buzzbist == 0U )
    {
      cmdLineIf.printFunc( "BIST Passed(BUzzer)\r\n" );
    }
    else
    {
      cmdLineIf.printFunc( "BIST Failed(BUzzer)\r\n" );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_get_operating_state( int argc, char **argv )
{
    const bool verbose = isVerbose( argc, argv );

    const behaviour_state_enum_operational_States state = getBehavioural_Operational_State( );

    if( verbose )
    {
      static const char * state_text[ ] = \
      {
        "Idle", "Smoke Alarm", "Smoke Alarm Silence", "Heat Alarm", "Heat Alarm Silence",
        "CO Alarm", "CO Alarm Silence", "Remote Alarm", "BIST Mode", "Airing Configuration"
      };

      cmdLineIf.printFunc( "Operating State: %s\r\n", state_text[ state ] );
    }
    else
    {

      cmdLineIf.printFunc( "%d\r\n", state );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_set_smoke_cleanair(int argc, char **argv)
{
    uint16_t saved_reflection_threshold;

    const uint16_t smoke_value = get_smoke_level();

    DataLogging_SetSmokeCalRefThreshold(smoke_value);

    saved_reflection_threshold = DataLogging_GetSmokeCalRefThreshold();

    if(smoke_value != saved_reflection_threshold)
    {
        cmdLineIf.printFunc("%s\r\n", "Smoke reflection (clean air) threshold EEPROM write failed.");
    }
    else
    {
        if(isVerbose(argc, argv))
        {
            cmdLineIf.printFunc("%s\r\n", "Smoke reflection (clean air) threshold is set to:");
            cmdLineIf.printFunc("%d\r\n", smoke_value);
            cmdLineIf.printFunc("Command executed successfully\r\n");
         }
        else
        {
            cmdLineIf.printFunc("%d\r\n", smoke_value);
        }

        DataLogging_SetCRC( );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_set_smoke_cal(int argc, char **argv)
{
    uint16_t saved_reflection_threshold;
    uint16_t saved_cal_threshold;
    uint16_t smoke_cal_threshold;


    const uint16_t smoke_value = get_smoke_level();

    saved_reflection_threshold = DataLogging_GetSmokeCalRefThreshold();
    smoke_cal_threshold = smoke_value - saved_reflection_threshold;

    DataLogging_SetSmokeCalThreshold(smoke_cal_threshold);

    saved_cal_threshold = DataLogging_GetSmokeCalThreshold();

    if(smoke_cal_threshold != saved_cal_threshold)
    {
        cmdLineIf.printFunc("%s\r\n", "Smoke calibration threshold EEPROM write failed.");
    }
    else
    {
        if(isVerbose(argc, argv))
        {
            cmdLineIf.printFunc("%s\r\n", "smoke calibration threshold is set to:");
            cmdLineIf.printFunc("%d\r\n", smoke_cal_threshold);
            cmdLineIf.printFunc("Command executed successfully\r\n");
         }
        else
        {
            cmdLineIf.printFunc("%d\r\n", smoke_cal_threshold);
        }

        DataLogging_SetCRC( );
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_GetCurrentSystemFault(int argc, char **argv)
{
    uint32_t current_system_fault = FaultHandler_GetFaultFlags();
    if(isVerbose(argc, argv))
    {
        if( 0 != current_system_fault)  //If System Fault is there.
        {
            cmdLineIf.printFunc("The system has registered following Faults:\r\n");
            for(int i = 0; i < (sizeof(faultStringMap)/sizeof(faultStringMap[0])); i++ )
            {
                if((current_system_fault & faultStringMap[i].bitMap) != 0)
                {
                    cmdLineIf.printFunc("%s     %s", faultStringMap[i].StringVal, (i < DegradedSmokeChamberFault)?(" (Major Fault)\r\n"):(" (Minor Fault)\r\n"));
                }
            }
            cmdLineIf.printFunc("Command executed successfully\r\n");
        }
        else
        {
            cmdLineIf.printFunc("No Fault\r\n");
        }
    }
    else
    {
        cmdLineIf.printFunc("%d\r\n", current_system_fault);
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_SetCurrentSystemFault(int argc, char **argv)
{
    uint32_t faultToInject = 0;
    if(argc > 1)
    {
        for(int i = 1; i < argc; i++)
        {
            if(0 == strncmp(argv[i], "-h", 2))
            {
                cmdLineIf.printFunc("System Faults:\r\n");
                for(int j = 0; j < (sizeof(faultStringMap)/sizeof(faultStringMap[0])); j++)
                {
                    cmdLineIf.printFunc("%d  (%s)\r\n", j, faultStringMap[j].StringVal);
                }
                break;
            }
            else
            {
                char *token = strtok(argv[1], ",");
                while(NULL != token)
                {
                    faultToInject =  strtol(token, NULL, 10);
                    FaultHandler_FaultSet(faultToInject);
                    token = strtok(NULL, ",");
                }
                break;
            }
        }
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_ClrCurrentSystemFault(int argc, char **argv)
{
    uint32_t faultToClear = 0;
    if(argc > 1)
    {
        for(int i = 1; i < argc; i++)
        {
            if(0 == strncmp(argv[i], "-h", 2))
            {
                cmdLineIf.printFunc("System Faults that can be cleared:\r\n");  /* Only minor faults can be cleared */
                for(int j = minorFaultStartIdx; j < (sizeof(faultStringMap)/sizeof(faultStringMap[0])); j++)
                {
                    cmdLineIf.printFunc("%d  (%s)\r\n", j, faultStringMap[j].StringVal);
                }
                break;
            }
            else
            {
                char *token = strtok(argv[1], ",");
                while(NULL != token)
                {
                    faultToClear =  strtol(token, NULL, 10);
                    if(faultToClear > minorFaultStartIdx)
                    {
                        FaultHandler_FaultClear(faultToClear);
                    }
                    else
                    {
                        cmdLineIf.printFunc("%d is a Major Fault\r\n", faultToClear);
                    }
                    token = strtok(NULL, ",");
                }
                break;
            }
        }
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_start_heat_simulation(int argc, char **argv)
{
    if(argc >1)
    {
        bool simulatedMode = (bool)strtol(argv[1], NULL, 10);
        SetSimulatedHeatMode(simulatedMode);
    }
    else
    {
        cmdLineIf.printFunc("Please pass a value of (0/1) to set/reset simulated heat mode\r\n");
    }
}

static cliStatus_t CLI_inject_heat_value(int argc, char **argv)
{
    if(argc > 1) 
    {
        uint32_t simulatedHeat = (uint32_t)strtol(argv[1], NULL, 10);
        InjectCurrentHeatValue(simulatedHeat);
    }
    else
    {
        cmdLineIf.printFunc("Please pass a heat value in degree centegrade scaled by 10\r\n");
    }
}

/* @Note Fill up the other SPI Command types*/
static cliStatus_t CLI_get_spi_msg(int argc, char **argv)
{
    INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIMsg;
    
    if(argc >= 2)
    {
        uint8_t index = strtol(argv[1], NULL, 10);
        SPIMsg = SPIComms_GetTxPacketElement(index-1);
        if(NULL != SPIMsg)
        {
            if(isVerbose(argc, argv))
            {
                uint8_t cmd = (SPICOMMS_COMMANDS)SPIMsg->app_data_cmd;
                switch(cmd)
                {
                    case SPICmdAlarm:
                    {
                        cmdLineIf.printFunc("SPI CMD Type: ALARM\r\n");
                        cmdLineIf.printFunc("Alarm Type: %s\r\n", AlarmTypeString[SPIMsg->app_data[0]]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdAriringLight:
                    {
                        char *airingLightState[] = {"Airing Light OFF", "Airing Light ON"};
                        cmdLineIf.printFunc("SPI CMD Type: Airing Light\r\n");
                        cmdLineIf.printFunc("Airing Light State: %s\r\n", airingLightState[SPIMsg->app_data[0]]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdFwVersion:
                    {
                        cmdLineIf.printFunc("SPI CMD Type: FW Version Number\r\n");
                        cmdLineIf.printFunc("Firmware Number: %d\r\n", ((SPIMsg->app_data[0] << 8) | SPIMsg->app_data[1]));
                        cmdLineIf.printFunc("FW Revision (Major:Minor:Build/Patch): %d.%d.%d \r\n", SPIMsg->app_data[2], SPIMsg->app_data[3], SPIMsg->app_data[4]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdLogbookRecord:
                    {
                        cmdLineIf.printFunc("SPI Logbook Record\r\n");
                    }
                    break;
                    case SPICmdDemountingLogbook:
                    {
                        cmdLineIf.printFunc("SPI DemountLogbook Record\r\n");
                    }
                    break;
                    default:
                    {
                        cmdLineIf.printFunc("SPI CMD Type cannot be decoded\r\n");
                    }
                    break;
                }
            }
            else
            {
                uint8_t cmd = (SPICOMMS_COMMANDS)SPIMsg->app_data_cmd;
                cmdLineIf.printFunc("%d" ,cmd);
                for(uint16_t idx = 0; idx < SPIMsg->app_data_len; idx++)
                {
                    cmdLineIf.printFunc(",%d" ,SPIMsg->app_data[idx]);
                }
                cmdLineIf.printFunc("\r\n");
            }
        }
        else
        {
            cmdLineIf.printFunc("No SPI Comms History Found\r\n");
        }
    }
    else
    {
        cmdLineIf.printFunc("Please pass the index number of the SPI Command History (1 - 5)\r\n");
    }
}


/* @Note Fill up the other SPI Command types*/
static cliStatus_t CLI_dump_all_spi_msg(int argc, char **argv)
{
    INTER_MCU_SPI_COMMS_HISTORY_ELEMENT *SPIMsg;
    uint8_t NoOfSpiHistoryElements = SPIComms_GetTxPacketNoOfElements();
    uint8_t Index = SPIComms_GetTxPacketCurrentIndex();
    for(int i = 0; i < NoOfSpiHistoryElements; i++)
    {
        if(0 == Index)
        {
            Index = (NO_OF_SPI_HISTORY_ENTRIES - 1);
        }
        else
        {
            --Index;
        }
        SPIMsg = SPIComms_GetTxPacketElement(Index);
        if(NULL != SPIMsg)
        {
            if(isVerbose(argc, argv))
            {   
                uint8_t cmd = (SPICOMMS_COMMANDS)SPIMsg->app_data_cmd;
                if(SPIMsg->app_data_len > SPI_APP_DATA_LEN)
                {
                    cmdLineIf.printFunc("Cannot Dump SPI App Data as Length (%d) more than available buffer size\r\n", SPIMsg->app_data_len);
                }
                switch(cmd)
                {
                    case SPICmdAlarm:
                    {
                        cmdLineIf.printFunc("%d> SPI CMD Type: ALARM\r\n", (i+1));
                        cmdLineIf.printFunc("Alarm Type: %s\r\n", AlarmTypeString[SPIMsg->app_data[0]]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdAriringLight:
                    {
                        char *airingLightState[] = {"Airing Light OFF", "Airing Light ON"};
                        cmdLineIf.printFunc("%d> SPI CMD Type: Airing Light\r\n",(i+1));
                        cmdLineIf.printFunc("Airing Light State: %s\r\n", airingLightState[SPIMsg->app_data[0]]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdFwVersion:
                    {
                        cmdLineIf.printFunc("%d> SPI CMD Type: FW Version Number\r\n", (i+1));
                        cmdLineIf.printFunc("Firmware Number: %d\r\n", ((SPIMsg->app_data[0] << 8) | SPIMsg->app_data[1]));
                        cmdLineIf.printFunc("FW Revision (Major:Minor:Build/Patch): %d.%d.%d \r\n", SPIMsg->app_data[2], SPIMsg->app_data[3], SPIMsg->app_data[4]);
                        cmdLineIf.printFunc("timestamp: %d \r\n", SPIMsg->timestamp);
                    }
                    break;
                    case SPICmdLogbookRecord:
                    {
                       cmdLineIf.printFunc("SPI Logbook Record\r\n");
                    }
                    break;
                    case SPICmdTrigOCDetection:
                    {
                       cmdLineIf.printFunc("SPI Laser Detection\r\n");
                       cmdLineIf.printFunc("%d.%d.%d \r\n", SPIMsg->app_data[0], SPIMsg->app_data[1], SPIMsg->app_data[2]);
                    }
                    break;
                    default:
                    {
                        cmdLineIf.printFunc("%d> SPI CMD Type cannot be decoded. Dumping raw data\r\n", (i+1));
                        //TX
                        uint8_t cmd = (SPICOMMS_COMMANDS)SPIMsg->app_data_cmd;
                        cmdLineIf.printFunc("%d" ,cmd);
                        for(uint16_t idx = 0; idx < SPIMsg->app_data_len; idx++)
                        {
                            cmdLineIf.printFunc(",%d" ,SPIMsg->app_data[idx]);
                        }
                        cmdLineIf.printFunc(",%d" ,SPIMsg->timestamp);
                        cmdLineIf.printFunc("\r\n");
                    }
                    break;
                }
                cmdLineIf.printFunc("%d> SPI Response\r\n", (i+1));
                cmdLineIf.printFunc("Rx Payload Len: %d\r\n", SPIMsg->rx_data[0]);
                cmdLineIf.printFunc("Link Layer Status: ");
                switch(SPIMsg->rx_data[1])
                {
                    case NoLinkError:
                        cmdLineIf.printFunc("%s\r\n", LinkErrStatus[0]);
                    break;
                    case LengthViolation:
                        cmdLineIf.printFunc("%s\r\n", LinkErrStatus[1]);
                    break;
                    case CRCViolation:
                        cmdLineIf.printFunc("%s\r\n", LinkErrStatus[2]);
                    break;
                    case EFViolation:
                        cmdLineIf.printFunc("%s\r\n", LinkErrStatus[3]);
                    break;
                    case BFViolation:
                        cmdLineIf.printFunc("%s\r\n", LinkErrStatus[4]);
                    break;
                    default:
                        break;
                };
                uint16_t crc_value = SPIMsg->rx_data[3] << 8;
                crc_value |= SPIMsg->rx_data[2];
                cmdLineIf.printFunc("CRC: 0x%04X\r\n", crc_value);
                cmdLineIf.printFunc("RX Payload: ");
                for(int idx = 0; idx < SPIMsg->rx_data[0]; idx++)
                {
                    cmdLineIf.printFunc("%d ", SPIMsg->rx_data[4+idx]);
                }
                cmdLineIf.printFunc("\r\n");
            }
            else
            {   
                //TX
                uint8_t cmd = (SPICOMMS_COMMANDS)SPIMsg->app_data_cmd;
                cmdLineIf.printFunc("%d" ,cmd);
                for(uint16_t idx = 0; idx < SPIMsg->app_data_len; idx++)
                {
                    cmdLineIf.printFunc(",%d" ,SPIMsg->app_data[idx]);
                }
                cmdLineIf.printFunc(",%d" ,SPIMsg->timestamp);
                cmdLineIf.printFunc("\r\n");
                //RX
                cmdLineIf.printFunc("%d" ,SPIMsg->rx_data[0]);
                for(uint16_t idx = 1; idx < (SPIMsg->rx_data[0] + 4); idx++)
                {
                    cmdLineIf.printFunc(",%d" ,SPIMsg->rx_data[idx]);
                }
                cmdLineIf.printFunc(",%d" ,SPIMsg->timestamp);
                cmdLineIf.printFunc("\r\n");
            }
        }
        else
        {
            cmdLineIf.printFunc("No SPI Comms History Found\r\n");
        }
    }
}

static cliStatus_t CLI_get_battery_calib(int argc, char **argv)
{
    const bool verbose = isVerbose( argc, argv );

    if( verbose )
    {
      cmdLineIf.printFunc("Low Battery Threshold  = %u\r\n", DataLogging_GetLowBatteryThreshold());
      cmdLineIf.printFunc("Dead Battery Threshold = %u\r\n", DataLogging_GetDeadBatteryThreshold());
      cmdLineIf.printFunc("Bist strike count      = %u\r\n", DataLogging_GetBatteryBistStrikeCount());
    }
    else
    {
      cmdLineIf.printFunc("BATTLOWTHRESH   = %u\r\n", DataLogging_GetLowBatteryThreshold());
      cmdLineIf.printFunc("BATTDEADTHRESH  = %u\r\n", DataLogging_GetDeadBatteryThreshold());
      cmdLineIf.printFunc("BATTSTRIKECOUNT = %u\r\n", DataLogging_GetBatteryBistStrikeCount());
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_set_battery_calib(int argc, char **argv)
{
    if(argc < 3 )
    {
      cmdLineIf.printFunc("Set battery calibration values into EEPROM\r\n" );
      cmdLineIf.printFunc("set-battery-calib <low battery Threshold> <dead battery threshold > <Bist strike count>\r\n" );
      cmdLineIf.printFunc("Enter all values as decimal, do not use HEX numbers\r\n" );
    }
    else
    {
        uint16_t lowBattThres = 0U;
        uint16_t deadBattThres = 0U;
        uint8_t strikeBattCount = 0U;

        lowBattThres = (uint16_t)atoi( argv[ 1 ]);
        deadBattThres = (uint16_t)atoi( argv[ 2 ]);
        strikeBattCount = (uint8_t)atoi( argv[ 3 ]);

        DataLogging_SetLowBatteryThreshold(lowBattThres);
        DataLogging_SetDeadBatteryThreshold(deadBattThres);
        DataLogging_SetBatteryBistStrikeCount(strikeBattCount);
        
        DataLogging_SetCRC( );

        CLI_get_battery_calib( argc, argv );
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_start_smoke_simulation(int argc, char **argv)
{
    if(argc >1)
    {
        bool simulatedMode = (bool)strtol(argv[1], NULL, 10);
        SetSimulatedSmokeMode(simulatedMode);
    }
    else
    {
        cmdLineIf.printFunc("Please pass a value of (0/1) to set/reset simulated smoke mode\r\n");
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_get_super_smoke_threshold(int argc, char **argv)
{
    uint16_t superSmokeThreshold = getSuperSmokeThreshold();
    cmdLineIf.printFunc("%d\r\n", superSmokeThreshold);
    return UARTCLI_OK;
}

static cliStatus_t CLI_get_smoke_threshold(int argc, char **argv)
{
    uint16_t smokeThreshold = getSmokeThreshold();
    cmdLineIf.printFunc("%d\r\n", smokeThreshold);
    return UARTCLI_OK;
}

static cliStatus_t CLI_inject_smoke_value(int argc, char **argv)
{
    if(argc > 1) 
    {
        uint32_t simulatedSmoke = (uint32_t)strtol(argv[1], NULL, 10);
        InjectCurrentSmokeValue(simulatedSmoke);
    }
    else
    {
        cmdLineIf.printFunc("Please pass a smoke value in xxxx (??)\r\n");
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_inject_smoke_threshold(int argc, char **argv)
{
    if(argc > 1) 
    {
        uint32_t simulatedSmokeThreshold = (uint32_t)strtol(argv[1], NULL, 10);
        InjectCurrentSmokeThreshold(simulatedSmokeThreshold);
    }
    else
    {
        cmdLineIf.printFunc("Please pass a smoke threshold value in xxxx (Units ??)\r\n");
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_read_eeprom_data(int argc, char **argv)
{
    if( argc > 2 ) 
    {
        uint16_t startAddress           = strtol( argv[ 1 ], NULL, 10 );
        uint16_t totalNoOfBytes         = strtol( argv[ 2 ], NULL, 10 );
        uint16_t noOfBytesToRead        = 0;

        I2C_BusAcquire(I2CPower_EEPROM, EEPROM_I2C_POWERUP); /* Acquire I2C Bus */

        while(true)
        {
            noOfBytesToRead = (totalNoOfBytes > MAX_EEPROM_READ_BUF_LEN)?(MAX_EEPROM_READ_BUF_LEN):(totalNoOfBytes);
            if(EEPROM_Read(eepromReadBytes, startAddress, noOfBytesToRead))
            {
                for(uint16_t idx = 0; idx < noOfBytesToRead; idx++)
                {
                    if(isVerbose(argc, argv))
                    {
                        if(((idx % 16) == 0) && (idx != 0))
                        {
                            cmdLineIf.printFunc("\r\n");
                        }
                    }
                    cmdLineIf.printFunc("0x%02X ", eepromReadBytes[idx]);
                }
                if(isVerbose(argc, argv))
                {
                    cmdLineIf.printFunc("\r\n");
                }
            }
            if(totalNoOfBytes < MAX_EEPROM_READ_BUF_LEN)
            {
                break;
            }
            else
            {
                startAddress += MAX_EEPROM_READ_BUF_LEN;
                totalNoOfBytes -= MAX_EEPROM_READ_BUF_LEN;
            }
        }

        cmdLineIf.printFunc("\r\n");
        I2C_BusRelease(); /* Release I2C Bus */
    }
    else
    {
        cmdLineIf.printFunc("Please pass the start address and length of bytes to be read.\r\n");
    }

    return UARTCLI_OK;
}

static cliStatus_t CLI_get_eeprom_index(int argc, char **argv)
{
    const char *IndexTypeString[] = 
    {
        "Main Logbook Index",
        "Demounting Index",
        "Smoke Events Index",
        "Co Events Index",
        "Heat Events Index",
        "Faults Events Index",
        "Battery Level Index",
        "Battery Impedence Index"
    };

    if(argc > 1)
    {
        if(0 == strncmp(argv[1], "-h", 2))
        {
            cmdLineIf.printFunc("Available Index\r\n");
            for(uint8_t idx = 0; idx < sizeof(IndexTypeString)/sizeof(IndexTypeString[0]); idx++)
            {
                cmdLineIf.printFunc("%d %s\r\n", (idx+1), IndexTypeString[idx]);
            }
        }
        else
        {
            uint8_t indexType = strtol(argv[1], NULL, 10);
            uint16_t index = 0;
            switch(indexType)
            {
                case 1:
                {
                    index = DataLogging_GetMainLogbookIndex();
                }
                break;
                case 2:
                {
                    index = DataLogging_GetDemountingIndex();
                }
                break;
                case 3:
                {
                    index = DataLogging_GetSmokeEventsIndex();
                }
                break;
                case 4:
                {
                    index = DataLogging_GetCOEventsIndex();
                }
                break;
                case 5:
                {
                    index = DataLogging_GetHeatEventsIndex();
                }
                break;
                case 6:
                {
                    index = DataLogging_GetFaultsEventsIndex();
                }
                break;
                case 7:
                {
                    index = DataLogging_GetBatteryLevelIndex();
                }
                break;
                case 8:
                {
                    index = DataLogging_GetBatteryImpedanceIndex();
                }
                break;
                default:
                {
                    cmdLineIf.printFunc("Please choose the right Index type\r\n");
                }
                break;
            }
            if(index > 0)
            {
                cmdLineIf.printFunc("%s: %d\r\n", IndexTypeString[indexType-1], index);
            }
        }   
    }
    return UARTCLI_OK;
}



static cliStatus_t CLI_GetBehaviorialOperationalState(int argc, char **argv)
{
    /*@Note: Take care to sync the strings if values added or removed */
    const char *const verbose_string[] = 
    {
        "IDLE",
        "SMOKE ALARM",
        "SMOKE ALARM SILENCE",
        "HEAT_ALARM",
        "HEAT ALARM SILENCE",
        "CO ALARM",
        "CO ALARM SILENCE",
        "REMOTE ALARM",
        "BIST MODE",
        "AIRING_CONFIGURATION"
    };

    behaviour_state_enum_System_modes current_system_state = getBehavioural_Operational_State();
    if(isVerbose(argc, argv))
    {
        cmdLineIf.printFunc("%s\r\n", verbose_string[current_system_state]);
        cmdLineIf.printFunc("Command executed successfully\r\n");
    }
    else
    {
        cmdLineIf.printFunc("%d\r\n", current_system_state);
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_eeprom_erase(int argc, char **argv)
{
  const bool ok = DataLoggingErase(0u);

  if( ok )
  {
    cmdLineIf.printFunc("EEPROM erased\r\n");
  }
  else
  {
    cmdLineIf.printFunc("Failed to erase EEPROM\r\n");
  }

  return UARTCLI_OK;
}

static cliStatus_t CLI_set_eeprom_crc(int argc, char **argv)
{
  const uint16_t crc_current = DataLogging_GetCRC( );

  DataLogging_SetCRC( );

  const uint16_t crc_new = DataLogging_GetCRC( );

  cmdLineIf.printFunc("EEPROM CRC CURRENT = 0x%04X\r\n", crc_current);
  cmdLineIf.printFunc("EEPROM CRC NEW     = 0x%04X\r\n", crc_new);

  return UARTCLI_OK;
}

static cliStatus_t CLI_get_Temperature(int argc, char **argv)
{
  //(void)argc;
  //(void)argv;
  sht4x_multiple_read(COCAL_NUM_OF_SENSOR_READS);
  int16_t currTemp = sht_get_temperature();
  currTemp = currTemp/100;
  if(isVerbose(argc, argv))
    {
      cmdLineIf.printFunc("Temperature %d C\r\n", currTemp);
    }
  else
    {
      cmdLineIf.printFunc("%d\r\n", currTemp);
    }
  return UARTCLI_OK;

}

static cliStatus_t CLI_ReadAllSensors(int argc, char **argv)
{
  sht4x_multiple_read(COCAL_NUM_OF_SENSOR_READS);

  const int16_t   currTemp                = sht_get_temperature() / 100;
  const uint32_t  humidity                = sht_get_humidity();
  const uint16_t  CO_sensor_test_value    = getRawCo();

  if(isVerbose(argc, argv))
  {
      cmdLineIf.printFunc("Temperature %d C\r\n", currTemp);
      cmdLineIf.printFunc("Humidity %d%%\r\n", humidity);
      cmdLineIf.printFunc("Raw CO: %d \r\n", CO_sensor_test_value);

      if(((currTemp*100) < COCAL_TEMP_MIN )&&((currTemp*100) < COCAL_TEMP_MAX))
        {
          if((currTemp*100) < COCAL_TEMP_MIN )
            {
              cmdLineIf.printFunc("Temperature is below CO calibration min threshold of %d\r\n", COCAL_TEMP_MIN);

            }
          else
            {
              cmdLineIf.printFunc("Temperature is above max threshold of %d\r\n", COCAL_TEMP_MAX);
            }
          cmdLineIf.printFunc("Set Temperature below 27C and above 15C before starting calibration \r\n");
        }
      if((humidity < COCAL_HUMIDITY_MIN )&&(humidity < COCAL_HUMIDITY_MAX))
        {
          if(humidity < COCAL_HUMIDITY_MIN )
            {
              cmdLineIf.printFunc("Humidity is below min threshold of %d\r\n", COCAL_HUMIDITY_MIN);

            }
          else
            {
              cmdLineIf.printFunc("Humidity is above max threshold of %d\r\n", COCAL_HUMIDITY_MAX);
            }
          cmdLineIf.printFunc("Set Humidity below 61 and above 34 before starting calibration \r\n");
        }
    }
  else
  {

      cmdLineIf.printFunc("TEMPERATURE = %d\r\n", currTemp);
      cmdLineIf.printFunc("HUMIDITY    = %d\r\n", humidity);
      cmdLineIf.printFunc("RAWCO       = %d\r\n", CO_sensor_test_value);

      //!cmdLineIf.printFunc("%d,%d,%d,\r\n", currTemp,humidity,CO_sensor_test_value);
  }

  return UARTCLI_OK;
}

static cliStatus_t CLI_get_current_value(int argc, char **argv)
{
    Current_values read_data;
    uint32_t fault_val = FaultHandler_GetFaultFlags ();
    //Get the current Values
    get_value_for_SPI(getBehavioural_System_Modes(false), fault_val,&read_data);
    if(isVerbose(argc, argv))
    {
        cmdLineIf.printFunc("Temp: %d\r\n", read_data.temp_val);
        cmdLineIf.printFunc("Batt A: %d\r\n", read_data.battA_val);
        cmdLineIf.printFunc("Batt B: %d\r\n", read_data.battB_val);
        cmdLineIf.printFunc("CO: %d\r\n", read_data.Co_val);
        cmdLineIf.printFunc("Heat: %d\r\n", read_data.Heat_val);
        cmdLineIf.printFunc("Smoke: %d\r\n", read_data.Smoke_val);
        cmdLineIf.printFunc("Degraded Chamber: %d\r\n", read_data.Degraded_chamber_val);
        cmdLineIf.printFunc("Coverage Status: %d\r\n", read_data.Coverage_Status);
        cmdLineIf.printFunc("Obs Status: %d \r\n", read_data.Obs_Status);
        cmdLineIf.printFunc("Brightness: %d\r\n", read_data.brightness);
        cmdLineIf.printFunc("Humidity: %d\r\n", read_data.Humidity_val);
    }
    else
    {
        cmdLineIf.printFunc("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", read_data.temp_val,read_data.battA_val,
        read_data.battB_val,read_data.Co_val,read_data.Heat_val,read_data.Smoke_val,read_data.Degraded_chamber_val,
        read_data.Coverage_Status,read_data.Obs_Status,read_data.brightness,read_data.Humidity_val);
    }
    return UARTCLI_OK;
}

static cliStatus_t CLI_SetBehaviorialSystemMode(int argc, char **argv)
{
  if(argc < 2 )
  {
    cmdLineIf.printFunc("Set System Behavioural System Mode\r\n" );
    cmdLineIf.printFunc("0 = Standby_Mode, 1 = Commissioning_Mode, 2 = Operational_Mode, 3 = Functional_Test_Mode, 4 = Transport_Mode, 5 = Shutdown_Mode\r\n" );
    cmdLineIf.printFunc("if system mode changed to Standby, commissioning, transport or shutdown mode, then power cycle device for normal operation\r\n" );

  }
  else
  {
      uint8_t sysMode = 0U;
      bool pValid = true;
      sysMode = (uint8_t)atoi( argv[ 1 ]);

      switch(sysMode)
      {
          case 0U:
            setBehavioural_System_Modes(Standby_Mode);
          break;
          case 1U:
            setBehavioural_System_Modes(Commisioning_Mode);
          break;
          case 2U:
            setBehavioural_System_Modes(Operational_Mode);
          break;
          case 3U:
            setBehavioural_System_Modes(Functional_Test_Mode);
          break;
          case 4U:
            setBehavioural_System_Modes(Transport_Mode);
          break;
          case 5U:
            setBehavioural_System_Modes(Shutdown_Mode);
          break;
          default:
            pValid = false;
            cmdLineIf.printFunc("Invalid Argument Passed\r\n");
          break;
      }

      if(pValid)
      {

          if(sysMode == getBehavioural_System_Modes(true))
          {
            cmdLineIf.printFunc("Command executed successfully\r\n");
          }
          else
          {
              cmdLineIf.printFunc("Failed to executed command\r\n");
          }
      }
  }
  return UARTCLI_OK;
}

cliStatus_t CLI_SetMountEvent(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  recordMountEvent();
  cmdLineIf.printFunc("Command executed successfully\r\n");
  return UARTCLI_OK;
}

cliStatus_t CLI_SetDeMountEvent(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  recordDemountEvent();
  cmdLineIf.printFunc("Command executed successfully\r\n");
  return UARTCLI_OK;
}

cliStatus_t CLI_SendFwVersionToMcu2(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  SPIComms_Send_Data_to_MCU2(SPI_CMD_Firmware_version);
  cmdLineIf.printFunc("Command executed successfully\r\n");
  return UARTCLI_OK;
}

static cliStatus_t CLI_set_production( int argc, char **argv )
{
  const bool opt_help = isOption( argc, argv, "-?" );

  if( opt_help )
  {
    cmdLineIf.printFunc("set-production\r\n" );
    cmdLineIf.printFunc("Set or clear production magic value.\r\n" );
    cmdLineIf.printFunc("where:\r\n" );
    cmdLineIf.printFunc("  -c: Clear value\r\n" );
    cmdLineIf.printFunc("  -s: Set value\r\n" );
    cmdLineIf.printFunc("  -?: Print this help information\r\n" );
  }
  else
  {
      const bool opt_clear  = isOption( argc, argv, "-c" );
      const bool opt_set    = isOption( argc, argv, "-s" );

      if( opt_clear )
      {
        prod_reset( );
      }
      else if( opt_set )
      {
          prod_set_value(PROD_COMP_BB);
      }
      else
      {
        ;
      }

      const uint32_t value = prod_get_value( );

      cmdLineIf.printFunc("Production Value: 0x%08X\r\n", value );
  }

  return( UARTCLI_OK );
}

static cliStatus_t CLI_get_RawLaserData(int argc, char **argv)
{
  bool paramValid = true;
  if(argc >1)
  {
      uint32_t sensorValue = (uint32_t)strtol(argv[1], NULL, 10);
      switch(sensorValue)
      {
        case 1U:
        Set_OC_Parameter(OC_Reading, 0u, 0u, 0u, 1U);
        break;
        case 2U:
        Set_OC_Parameter(OC_Reading, 0u, 0u, 0u, 2U);
        break;
        case 3U:
        Set_OC_Parameter(OC_Reading, 0u, 0u, 0u, 3U);
        break;
        default:
        paramValid = false;
        break;
      }

      if(paramValid)
      {
           set_obs_det_ftm_timeover(false);
           SPIComms_Send_Data_to_MCU2(SPI_CMD_Trig_Detection);
      }

      while( get_obs_det_ftm_timeover() == false)
      {
         ;
      }

      for (uint16_t counter = 1u, laser_data=1u; counter < 161; laser_data++, counter++)
      {
         raw_laser_data[counter] = ((laser_data_RX[laser_data*2] << 8u) | laser_data_RX[(2*laser_data) - 1]);
      }

      for(uint16_t counter = 1u; counter < 161; counter++)
      {
          cmdLineIf.printFunc("\nRawReading[%d] = 0x%X", counter, raw_laser_data[counter]);
      }
  }
  else
  {
          cmdLineIf.printFunc("Please pass a value of (0/1) to set/reset simulated heat mode\r\n");
  }

  return( UARTCLI_OK );
}


static void WriteFormatted (const char * format, ...)
{
  va_list args;
  va_start (args, format);
  vsprintf(SendResp,format, args);
  eusart_send_data_blocking(SendResp);
  va_end (args);
}

cliStatus_t UARTCLI_Init(cli_t *cli)
{
	/* Set buffer ptr to beginning of buf */
	cliBuffPtr = CliBuffer;
	cmdPending = 0;
	/* Print the CLI prompt. */
	cmdLineIf.printFunc(cliPrompt);
	return UARTCLI_OK;
}

void uartCLITask(void *arg) 
{
    RTOS_ERR err;
    OS_FLAGS flags;
    cliStatus_t status;
    cmdLineIf.cmdTbl = cliCommandTbl;
    cmdLineIf.printFunc = WriteFormatted;
    cmdLineIf.cmdCount = sizeof(cliCommandTbl)/sizeof(cliCommandTbl[0]);
    while (true) 
    {
        flags = OSFlagPend(&CommsEventFlags, /* Pointer to user-allocated event flag. */
               COMMS_UART_CLI_MSG|COMMS_UART_CLI_CMD_EXPAND, /* Flag bitmask to match. */
               0u, /* Wait indefinitely. */
               OS_OPT_PEND_FLAG_SET_ANY |
               OS_OPT_PEND_BLOCKING | /* task will block and */
               OS_OPT_PEND_FLAG_CONSUME, /* consume flags */
               NULL, /* Timestamp is not used. */
               &err);
            if(flags & COMMS_UART_CLI_MSG)
            {
                status = UARTCLI_ProcessCommand(&cmdLineIf);
                cmdLineIf.printFunc(cliPrompt);
            }
            else
            {
                cmdLineIf.printFunc("\r");
                cmdLineIf.printFunc("                                    ");
                cmdLineIf.printFunc("\r%s%s", cliPrompt, CliBuffer);
            }
    }
}

bool UARTCLI_IsCliModeActive(void)
{
    return cli_mode_active;
}

void UARTCLI_SetCliMode(bool active)
{
    RTOS_ERR err;
    if(true == active)
    {
        OSSemPend(&SerialTX_Sema, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); 
        bool dmaActive;
        while(true)
        {
            DMADRV_TransferActive(dmaTxChannel, &dmaActive);
            if (!dmaActive) 
            {
                break;
            }
            OSTimeDly(2, OS_OPT_TIME_DLY, &err);
        }
        FlushDebugMessages();
        UARTCLI_Init(&cmdLineIf);
    }
    else
    {
        bool dmaActive;
        while(true)
        {
            DMADRV_TransferActive(dmaTxChannel, &dmaActive);
            if (!dmaActive) 
            {
                break;
            }
            OSTimeDly(5, OS_OPT_TIME_DLY, &err);
        }

        OSSemPost(&SerialTX_Sema, OS_OPT_POST_NONE, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u); 
    }
    cli_mode_active = active;
}

static bool UARTCLI_ProcessShortcut(uint8_t rxdByte, uint16_t bytesCountRxd, uint16_t *expandedCmdLen)
{
    static bool cmdExpansionInProgress = false;
    static uint8_t cmdIndexStr[5] = {0}; 
    bool processedByte = false;
    //The first bytes received is a number, be ready for expansion of command
    if((rxdByte >= 48) && (rxdByte <= 57) && (bytesCountRxd < 3))
    {
        cmdIndexStr[bytesCountRxd] = rxdByte;
        cmdExpansionInProgress = true;
    }
    //Command expansion in progress, and we received a 'TAB', so now expand the command
    else if((rxdByte == 9) && (true == cmdExpansionInProgress))
    {
        cmdIndexStr[bytesCountRxd] = '\0';
        uint16_t cmdIndex = strtol(cmdIndexStr, NULL, 10);
        uint16_t noOfCmds = sizeof(cliCommandTbl)/sizeof(cliCommandTbl[0]);
        if((cmdIndex > 0) && (cmdIndex <= noOfCmds))
        {
            uint16_t cmdLen = strlen(cliCommandTbl[(cmdIndex - 1)].cmd);
            memset(CliBuffer, 0, CLI_BUFFER_LEN);
            memcpy(CliBuffer, cliCommandTbl[(cmdIndex - 1)].cmd, cmdLen);
            cliBuffPtr = CliBuffer + cmdLen;
            *expandedCmdLen = cmdLen;
            processedByte = true;
        }
        cmdExpansionInProgress = false;
    }
    return processedByte;
}

void UARTCLI_RxData(uint8_t rxByte)
{
    static uint16_t bytes_received = 0;
    uint16_t expandedCmdLen = 0;
	switch(rxByte) 
    {
        case CMD_TERMINATOR:
            if(!cmdPending) 
            {
                RTOS_ERR err;
                *cliBuffPtr = '\0';     
                strcpy(CmdBuffer, CliBuffer); 
                cliBuffPtr = CliBuffer; 
                (void) OSFlagPost(&CommsEventFlags, 
                          COMMS_UART_CLI_MSG, OS_OPT_POST_FLAG_SET,    
                          &err);
                APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
                bytes_received = 0;
                cmdPending = true;
            }
            break;

        case '\b':
            /* Backspace. Delete character. */
            if(cliBuffPtr > CliBuffer)
                cliBuffPtr--;
            break;
        
        default:
            if(UARTCLI_ProcessShortcut(rxByte, bytes_received, &expandedCmdLen))
            { 
                RTOS_ERR err;
                (void) OSFlagPost(&CommsEventFlags, 
                          COMMS_UART_CLI_CMD_EXPAND, OS_OPT_POST_FLAG_SET,    
                          &err);
                APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1u);
            }
            /* Normal character received, add to buffer. */
            else
            {
                if((cliBuffPtr - CliBuffer) < (uint8_t)CLI_BUFFER_LEN)
                {
                    *cliBuffPtr++ = rxByte;
                    bytes_received++;   //Increment no of bytes received
                }
            }
            break;
	}
}

cliStatus_t UARTCLI_ProcessCommand(cli_t *cli)
{
    cliStatus_t retStatus = UARTCLI_ERR_CMD_NOT_FOUND;
    bool cmdFound = false;
    if(!cmdPending)
    {
        retStatus = UARTCLI_IDLE;
    }
    else
    {
        uint16_t buflen = strlen(CmdBuffer);
        if(buflen > 0)
        {
            uint8_t argc = 0;
            char *argv[10];

            /* Get the first token (cmd name) */
            argv[argc] = strtok(CmdBuffer, " ");

            /* Walk through the other tokens (parameters) */
            while((argv[argc] != NULL) && (argc < 10)) 
            {
                argv[++argc] = strtok(NULL, " ");
            }

            /* Search the command table for a matching command, using argv[0]
            * which is the command name. */
            for(uint16_t idx = 0; idx < cli->cmdCount; idx++) 
            {
                if(strcmp(argv[0], cli->cmdTbl[idx].cmd) == 0) 
                {
                    /* Found a match, execute the associated function. */
                    if((argc-1) <= cli->cmdTbl[idx].maxNoOfParams)
                    {
                        retStatus = cli->cmdTbl[idx].handler(argc, argv);
                    }
                    /* Very basic protection against Number Of arguments passed*/
                    else
                    {
                        cmdLineIf.printFunc("Incorrect number of arguments passed. Type help for details\r\n");
                    }
                    cmdFound = true;
                }
            }
            if(false == cmdFound)
            {
                cmdLineIf.printFunc(cliUnrecog);
            }
        }
    }
    cmdPending = 0;
    return retStatus;
}
#endif
