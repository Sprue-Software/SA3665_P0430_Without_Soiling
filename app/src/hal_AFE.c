/*********************************************************************************
 * @file  hal_AFE.c
 * @brief Handle the AFE & ADC configuration
 * @project SA3345 P200 Firmware
 * @date  20 Apr 2022
 * @author  NDI
 *******************************************************************************/

#include <string.h>
#include "hal_AFE.h"
#include "em_iadc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "comms_handler.h"
#include "hal_gpio.h"
#include "ecode.h"
#include "spi_comms.h"
#include "hal_LETimer.h"
#include "board.h"
#include "battery_measurement.h"
#include "fault_handler.h"
#include "data_logging.h"
#include "v3sctrl.h"
#include "os.h"
#include "ustimer.h"
#include "ambient_light.h"
#include "led_buzzer.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/**
 * @brief Value representing a gain of 1
 * 
 * @note The calibration gain value stored in the EEPROM is x1000
*/
#define ABUF_GAIN_1 (1000)

/**
 * @brief Maximum allowed value of offset in mV
*/
#define ABUF_OFFSET_MAX (3)

/******************ADC Configuration*******************************************/
/* Set CLK_ADC to 10 MHz*/
#define CLK_SRC_ADC_FREQ        20000000  /* CLK_SRC_ADC */
#define CLK_ADC_FREQ            20000000  /* CLK_ADC - 10 MHz max in normal mode*/
/*#define GPIO_ADC 1*/
/* PG23 Eval Kit */
#ifdef GPIO_ADC
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortAPin5;
#define IADC_INPUT_0_BUS          ABUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0
#endif
/******************ADC Configuration END*******************************************/

#define TOTAL_PARASITIC_RESISTANCE  (0.31)//310u /*(0.31 ohm) total parasitics resistance */
#define FULL_SCALE_ADC_VOLTAGE    (2420u)  /* 2.42V */
#define AFE_ABUF_VOLT_AFTER_GAIN  (20u) /* ABUF gain(0.5x) / 100mA of battery loaded current measurement */
#define ADC_RESOLUTION        (16u)
#define BATTERY_ADC_FAULT     (0x34E4u) /* Battery circuit fault threshold in ADC counts */

/**
 * @brief Register 4 retry count
 * 
 * Number of times register 4 needs to be read to obtain pass or fail status
 * 
 * @note This matches the value used in the MCU1 DVT firmware
 */
#define BIST_HORN_REG4_RETRY_COUNT          (1)

/**
 * @brief Register 4 pass/fail status bit
 */
#define BIST_HORN_REG4_TEST_PASSED_BIT      (1<<7)

/**
 * @brief Start/stop test delay
 */
#define BIST_HORN_TEST_DELAY                (50)      /* ms */

/**
 * @brief Delay between running horn tests
 */
#define BIST_HORN_INTER_TEST_DELAY          (100)     /* ms */

#define BUZZ_BIST_RETRY                     (3U)

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
static buzzerfault data_Fault;
static Co_fault_state co_fault_type;
static uint8_t reg = 0x00u;
static uint32_t Co_ADC_Reading = 0x00u;
static uint32_t Co_ref_ADC_Reading = 0x00u;
static bool Co_shunt=false;
static bool buzzer_off;
static volatile IADC_Result_t sample;
static OS_MUTEX ADC_Mutex;/*Mutex for  for ADC*/
static OS_MUTEX AFE_Mutex;/*Mutex for  for AFE*/

static bool v3s_state;

static dl_abuf_cfg_data_t abuf_config;

/*******************************************************************************/

OS_TCB AFETasktcb;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

/********** AFE & ADC functions  ************/
void Stop_ADC(void);
void Enable_SetHigh(void);
void Enable_SetLow(void);
/********** Smoke ************/
void hal_AFE_Smoke_70us_Integration(SmokeResult *result);
/********** Horn ************/
uint8_t hal_AFE_HornFaultTest(void);
void hal_AFE_Buzzer_Configuration(Buzzer_State state);
/********** HeartBeat ******/
void hal_AFE_HeartBeatOn(bool onoff);
/********** CO ************/
uint32_t hal_AFE_readCO(void);
uint32_t hal_AFE_readCO_ref(void);
void hal_AFE_COSensortest(void);
void EnableCO(void);
void hal_AFE_DisableCO(void);
/********** Heat ************/
void TSinCTL_SetHigh(void);
void TSinCTL_SetLow(void);
uint32_t hal_AFE_ThermistorRead(void);
/********* Battery Functions****/
void BAT_A_LOAD_EN_SetLow(void);
void BAT_A_LOAD_EN_SetHigh(void);
void BAT_B_LOAD_EN_SetLow(void);
void BAT_B_LOAD_EN_SetHigh(void);
/******************************************************************************/

typedef struct
{
    OS_TCB *sourcetaskTCB;
    AFE_setup_t SetupCmd;
    uint8_t *bist_result;
    SmokeResult *smoke_result;
}AFEReqMessage_t;

typedef struct 
{
    OS_TCB *taskTcb;
    AFERspMessage_t *response;
    bool used;
} ResponseData_t;

static ResponseData_t responseData[8] = {0};  //SK: Need to #define the array size

/**
 * @brief Configure_AFE:
 * @param AFE_setup_t setup Configuration: this parameter will decide which module (e.g co, smoke, buzzer ) need to test/read
 * @param input: "bist_result", this parameter will return  BIST result, if pass =1  else fail=0 respectively
 * @param input: SmokeResult, This will return Smoke result,
 * @req PTR-554 MCU > Digital Interface > Features
 */
static AFEReqMessage_t AFEMessage;
static OS_TCB *currentTaskTCB = NULL;

/*******************************************************************************
 * @brief   Set AFE POR value
 *
 * @details This function configures the AFE chip to POR values
 *
 * @see     SPIComms_AcquireBus() SPIComms_WriteAFE()
 */
void reset_POR( void )
 {
  /* Register 0 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_0, 0x00u );
  
  /* Register 1 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_1, 0x00u );
  
  /* Register 2 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_2, 0x00u );
  
  /* Register 3 */
  //SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_3, 0x00u );
  
  /* Register 4 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_4, 0x00u );
  
  /* Register 5 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_5, 0x11u );
  
  /* Register 6 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_6, 0x00u );
  
  /* Register 7 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_7, 0x00u );
  
  /* Register 8 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_8, 0x00u );
  
  /* Register 9 */
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_9, 0x77u );
}

/*******************************************************************************
 * @brief   Get ABUF calibration
 *
 * @details This function reads the ABUF calibration values from EEPROM
 *
 * @see     dl_abuf_cfg_data_t
 */
static void getABUFConfig( dl_abuf_cfg_data_t * const config )
{
  memset( config, 0xFF, sizeof( dl_abuf_cfg_data_t ) );

  /* Get eeprom integrity? */
  const bool eeprom_ok = data_logging_is_eeprom_ok( );

  /* Can eeprom be trusted? */
  if( eeprom_ok )
  {
    DataLogging_GetAbufConfig( config );
  }

  if( ( config->gain == 0x0 ) || ( config->gain == 0xFFFF ) )
  {
    config->gain = ABUF_GAIN_1;         /* No Gain */
  }

  if( config->offset == 0xFFFF )
  {
    config->offset = 0u;                /* No offset */
  }

  if( config->offset > ABUF_OFFSET_MAX )
  {
    config->offset = ABUF_OFFSET_MAX;   /* Max allowed offset */
  }

  DEBUG_AFE("\nAFE Offset:", true, config->offset);
  DEBUG_AFE("\nAFE Gain:  ", true, config->gain);
}

void hal_AFE_Post(AFE_setup_t setup, uint8_t *bist_result, SmokeResult *result, bool isHighPrio) {
    RTOS_ERR err;
    OS_OPT MessageOption = OS_OPT_POST_NONE;
    OSMutexPend(&AFE_Mutex, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    currentTaskTCB = (OS_TCB *)OSTaskRegGet(DEF_NULL, 0, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    if(NULL != bist_result)
    {
        *bist_result = 0;
    }
    if(NULL != result)
    {
        memset(result, 0,sizeof(SmokeResult));
    }

    memset(&AFEMessage, 0, sizeof(AFEMessage));

    AFEMessage.SetupCmd = setup;
    AFEMessage.bist_result = bist_result;
    AFEMessage.smoke_result = result;
    AFEMessage.sourcetaskTCB = currentTaskTCB;

    if(true == isHighPrio)
    {
        MessageOption = OS_OPT_POST_LIFO;
    }
    else
    {
        MessageOption = OS_OPT_POST_FIFO;
    }
    OSTaskQPost(&AFETasktcb, &AFEMessage, sizeof(AFEMessage), MessageOption, &err );
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    OSMutexPost(&AFE_Mutex, OS_OPT_POST_NONE, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

bool hal_AFE_RegisterResponseVar(AFERspMessage_t *response)
{
    RTOS_ERR err;
    bool retVal = false;
    if(NULL != response)
    {
        for(int cnt = 0; cnt < 8; cnt++)
        {
            if(false == responseData[cnt].used)
            {
                responseData[cnt].taskTcb = (OS_TCB *)OSTaskRegGet(DEF_NULL, 0, &err);
                APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
                responseData[cnt].response = response;
                responseData[cnt].used = true;
                retVal = true;
                break;
            }
        }
    }
    return retVal;
}


static AFE_setup_t setup;
static uint32_t afe_adc_data = 0u;
static uint32_t ADC_data_ref = 0u;
static AFEReqMessage_t *ReqMessage = NULL;

void AFE_Task(void *arg) {
    RTOS_ERR err;

    OSTaskRegSet(DEF_NULL, 0, &AFETasktcb, &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    OS_MSG_SIZE size;

    //Initialize the AFE Hardware
    hal_AFE_init();

    while(1)
    {
        afe_adc_data = 0u;
        ADC_data_ref = 0u;
        ReqMessage = NULL;

        ReqMessage = (AFEReqMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

        AFERspMessage_t *response = NULL;

        for(int cnt = 0; cnt < 8; cnt++)
        {
            if(ReqMessage->sourcetaskTCB == responseData[cnt].taskTcb)
            {
                response = responseData[cnt].response;
                break;
            }
        }

        if(NULL != ReqMessage)
        {
            setup = ReqMessage->SetupCmd;

            switch (setup) {
            case setup_batteryVoltage:
            case setup_batteryImpedance:
                hal_AFE_ADC_Setup(setup);
                battery_measure(); /* measure dual battery voltage and impedance */
				Stop_ADC();
                break;
            case setup_lightSensor:
                hal_AFE_ADC_Setup(setup_lightSensor);
                break;
            case setup_SmokeTest:
                hal_AFE_ADC_Setup(setup_SmokeTest);
                /*Configure AFE for Smoke*/
                hal_AFE_POR( );
                hal_AFE_Smoke_70us_Integration(ReqMessage->smoke_result);
                break;
            case setup_Co_High_gain:
                hal_AFE_ADC_Setup(setup_Co_High_gain);
                /*Configure AFE*/
                Enable_SetLow();
                ADC_data_ref = hal_AFE_readCO_ref();
                afe_adc_data = hal_AFE_readCO();
                Stop_ADC();
                DEBUG_AFE("CO ref :", true, ADC_data_ref);
                DEBUG_AFE(" CO Raw :", true, afe_adc_data);
                if (afe_adc_data <= ADC_data_ref)
                {
                    afe_adc_data = 0u;
                }
                else
                {
                    afe_adc_data = afe_adc_data - ADC_data_ref;
                }
                response->afe_adc_data = afe_adc_data;
                break;
            case setup_Co_Low_gain:
                hal_AFE_ADC_Setup(setup_Co_Low_gain);
				        /*Configure AFE*/
                ADC_data_ref = hal_AFE_readCO_ref();
                afe_adc_data = hal_AFE_readCO();
                Stop_ADC();
                if (afe_adc_data <= ADC_data_ref)
                {
                    afe_adc_data = 0u;
                }
                else
                {
                    afe_adc_data = afe_adc_data - ADC_data_ref;
                }
                response->afe_adc_data = afe_adc_data;
                break;
            case setup_CoSensorTest:
                hal_AFE_ADC_Setup(setup_CoSensorTest);
                /*Configure AFE*/
                Enable_SetLow();
                hal_AFE_COSensortest();
                Stop_ADC();

                break;
            case setup_thermistor:
                hal_AFE_ADC_Setup(setup_thermistor);
                response->afe_adc_data = hal_AFE_ThermistorRead();
                Stop_ADC();
                DEBUG_AFE("\nAFE thermistor:", true, Co_ADC_Reading);
                break;
            case setup_Buzzer_2_wire_init: /* 2 wire  Buzzer */
                hal_AFE_Buzzer_Configuration(Buzzer_2_wire_init);
                buzzer_off = false;
                break;
            case setup_Buzzer_2_wire_on:/* 2 wire  Buzzer on */
                hal_AFE_Buzzer_Configuration(Buzzer_2_wire_on);
                buzzer_off = false;
                break;
            case setup_Buzzer_2_wire_oFF:/* 2 wire  Buzzer off */
                hal_AFE_Buzzer_Configuration(Buzzer_2_wire_off);
                buzzer_off = false;
                break;
            case setup_Buzzer_2_wire_deinit:/* 2 wire deinit  Buzzer */
                hal_AFE_Buzzer_Configuration(Buzzer_2_wire_deinit);
                buzzer_off = true;
                break;
            case setup_Buzzer_3_wire_init:/* 3 wire init  Buzzer */
                Enable_SetLow();
                hal_AFE_POR( );
                hal_AFE_Buzzer_Configuration(Buzzer_3_wire_init);
                break;
            case setup_Buzzer_3_wire_deinit:/* 3 wire deinit  Buzzer */
                hal_AFE_Buzzer_Configuration(Buzzer_3_wire_deinit);
                break;
            case setup_Buzzer_3_wire_on:/* 3 wire  Buzzer */
                hal_AFE_Buzzer_Configuration(Buzzer_3_wire_on);
                break;
            case setup_Buzzer_3_wire_oFF:/* 3 wire  Buzzer */
                hal_AFE_Buzzer_Configuration(Buzzer_3_wire_off);
                break;
            case setup_BuzerTest:
                /*Configure AFE*/
                if(NULL != ReqMessage->bist_result)
                {
                    *ReqMessage->bist_result = hal_AFE_HornFaultTest();
                }
                break;
            case setup_Heartbeat:
                hal_AFE_HeartBeatOn(true);
                LETimer_delay_ms(4u);
                hal_AFE_HeartBeatOn(false);
                break;
            case setup_FW_TEST_Adc_GPIO_SOIL_A:
                hal_AFE_ADC_Setup(setup_FW_TEST_Adc_GPIO_SOIL_A);

                break;
            case setup_FW_TEST_Adc_GPIO_SOIL_B:
                hal_AFE_ADC_Setup(setup_FW_TEST_Adc_GPIO_SOIL_B);

                break;
            case setup_FW_TEST_Adc_GPIO_LIGHT:
                hal_AFE_ADC_Setup(setup_FW_TEST_Adc_GPIO_LIGHT);

                break;
            case setup_powerLEDOn:
                hal_AFE_HeartBeatOn(true);
                break;
            case setup_powerLEDOff:
                hal_AFE_HeartBeatOn(false);
                break;
            default:
                /*Unknown setup type*/
                break;
            }
        }
        OSTaskQPost(ReqMessage->sourcetaskTCB, response, sizeof(AFERspMessage_t), OS_OPT_POST_FIFO, &err);
    }
}


/**
 *@brief Measure_ADC: Read the ADC value
 *@details: This function will read the ADC value depend on the ADC configuration set
 * @return ADC Count
 */
uint32_t Measure_ADC(void) {

  uint32_t status;

  uint32_t timer = 100000U;

  static uint8_t faultCount = 0u;

  IADC_command(IADC0, iadcCmdStartSingle); /* Start single conversion */

  do
  {
    status = IADC0->STATUS;
    timer--;
  }
  while( ( ( status & ( _IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SINGLEFIFODV_MASK ) ) != IADC_STATUS_SINGLEFIFODV ) && ( timer > 0U ) );

  if (timer > 0U) /*Check for successful conversion*/
  {
    sample = (IADC_Result_t) IADC_pullSingleFifoResult(IADC0);
    faultCount = 0U; /*Reset fault count*/
  }
  else {
    sample.data = 0U; /*A value of 0 indicates ADC failure*/
    faultCount++;
  }
  return sample.data;
}

/**
 * @brief hal_AFE_ADC_Setup: Setup the ADC peripheral for measurement of default AFE modules
 * @deatils: All ADC setups are mentioned in hardware firmware interface document
 * @param setup Configuration: To specify AFE-Modules
 * @return NULL
 */
void hal_AFE_ADC_Setup(AFE_setup_t setup) {
  /* Default ADC Settings*/
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;
  /* Enable IADC clock*/
  CMU_ClockEnable(cmuClock_IADC0, true);
  /* Reset IADC to reset configuration in case it has been modified*/
  IADC_reset(IADC0);
  /* Configure IADC clock source*/
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);
  init.warmup = iadcWarmupKeepWarm;
  /* Set the HFSCLK prescale value here*/
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);
  /* Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate*/
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
  CLK_ADC_FREQ, 0, iadcCfgModeNormal, init.srcClkPrescale);
  initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
  /* Configure Input sources for single ended conversion*/
#define ADC1 1
#ifdef ADC1
  /* Assign pins to positive and negative inputs in single-ended mode*/
  initSingleInput.posInput = (uint32_t) (iadcPosInputPadAna0 | 1u); /* connects to AIN1 */
  initSingleInput.negInput = iadcNegInputGnd;
#endif
#ifdef ADC0
      /* Assign pins to positive and negative inputs in single-ended mode*/
      initSingleInput.posInput = iadcPosInputPadAna0 ; /* connects to AIN0*/
      initSingleInput.negInput = iadcNegInputGnd;
#endif

  switch (setup) {
  case setup_batteryVoltage:
  case setup_batteryImpedance:
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16;//iadcDigitalAverage2; /* take average of 2 samples, just to reduce conversion time ~10us*/
    initSingle.alignment = iadcAlignRight16; /*use the 16-bit resolution*/
    break;
  case setup_lightSensor: /* Need to check which AIN0 Or AN1 */
    initSingleInput.posInput = iadcPosInputPadAna0; /* connects to AIN0 */
    initSingleInput.negInput = iadcNegInputGnd;
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight16; /*use the 16-bit resolution*/
    break;
  case setup_Co_High_gain:
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage2; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight12;                /*use the 12-bit resolution*/
    break;
  case setup_Co_Low_gain:
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight16;                 /*use the 16-bit resolution*/
    break;
  case setup_CoSensorTest:
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight12; /*use the 16-bit resolution*/
    break;
  case setup_thermistor:
#ifdef ADC0
    initSingleInput.posInput = iadcPosInputPadAna0;  /*connects to AIN0*/
    initSingleInput.negInput = iadcNegInputGnd;
#endif
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight16; /*use the 16-bit resolution*/
    break;
  case setup_SmokeTest:
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight16; /*use the 16-bit resolution*/
    break;
  case setup_FW_TEST_Adc_GPIO_LIGHT:
    initSingleInput.posInput = IADC_INPUT_0_PORT_PIN_LIGHT;
    initSingleInput.negInput = iadcNegInputGnd;
    GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain0P5x;
    initAllConfigs.configs[0].adcMode = iadcCfgModeNormal;
    initAllConfigs.configs[0].digAvg = iadcDigitalAverage16; /* take average of 16 samples now*/
    initSingle.alignment = iadcAlignRight16; /*use the 16-bit resolution*/
    break;

  default:
    /*Unknown setup type*/
    break;
  }

  IADC_init(IADC0, &init, &initAllConfigs);
  /* Initialize Single ended*/
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

}

/**
 * @brief hal_AFE_HeartBeatOn
 * @details: This function will switch on & off AFE- heartbeat/Power LED
 * @param onoff =true  LED on else off
 * @return NULL
 * @req PTR-1465
 */
void hal_AFE_HeartBeatOn(bool onoff) {
  RTOS_ERR err_led;
  SPIComms_AcquireBus();
  if (onoff == true) {

    if(ambient_light_get_status() == AMBIENT_LEVEL_DARKNESS)
    {
        /*Low Boost Enable and low voltage for darkness */
        SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1,(uint8_t) AFE_REG_1_LBE);
        SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, (uint8_t)AFE_REG_5_LOW_BOOST_VOLTAGE_SETTINGS | AFE_REG_5_RDNOW);
    }
    else
    {
        /*set High Boost Enable 10 v*/
        SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1,(uint8_t) AFE_REG_1_HBE);
        SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, (uint8_t)AFE_REG_5_HIGH_BOOST_VOLTAGE_SETTINGS_10_0V | AFE_REG_5_RDNOW);
    }

    LETimer_delay_ms(6u);
    /*set RLED Enable*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2, (uint8_t)AFE_REG_2_RLED_EN);
  }
  else {
    /*Clear REG0 */
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2,(uint8_t) AFE_SET_ZERO);

    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, (uint8_t) AFE_SET_ZERO);
    /*Clear REG1 */
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1,(uint8_t) AFE_SET_ZERO);
  }
  keep_CO_powered_up();
  SPIComms_ReleaseBus();
}

/**
 * @brief hal_AFE_Buzzer_Configuration:
 * @details: This function will configure AFE for 3 wire & 2 wire mode
 * for 2 wire mode PWM frequncy will be generated by Ledbuzz Module
 * @param  buzzer state 3 wire or 2 wire configuration
 * @return NULL
 * @req PTR-1489
 *
 */
void hal_AFE_Buzzer_Configuration(Buzzer_State state) {
  reg = 0x00;
  RTOS_ERR err_buzz;
  SPIComms_AcquireBus();
  if (state == Buzzer_3_wire_init) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, (uint8_t)AFE_REG_1_HBE); /* Reg 1 - High Boost Enable */
    reg |= ((uint8_t) (((uint8_t) AFE_REG_8_HEN_CONTROL_ENABLED
        | ((uint8_t) AFE_REG_8_EN_CONTROL_ENABLED))
        & (uint8_t) (~((uint8_t) AFE_REG_8_FEED_HORN_CONTROL_FOR_2_OR_3_PIN))));
    //DEBUG_AFE("\n reg AFE", true, reg);
    //SPIComms_WriteAFE(AFE_WRITE_REGISTERS_8, 0x02);
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_8, reg); /* Reg 8 - Horn Enable */
    LETimer_delay_ms(10u);
    reg = 0u;
  }
  if (state == Buzzer_2_wire_init) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, (uint8_t)AFE_REG_1_HBE); /* Reg 1 - High Boost Enable*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, (uint8_t)AFE_REG_5_HIGH_BOOST_VOLTAGE_SETTINGS_11_5V | AFE_REG_5_RDNOW); /* Reg 5 - High Boost 11.5v*/
    reg |= ((uint8_t) (((uint8_t) AFE_REG_8_EN_CONTROL_ENABLED
        | ((uint8_t) AFE_REG_8_FEED_HORN_CONTROL_FOR_2_OR_3_PIN))
        & ~(uint8_t) ((uint8_t) AFE_REG_8_HEN_CONTROL_ENABLED)));
    //DEBUG_AFE("\n reg AFE", true, reg);
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_8, reg); /* Reg 8 - Horn Enable*/
    LETimer_delay_ms(10u);
    reg = 0u;
  }
  
  if (state == Buzzer_2_wire_on) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_4,(uint8_t) AFE_REG_4_HORN_ENABLE); /* Reg 4 - Horn Enable */
  }
  if (state == Buzzer_3_wire_on) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_4,(uint8_t) AFE_REG_4_HORN_ENABLE); /* Reg 4 - Horn Enable */
  }
  if (state == Buzzer_3_wire_off) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_4, 0u);  /* Reg 4 - Horn Enable */
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, 0u);  /* Reg 1 - High Boost Enable*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, 0u);  /* Reg 5 - High Boost 11.5v*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_8, 0u);
  }
  if (state == Buzzer_2_wire_off) {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_4, 0u);  /* Reg 4 - Horn Enable*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, 0u);  /* Reg 1 - High Boost Enable*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, 0u);  /* Reg 5 - High Boost 11.5v*/
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_8, 0u);
  }
  keep_CO_powered_up();
  SPIComms_ReleaseBus();
}

/**
 * @brief hal_AFE_Smoke_70us_Integration:
 * @details Function perform a 70us integration for Smoke
 * @param struct smoke result pointer , result will be copied in to structure
 * @return NULL
 * @req PTR-1463 PTR-1458 PTR-1356  PTR-1204 PTR-1185
 */
void hal_AFE_Smoke_70us_Integration(SmokeResult *result) {
  reg = 0x00u;
  RTOS_ERR err_smoke;
  if (result != NULL) {

    SPIComms_AcquireBus();
    hal_AFE_AcquireADC();
    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_7, AFE_REG_7_LED1_CURRENT_200MA); /* LED1 200mA */
    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_5, AFE_REG_5_LOW_BOOST_VOLTAGE_SETTINGS | AFE_REG_5_RDNOW); /* Low Boost Voltage Setting 5.2v */

    /* This need to be added in Main firmware */
    SPIComms_WriteAFE(0x02, 0x01);       //Start the low boost regulator and allow for soft start
    hal_AFE_delay_ms(4);       //soft start time 4ms typ

    SPIComms_WriteAFE(0x02, 0x05);      //Start the IRCAP charge
    hal_AFE_delay_ms(2);

    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_2, 0x03);/*enable the photo amp and sets the status to ready*/
    SPIComms_WriteAFE(0x02, 0x00); /*– turns off the boost and IRCAP charging */
    SPIComms_WriteAFE(0x0C, 0x82); /* 0x0C, 0x82 – sets up the gain and integration timing...*/
    hal_AFE_delay_us(196); /* delay of 200 uS*/

    /************* Start Photo Amp Integration for dark ********************/
    Enable_SetHigh(); /*start dark integration utilising Enable Pin */
    hal_AFE_delay_us(5); /* delay 5us*/
    Enable_SetLow();
    hal_AFE_delay_us(78); /*allow enough time for the integration with some slack 85 us*/
    /************* Stop Photo Amp Integration ********************/
    result->darkresult = Measure_ADC(); /* read the dark integration value */
    reg = 0x0;
    reg |= AFE_REG_2_PHOTO_AMP_POWER_ON;
    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_2, reg); /* reset integration capacitors - RESET State Do we need AMP Power On ? */
    hal_AFE_delay_us(9); /* Stabilisation time 10us */
    reg = 0x00;
    reg |=(AFE_REG_2_PHOTO_AMP_POWER_ON | AFE_REG_2_PHOTO_INEGRATOR_STATUS | AFE_REG_2_LED1_EN);
    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_2, reg); /* enable the photo-amp, and set the Status bit to 1, enable LED1 - READY State */
    hal_AFE_delay_us(9); /* Stabilisation time 10us*/
    /************* Start Photo Amp Integration for bright ********************/
    Enable_SetHigh();             //start Lit integration utilising Enable Pin
    hal_AFE_delay_us(5); /* Stabilisation time 5us*/
    Enable_SetLow();
    hal_AFE_delay_us(78); /* Stabilisation 85us */
    /************* Stop Photo Amp Integration ********************/
    result->brightresult = Measure_ADC(); /*read the integration value*/
    //hal_AFE_delay_ms(1); /* Note from Microchip: This time can be minimised dependent upon whether you want to take multiple samples */
    SPIComms_WriteAFE(AFE_WRITE_REGISTERS_2, AFE_SET_ZERO); /* disable the photo-amp, will put into RESET State */
    keep_CO_powered_up();
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_5, (uint8_t) AFE_SET_ZERO);
    Stop_ADC();
    hal_AFE_ReleaseADC();
    SPIComms_ReleaseBus();

    DEBUG_AFE("\n Smoke Dark reading ", true, result->darkresult);
    DEBUG_AFE("\n Smoke Bright reading ", true, result->brightresult);
  }

}

static bool perform_horn_test( const uint8_t test, uint8_t * reg )
{
  /* Test result status */
  bool passed = false;          /* Default, failed */

  /* Test must be configured with enable pin low */
  Enable_SetLow( );

  /* Write test */
  uint8_t reg4 = test | AFE_REG_4_HORN_TEST_ENABLE;
  SPIComms_WriteAFE( ( uint8_t )AFE_WRITE_REGISTERS_4, reg4 );

  /* Start the test */
  Enable_SetHigh( );

  /* Has to be driven high for this period of time */
  hal_AFE_delay_ms( BIST_HORN_TEST_DELAY );

  /* End the test */
  Enable_SetLow( );

  /* Get test result */
  reg4 = SPIComms_ReadAFE( (uint8_t )AFE_WRITE_REGISTERS_4, ( uint8_t ) AFE_DUMMY_DATA );

  /* Passed or failed */
  if ( reg4 & BIST_HORN_REG4_TEST_PASSED_BIT )
  {
    passed = true;
  }

  /* Save register for caller */
  *reg = reg4;

  /* Passed result for the caller */
  return( passed );
}

/**
 *@brief hal_AFE_HornFaultTest: This will test the diffrent type of buzzer faults
 *@details This function will be called by diagnostic module periodicaly
 * *@return test result
 *@req PTR-1454
 */
uint8_t hal_AFE_HornFaultTest( void )
{
  bool ok;

  static bool failed = false;
  static uint8_t buzz_counter = 0U;

  uint8_t reg = 0u;
  uint8_t ret = 0u;


  /* Get exclusive access */
  LEDBuzz_AcquireBuzzer( );
  hal_AFE_AcquireADC();
  SPIComms_AcquireBus( );

  /* Set AFE into known configuration */
  reset_POR( );

  /* Start the low boost regulator and allow for soft start */
  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, AFE_REG_1_LBE );

  /* Soft start time 4ms typ, plus 25% for margin */
  hal_AFE_delay_ms( 5u );

  /* Start the IRCAP charge */
  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, AFE_REG_1_IRCAP_POWER | AFE_REG_1_LBE );

  hal_AFE_delay_ms( 20u );

  /* Stop charging IRCAP */
  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, 0x00u );

  /* Another delay */
  hal_AFE_delay_ms( BIST_HORN_TEST_DELAY );

  /* Reset fault codes */
  data_Fault.FB_short_to_VDD                                          = false;
  data_Fault.FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS   = false;
  data_Fault.HS_open_or_shorted_to_VSS                                = false;
  data_Fault.HS_short_to_VDD                                          = false;
  data_Fault.FB_to_HB_short                                           = false;
  data_Fault.HB_short_to_VDD                                          = false;

  /*********************************Test 1 ****************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_1, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 1: [%02X,%02X] Pass FB short to VDD\r\n", AFE_REG_4_HORN_TEST_1, reg );
    data_Fault.FB_short_to_VDD = false;
  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 1: [%02X,%02X] Fail FB short to VDD\r\n", AFE_REG_4_HORN_TEST_1, reg );
    ret |= ( uint8_t )( 1u << 0 );
    data_Fault.FB_short_to_VDD = true;
  }

  hal_AFE_delay_ms( BIST_HORN_INTER_TEST_DELAY );

  /******************************TEST 2*************************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_2, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 2: [%02X,%02X] Pass FB open, HB open, FB short to VSS or HS, HB short to VSS or HS\r\n", AFE_REG_4_HORN_TEST_2, reg );
    data_Fault.FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS = false;
  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 2: [%02X,%02X] Fail FB open, HB open, FB short to VSS or HS, HB short to VSS or HS\r\n", AFE_REG_4_HORN_TEST_2, reg );
    ret |= ( uint8_t )( 1u << 1 );
    data_Fault.FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS = true;
  }

  hal_AFE_delay_ms( BIST_HORN_INTER_TEST_DELAY );

  /******************************TEST 3*************************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_3, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 3: [%02X,%02X] Pass HS open or shorted to VSS\r\n", AFE_REG_4_HORN_TEST_3, reg );
    data_Fault.HS_open_or_shorted_to_VSS = false;

  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 3: [%02X,%02X] Fail HS open or shorted to VSS\r\n", AFE_REG_4_HORN_TEST_3, reg );
    ret |= ( uint8_t )( 1u << 2 );
    data_Fault.HS_open_or_shorted_to_VSS = true;
  }

  hal_AFE_delay_ms( BIST_HORN_INTER_TEST_DELAY );

  /******************************TEST 4 *************************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_4, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 4: [%02X,%02X] Pass HS short to VDD\r\n", AFE_REG_4_HORN_TEST_4, reg );
    data_Fault.HS_short_to_VDD = false;
  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 4: [%02X,%02X] Fail HS short to VDD\r\n", AFE_REG_4_HORN_TEST_4, reg );
    ret |= ( uint8_t )( 1u << 3 );
    data_Fault.HS_short_to_VDD = true;
  }

  hal_AFE_delay_ms( BIST_HORN_INTER_TEST_DELAY );

  /******************************TEST 5 *************************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_5, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 5: [%02X,%02X] Pass FB to HB short\r\n", AFE_REG_4_HORN_TEST_5, reg );
    data_Fault.FB_to_HB_short = false;
  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 5: [%02X,%02X] Fail FB to HB short\r\n", AFE_REG_4_HORN_TEST_5, reg );
    ret |= ( uint8_t )( 1u << 4 );
    data_Fault.FB_to_HB_short = true;
  }

  hal_AFE_delay_ms( BIST_HORN_INTER_TEST_DELAY );

  /******************************TEST 6*************************************/

  ok = perform_horn_test( AFE_REG_4_HORN_TEST_6, &reg );

  if( ok )
  {
		DEBUG_AFE_PRINTF( "Test 6: [%02X,%02X] Pass HB short to VDD\r\n", AFE_REG_4_HORN_TEST_6, reg );
    data_Fault.HB_short_to_VDD = false;
  }
  else
  {
		DEBUG_AFE_PRINTF( "Test 6: [%02X,%02X] Fail HB short to VDD\r\n", AFE_REG_4_HORN_TEST_6, reg );
    ret |= ( uint8_t )( 1u << 5 );
    data_Fault.HB_short_to_VDD = true;
  }

  SPIComms_WriteAFE( (uint8_t )AFE_WRITE_REGISTERS_4, ( uint8_t ) AFE_SET_ZERO );

  keep_CO_powered_up( );

  /* Relase now */
  SPIComms_ReleaseBus( );
  hal_AFE_ReleaseADC( );
  LEDBuzz_ReleaseBuzzer( );

  if(ret != 0U )
  {
  	  if(failed != true)
  	  {
  	  	  buzz_counter++;
          if((buzz_counter >= BUZZ_BIST_RETRY))
      	  {
          		FaultHandler_FaultSet( BuzzerHwFault );
          		DataLogging_SetEventLogbookRecord( DEF_LBE_BUZZER_CHECK_HW_ERR_START, NULL );
          		failed = true;
          }
      }
  }
  else
  {
      buzz_counter = 0U;
  }
  return( ret );
}


void keep_CO_powered_up(void){
  uint8_t reg_value = 0U;
  reg_value |= (uint8_t) ((AFE_REG_3_CO_AMP_POWER_ON));
  reg_value |= (uint8_t)(AFE_REG_3_CO_REF_POWER_ON);
  if (Co_shunt == true)
  {
     reg_value |=  AFE_REG_3_CO_SHUNT_ENABLED;
     //DEBUG_AFE("CO Poison switch on", false, 0u);
  }
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, reg_value);
}

/**
 * @brief AFE_init: initialise the AFE semaphore & AFE default configuration
 *@details: This function shall be called by App.c
 *@params: none
 *@return none
 * @req PTR-1084, PTR-1070
 */
void hal_AFE_init( void )
{
    RTOS_ERR err;

    OSMutexCreate(&ADC_Mutex, "ADC mutex", &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    OSMutexCreate(&AFE_Mutex, "AFE mutex", &err);
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    /* Reset AFE */
    reset_POR( );

    /* Enable CO Amp Permanently  on */
    keep_CO_powered_up();

    /* charge the IRCAP for 20ms */
    hal_IRCAP_Charge(20u);

    buzzer_off = true;

    /* Configure ABUF compensation */
    getABUFConfig( &abuf_config );
}

void hal_IRCAP_Charge(uint16_t charge_time) {

  DEBUG_AFE("\nCharging IRCAP", false, 0u);

  /* Initialization of the IRCAP charging for 20ms during the power on time only */
  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, 0x01u );        /* Start the low boost regulator and allow for soft start */

  hal_AFE_delay_ms( 5u );                                   /* soft start time 4ms typ, plus 25% for margin */

  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, 0x05u );        /* Start the IRCAP charge */

  hal_AFE_delay_ms( charge_time );                          /* Only during the startup charge the IR cap for 20ms */

  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_1, 0x00u );        /* Stop charging IRCAPP */

  SPIComms_WriteAFE( AFE_WRITE_REGISTERS_9, 0x04u );        /* Smoke LED1 temp compensation 0.5%/C */
}

/**
 * @brief Temperature test, AFE thermister temperature measurement
 *@param NULL
 *@return thermistor ADC value
 *@req PTR-1400
 */
uint32_t hal_AFE_ThermistorRead(void) {
  RTOS_ERR err_temp;
  uint32_t temprature_reading = 0u;
  TSinCTL_SetLow();
  SPIComms_AcquireBus();
  hal_AFE_AcquireADC();
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0,(uint8_t) AFE_REG_0_ABUF_TEMP_SENSOR_INPUT); /*Register 0 - Set ABUF to the TSin Pin */

  TSinCTL_SetHigh(); /* Power the temperature NTC divider */
  hal_AFE_delay_us(100u);
  temprature_reading = Measure_ADC();/* read the scaled loaded temp sensor voltage */
  TSinCTL_SetLow();/* Remove power from the temperature NTC divider */
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO);/*Register 0 - Set ABUF back to zero*/
  keep_CO_powered_up();

  hal_AFE_ReleaseADC();
  SPIComms_ReleaseBus();
  return temprature_reading;
}

/**
 * @brief EnableCO: Setup and turn on the CO sensor
 **@param NULL
 *@return  NULL
 */
void EnableCO(void) {
  /* This function will enable  CO*/
//  RTOS_ERR err_co;
  // uint8_t data_read = 0u;
  //SPIComms_AcquireBus();
  reg = 0x00u;/* reset reg 0 */
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_REG_0_ABUF_CO_AMP_OUTPUT); /* Reg 0 - Set ABUF to the CO sensor */
  reg |= (uint8_t) ((AFE_REG_3_CO_AMP_POWER_ON) | (AFE_REG_3_CO_REF_POWER_ON));
 /*Enabled The Shunt to save sensor test */
  if (Co_shunt==true)
    {
      reg |=  AFE_REG_3_CO_SHUNT_ENABLED;
    }

  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, reg); /*Register 3 – enable the CO amp and the CO ref */
  hal_AFE_delay_ms(3u); /*wait 1ms for startup */
  //data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  //data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  //DEBUG_AFE("readC ",true, data_read);
  //SPIComms_ReleaseBus();
}

/**
 * @brief EnableCO: Setup and turn on the CO sensor
 **@param NULL
 *@return  NULL
 */
void Read_CO_ref (void)
{
  /* This function will enable  CO*/
  // RTOS_ERR err_co;
  //SPIComms_AcquireBus();
  reg = 0x00u;/* reset reg 0 */
  SPIComms_WriteAFE ((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_REG_0_ABUF_CO_0_3_V_REF_OUTPUT); /* Reg 0 - Set ABUF to the CO sensor refernce output */
  keep_CO_powered_up();
  hal_AFE_delay_ms (3u); /*wait 1ms for startup */
  //SPIComms_ReleaseBus();
}

/**
 *@brief hal_AFE_DisableCO:
 *@details Disable the CO sensor  but CO Amp is on always (HW recommendation )
 *@param NULL
 *@return  NULL
 */
void hal_AFE_DisableCO(void) {
  // RTOS_ERR err_co_disable;

  /* This function will disable CO*/
  /* Enable CO Amp Permanently  on */
  SPIComms_AcquireBus();
  reg = 0x00u;/* reset reg 0 */
  reg |= (uint8_t) ((AFE_REG_3_CO_AMP_POWER_ON));
   /*Enabled The Shunt to save sensor test */
    if (Co_shunt==true)
      {
        reg |=  AFE_REG_3_CO_SHUNT_ENABLED;
      }
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, reg); /*Register 3 - Turn off CO reference*/
  hal_AFE_delay_ms(10u);/*wait 10ms for startup */
  SPIComms_ReleaseBus();

}

/**
 * @brief EnableCO_withshunt_on
 * @details Setup and turn on the CO sensor with shunt on
 *@param NULL
 *@return  NULL
 */
void EnableCO_withshunt_on(void) {
  // RTOS_ERR err_shunt;
  Co_shunt=true;
#if AFE_READ
  uint32_t data_read = 0u;
#endif
  SPIComms_AcquireBus();
  reg = 0x00u;
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0,(uint8_t) AFE_REG_0_ABUF_CO_AMP_OUTPUT); /* Reg 0 - Set ABUF to the CO sensor */
  reg |= (uint8_t) (AFE_REG_3_CO_AMP_POWER_ON | AFE_REG_3_CO_REF_POWER_ON | AFE_REG_3_CO_SHUNT_ENABLED);
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, reg); /*Register 3 – enable the CO amp and the CO ref */
  hal_AFE_delay_ms(3u); /*wait 1ms for startup */
  /*READ AFE */
#if AFE_READ
  data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  DEBUG_AFE("\r\n");
  DEBUG_AFE("CO with Shunt enabled AFE-READ REG3 0x%x \r\n", data_read);
#endif
  SPIComms_ReleaseBus();
}

/**
 * @brief DisableCO_withshunt_on
 * @details Disable the CO sensor  but CO Amp is on always (HW recommendation )
 * with shunt on
 * @param  none
 * @return none
 */
void DisableCO_withshunt_on(void) {
  RTOS_ERR err_disable_shunt;
  Co_shunt=false;
  SPIComms_AcquireBus();
  /* Enable CO Amp Permanently  on */
  reg = 0x00u;
  reg |= (uint8_t) (AFE_REG_3_CO_AMP_POWER_ON | AFE_REG_3_CO_REF_POWER_ON);
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, reg); /*Register 3 - Turn off CO reference*/
  hal_AFE_delay_ms(10u);/* delay 10 ms*/
#if AFE_READ
  data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  data_read = SPIComms_ReadAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0xaau);
  DEBUG_AFE("\r\n");
  DEBUG_AFE("END CO with Shunt enabled AFE-READ REG3 0x%x \r\n", data_read);
  DEBUG_AFE("Plz disable the shunt or go back to main menu to reset  \r\n");
#endif
  SPIComms_ReleaseBus();
}

/**
 * @brief hal_AFE_COSensortest: Setup and test the Co sensor
 * @details: This API will test CO for Open, Short, Closed condition
 * @return the pass or fail
 *@req PTR-1214 ,PTR-1216
 */
void hal_AFE_COSensortest (void)
{
   uint32_t threadshold_oc = 0u;
   uint32_t threadshold_good = 0u;
   uint32_t co_result = 0u;
   uint32_t co_reading_1 = 0u;
   uint32_t co_reading_2 = 0u;
   uint32_t co_reading_7 = 0u;
   RTOS_ERR err_os;

  /* This function will disable CO*/
  /* Enable CO Amp Permanently  on */
  SPIComms_AcquireBus();
  hal_AFE_AcquireADC();
  co_fault_type=CO_Good_Sensor;
  Enable_SetLow ();
  //Enable_SetHigh (); /* Test*/
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_REG_0_ABUF_CO_AMP_OUTPUT); /* Reg 0 - Set ABUF to the CO sensor */
  USTIMER_DelayIntSafe(19u);
  SPIComms_WriteAFE ((uint8_t)AFE_WRITE_REGISTERS_3, 0x83u); //Register 3 - CO Output TriStated, sink, test OFF, CO amp and Ref on

  hal_AFE_delay_ms(10u);
  co_result = Measure_ADC(); /* ADC reading Before test*/
  threadshold_oc= ((co_result*50u)/100u);
  SPIComms_WriteAFE ((uint8_t)AFE_WRITE_REGISTERS_3, 0xA3u); //Register 3 - CO Output TriStated, sink, test ON, CO amp and Ref on

  hal_AFE_delay_ms(100u);
  co_reading_1 = Measure_ADC(); /*First 100 MS read the ADC value*/

  hal_AFE_delay_ms(100u);
  co_reading_2 = Measure_ADC(); /*Second 100 MS read the ADC value*/

  //threadshold_good= co_reading_2+((co_reading_2*15u)/100u);

  if ( (co_reading_1 <= threadshold_oc) && (co_reading_2 <= threadshold_oc))
    {
      DEBUG_AFE("\n Open Circuit: ", true, threadshold_oc);
      co_fault_type=CO_Open_Circuit;
    }
  else /*Read further to find the Good or Closed Circuit */
    {
    SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_3, 0x03u);  // Register 3 - CO Output driven, sink, test OFF, CO amp and Ref on //关闭CO测试
    hal_AFE_delay_ms(100u);
    // co_reading_4 = Measure_ADC(); /*First 100 MS read the ADC value*/
    hal_AFE_delay_ms(100u);
    // co_reading_5 = Measure_ADC();                              /*Second 100 MS read the ADC value*/    
    hal_AFE_delay_ms(100u);
    // co_reading_6 = Measure_ADC(); /*Third 100 MS read the ADC value*/
    hal_AFE_delay_ms(200u);
    co_reading_7 = Measure_ADC(); /*Third 200 MS read the ADC value*/ //500ms
    if(co_reading_7 > co_result)
    {
      threadshold_good = co_reading_7 - co_result;
    }
    else
    {
      threadshold_good = 0;
    }
    if (threadshold_good > 34)
    {
      DEBUG_AFE("\n Good Sensor ", true, threadshold_good);
      co_fault_type = CO_Good_Sensor;
    }
    else
    {
      DEBUG_AFE("\n Short Circuit Sensor ", true, threadshold_good);
      co_fault_type = CO_Closed_Circuit;
    }
  }
      SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO); /* Reg 0 - disable ABUF to the CO sensor */
      DEBUG_AFE("\n B.T", true, co_result);
      DEBUG_AFE("\n 100:", true, co_reading_1);
      DEBUG_AFE("\n 200:", true, co_reading_2);

  hal_AFE_ReleaseADC();
  SPIComms_ReleaseBus();
}

/**
 *  @brief hal_AFE_readCO:
 *  @details Read the CO sensor
 *  @return ADC value
 *  @req PTR-1208
 */
uint32_t hal_AFE_readCO(void) {

  hal_AFE_AcquireADC();
  Co_ADC_Reading = 0u;
  SPIComms_AcquireBus();
  EnableCO(); /* Enable CO */
  Co_ADC_Reading = Measure_ADC(); /* ADC reading Before test*/
  //do not disable CO
  hal_AFE_ReleaseADC();
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO);/*Register 0 - Set ABUF back to zero*/
  SPIComms_ReleaseBus();
  /* return  CO value */
  return Co_ADC_Reading;
}

/**
 *  @brief hal_AFE_readCO:
 *  @details Read the CO sensor
 *  @return ADC value
 *  @req PTR-1208
 */
uint32_t hal_AFE_readCO_ref (void)
{

  hal_AFE_AcquireADC();
  Co_ref_ADC_Reading = 0u;
  SPIComms_AcquireBus();
  Read_CO_ref ();
  Co_ref_ADC_Reading = Measure_ADC (); /* ADC reading Before test*/
  //do not disable CO
  hal_AFE_ReleaseADC();
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO);/*Register 0 - Set ABUF back to zero*/
  SPIComms_ReleaseBus();
  /* return  CO value */
  return Co_ref_ADC_Reading;
}

/**
 * @brief battery circuit BIST, check the fault in the battery circuit switches
 * @req PTR-1246,
 * @return returns the fault status of battery circuit
 * true = faulty, false= non-faulty
 */
bool batt_circuit_bist( void )
{
  /* Get current battery status */
  bool batt_bist_fault = is_battery_circuit_fault( );
  
  /* Don't re-check if it's already faulty */
  if( !batt_bist_fault )
  {
    uint8_t strike_count = getStrikeCount();

    /* check the battery status only when battery circuit is non faulty */
    do
    {
      hal_AFE_ADC_Setup(setup_batteryVoltage);	
      batt_A_Measurement( );  /* measure the voltage and impedance of battery A */
      batt_B_Measurement( );  /* Measure the voltage and impedance of battery B */

      if( ( get_battery_A_Voltage( ) < getLowBattThres( ) ) || ( get_battery_B_Voltage( ) < getLowBattThres( ) ) )
      {
        batt_bist_fault = true;

        strike_count = strike_count - 1u;
      }
      else
      {
        batt_bist_fault = false;
      }
    }
    while( ( batt_bist_fault == true ) && ( strike_count > 0u ) );

	Stop_ADC();

    if( batt_bist_fault == true )
    {
      DEBUG_BATT("\nBattery circuit fault", false, 0u);
    
     /* Battery circuit hardware error start */#
      set_low_battery_status(true);
      LEDBuzz_Post(PatternLowBatt);
      FaultHandler_FaultSet( BatteryFault );

      /* eeprom status update */
      DataLogging_SetEventLogbookRecord( DEF_LBE_BATTERY_ERR_START, NULL );

      /* Set battery fault */
      set_battery_circuit_fault( );
    }
    else
    {
        set_low_battery_status(false);
    }
  }

  return( batt_bist_fault );
}

/**
 * desc unloaded and loaded battery measurement of BATT and impedance calculation. Impedance is a factor of 100
 * @param rxData unused parameter
 */
void batt_A_Measurement(void) {
  uint32_t batt_load_volt_A = 0x00;

  set_ADC_Battery_A(0u);
  batt_load_volt_A = 0u;
  set_Batt_A_Impedance(0u);
  uint32_t voltage_drop = 0u;
  uint32_t data_read = 0x00u;


  reg = 0x00;

  SPIComms_AcquireBus();
  reset_POR( );
  hal_AFE_AcquireADC();
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0,(uint8_t) AFE_REG_0_ABUF_SCALED_VBATLTD_MONITOR_VOLTAGE_OUTPUT); /*turn on ABUF and set to VBATLD monitor */
  reg |= (uint8_t) (AFE_REG_1_LBE | AFE_REG_1_IRCAP_POWER);
  SPIComms_WriteAFE( (uint8_t)AFE_WRITE_REGISTERS_1, reg); /* charges the IRCAP, low boost mode */
 
  
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_0, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_0, 0xaa);

  data_read = 0;
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_1, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_1, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_1, 0xaa);
  
  hal_AFE_delay_ms(20u); // Allow enough time to charge the IRCAP? This 20ms is the initial requirement to fully charge the IRCAP from a fully depleted state. Characterisation of resupplying should be undertaken to understand the recharge time requirements.
  reg = 0x00;
  /* below Line Is making difference between */
  reg |= (uint8_t) (AFE_REG_2_VBATLED_EN | AFE_REG_2_PHOTO_AMP_POWER_ON
      | AFE_REG_2_PHOTO_INEGRATOR_STATUS);
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2, reg); //turn on low battery driver, ready for low battery test

  data_read = 0;
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_2, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_2, 0xaa);

  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, (uint8_t)AFE_SET_ZERO); //turn off Boost, IRCap pull-up   -> note that this happens after turning on photo
  BAT_A_LOAD_EN_SetHigh(); //Turn off PMOS to isolate battery
  hal_AFE_delay_us(180u); //wait 120us - load switch rise time
  //Enable_SetHigh();           //begin loaded battery test
  //hal_AFE_delay_us(3u);           //hold on as long as battery test
  set_ADC_Battery_A(Measure_ADC()); /* unloaded battery measurement Batt A, take 4 ADC samples 16-bit measurement */
  //Enable_SetLow();              //end battery test
  Enable_SetHigh();               //Below low battery test
  hal_AFE_delay_us(8u);           //hold on as long as battery test
  batt_load_volt_A = Measure_ADC(); /* unloaded battery measurement Batt A, take 4 ADC samples 16-bit measurement */
  //hal_AFE_delay_us(2u);             //hold on as long as battery test
  Enable_SetLow();              //end battery test
  hal_AFE_delay_us(5);               //hold on as long as battery test
  BAT_A_LOAD_EN_SetLow();             //Turn OFF Switch-1 (Normal operation)
  Enable_SetHigh();             //Quick fall time
  hal_AFE_delay_us(5u);             //fall-time delay
  Enable_SetLow();              //Quick fall time

  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2,(uint8_t) AFE_SET_ZERO); //turn off low battery driver & associated photo circuitry
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO);/*Register 0 - Set ABUF back to zero*/
  hal_AFE_ReleaseADC();
  SPIComms_ReleaseBus();

  if (get_ADC_Battery_A() > batt_load_volt_A)
  {
    voltage_drop = get_ADC_Battery_A() - batt_load_volt_A;                                                                                                    /* drop in the voltage due to loaded battery test */
    set_Batt_A_Impedance((((uint32_t) (voltage_drop * FULL_SCALE_ADC_VOLTAGE * AFE_ABUF_VOLT_AFTER_GAIN) >> ADC_RESOLUTION) - TOTAL_PARASITIC_RESISTANCE)); /* impedance in milli ohms */
  }
  else
  {
    set_Batt_A_Impedance(0xFFFFFFFF); /*Set impedance to infinity as the difference can be negative due to noisy input*/
  }
}

/**
 * desc unloaded and loaded battery measurement of BATT B
 * @param rxData unused parameter
 */
void batt_B_Measurement(void) {
  uint32_t batt_load_volt_B = 0x00;

  set_ADC_Battery_B(0u);
  batt_load_volt_B = 0u;
  set_Batt_B_Impedance(0u);
  uint32_t voltage_drop = 0u;
  reg = 0x00;
  uint8_t data_read = 0u;

  SPIComms_AcquireBus();
  reset_POR( );
  hal_AFE_AcquireADC();

  hal_AFE_delay_us(10u);       //Add a 10-us before starting battery B

  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0,(uint8_t) AFE_REG_0_ABUF_SCALED_VBATLTD_MONITOR_VOLTAGE_OUTPUT); /*turn on ABUF and set to VBATLD monitor */
  reg |= (uint8_t) (AFE_REG_1_LBE | AFE_REG_1_IRCAP_POWER);
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, reg);

  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_0, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_0, 0xaa);

  data_read = 0;
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_1, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_1, 0xaa);

  hal_AFE_delay_ms(20);// Allow enough time to charge the IRCAP? This 20ms is the initial requirement to fully charge the IRCAP from a fully depleted state. Characterisation of resupplying should be undertaken to understand the recharge time requirements.

  reg = 0x00;
  /* below Line Is making difference between */
  reg |= (uint8_t) (AFE_REG_2_VBATLED_EN | AFE_REG_2_PHOTO_AMP_POWER_ON
      | AFE_REG_2_PHOTO_INEGRATOR_STATUS);
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2, reg); //turn on low battery driver, ready for low battery test


  data_read = 0;
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_2, 0xaa);
  data_read = SPIComms_ReadAFE(AFE_WRITE_REGISTERS_2, 0xaa);

  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_1, (uint8_t)AFE_SET_ZERO); //turn off Boost, IRCap pull-up   -> note that this happens after turning on photo

  BAT_B_LOAD_EN_SetHigh();        //Turn off PMOS to isolate battery
  hal_AFE_delay_us(180u);       //wait 120us - load switch rise time
  //Enable_SetHigh();           //begin loaded battery test
  //hal_AFE_delay_us(3u);               //hold on as long as battery test
  //batt_load_volt_B = Measure_ADC(); /* loaded battery measurement Batt B, take 4 ADC samples 16-bit measurement */
  set_ADC_Battery_B(Measure_ADC()); // unloaded battery measurement
  //Enable_SetLow();              //end battery test
  //hal_AFE_delay_us(2u);           //hold on as long as battery test
  Enable_SetHigh();           //begin loaded battery test
  hal_AFE_delay_us(8);               //hold on as long as battery test
  //set_ADC_Battery_B(Measure_ADC()); /* unloaded battery measurement Batt B, take 4 ADC samples 16-bit measurement */
  batt_load_volt_B = Measure_ADC(); // loaded battery measurement
  Enable_SetLow();              //end battery test
  hal_AFE_delay_us(5u);               //hold on as long as battery test
  BAT_B_LOAD_EN_SetLow();      //Turn OFF Switch-1 (Normal operation)
  Enable_SetHigh();           //Quick fall time
  hal_AFE_delay_us(5u);               //fall-time delay
  Enable_SetLow();     

           //Quick fall time
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_2,(uint8_t) AFE_SET_ZERO); //turn off low battery driver & associated photo circuitry
  SPIComms_WriteAFE((uint8_t)AFE_WRITE_REGISTERS_0, (uint8_t)AFE_SET_ZERO);/*Register 0 - Set ABUF back to zero*/
  hal_AFE_ReleaseADC();
  SPIComms_ReleaseBus();
  if (get_ADC_Battery_B() > batt_load_volt_B)
  {
    voltage_drop = get_ADC_Battery_B() - batt_load_volt_B;                                                                                                    /* drop in the voltage due to loaded battery test */
    set_Batt_B_Impedance((((uint32_t) (voltage_drop * FULL_SCALE_ADC_VOLTAGE * AFE_ABUF_VOLT_AFTER_GAIN) >> ADC_RESOLUTION) - TOTAL_PARASITIC_RESISTANCE));
  }
  else
  {
    set_Batt_B_Impedance(0xFFFFFFFF); /*Set impedance to infinity as the difference can be negative due to noisy input*/
  }
}

/**
 * @brief Stop_ADC:
 * @details Disable ADC for power saving
 **@params: none
 **@return none
 */
void Stop_ADC(void) {
  /* reset IADC 0*/
  IADC_reset(IADC0);
  /* Stop IADC 0*/
  CMU_ClockEnable(cmuClock_IADC0, false);
  /* Stop FSRCO*/
  CMU_ClockEnable(cmuClock_FSRCO, false);
}

/**
 * @brief hal_AFE_delay_ms
 * @details wrapper function: Delay Function in Ms
 * @param delay in ms
 *@return null
 */
void hal_AFE_delay_ms(uint16_t time_ms_val) {
  /* use LETIMER for ms delay */
  LETimer_delay_ms(time_ms_val);
  /* use LETIMER for ms delay */
}
//refernce
//5us =  5 or get_battery_A_Voltage
//85us =  78
//10 us =  9
//20 us =19
//110 us=101
//120us =110
//150us =141

/**
 *  @brief hal_AFE_delay_us
 *  @details wrapper function: Delay Function in us
 *  @param delay in us
 *  This timer is not accurate , we have use ref. values to generate required us delay
 *  toggle the GPIO using this function and get the ~delay ,
 * @return null
 */
void hal_AFE_delay_us(uint16_t time_us_val) {
  uint16_t delay = 0u;
  /* delay in us*/
  delay = time_us_val * 6u;   /* UH: Please don't change this value, smoke module is using time according to this */
  for (uint16_t time = 0u; time < delay; time++) {
  __NOP();
}
}

/**
 *  @brief Enable_SetHigh Function.
 * @details: This function will set the Enable pin High
 * @params:null
 * @return null
 */
void Enable_SetHigh(void) {
/* Set the Enable pin High */
GPIO_PinOutSet(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN);
}
/**
 *  @brief  Enable_SetLow Function.
 * @details: This function will set the Enable pin Low
 * @params:Null
 * @return null
 */
void Enable_SetLow(void) {
/* Set the Enable pin Low */
GPIO_PinOutClear(DEF_AFE_ENABLE_PORT, DEF_AFE_ENABLE_PIN);
}

/**
 *  @brief TSinCTL_SetLow Function.
 * @details: This function will set the Tsin pin High
 * @params:Null
 * @return null */
void TSinCTL_SetHigh(void) {
/* Set the Tsin pin High */
GPIO_PinOutSet(DEF_HEAT_POWER_PORT, DEF_HEAT_POWER_PIN);
}
/**
 *  @brief  TSinCTL_SetLow Function.
 * @details: This function will set the TSin pin Low
 * @params:Null
 * @return null
 */
void TSinCTL_SetLow(void) {
/* Set the Tsin pin Low */
GPIO_PinOutClear(DEF_HEAT_POWER_PORT, DEF_HEAT_POWER_PIN);
}

/**
 *  @brief  BAT_A_LOAD_EN_SetHigh Function.
 * @details: This function will set the Battery A Load pin High
 * @params:Null
 * @return null
 */
void BAT_A_LOAD_EN_SetHigh(void) {
/* Set the BAT Load A pin High */
GPIO_PinOutSet(DEF_BATTERY_LOAD_DRV_A_PORT, DEF_BATTERY_LOAD_DRV_A_PIN);
}
/**
 *  @brief  BAT_A_LOAD_EN_SetHigh Function.
 *  @details: This function will set the Battery A Load pin Low
 * @params:Null
 * @return null
 */
void BAT_A_LOAD_EN_SetLow(void) {
/* Set the BAT Load A pin LOW */
GPIO_PinOutClear(DEF_BATTERY_LOAD_DRV_A_PORT, DEF_BATTERY_LOAD_DRV_A_PIN);
}

/**
 *  @brief  BAT_B_LOAD_EN_SetHigh Function.
 * @details: This function will set the Battery B Load pin High
 * @params:Null
 * @return null
 */
void BAT_B_LOAD_EN_SetHigh(void) {
/* Set the BAT Load B pin High */
GPIO_PinOutSet(DEF_BATTERY_LOAD_DRV_B_PORT, DEF_BATTERY_LOAD_DRV_B_PIN);
}
/**
 *  @brief  BAT_B_LOAD_EN_SetLow Function.
 *  * @details: This function will set the Battery B Load pin Low
 * @params:Null
 * @return null
 */
void BAT_B_LOAD_EN_SetLow(void) {
/* Set the BAT Load B pin Low */
GPIO_PinOutClear(DEF_BATTERY_LOAD_DRV_B_PORT, DEF_BATTERY_LOAD_DRV_B_PIN);
}

/**
 *  @brief  Enable Light sense
 * @details: This function will set the Ambient Light Sense  pin High
 * @params:Null
 * @return null
 */
void enable_light_Sense(void) {
  v3s_state = V3S_ON_LOCK( );
}
/**
 *  @brief  Disable Light sense
 * @details: This function will set the Ambient Light Sense  pin Low
 * @params:Null
 * @return null
 */
void disable_light_Sense(void) {
  V3S_ON_UNLOCK( v3s_state );
}

/**
 *  @brief  Disable SOIL A & B sensor
 * @details: This function will set the Soil A & B  pin Low
 * @params:Null
 * @return null
 */
void disable_Soil_A_B(void) {
GPIO_PinOutClear(MCU1_SOILA_ENABLE_PORT, MCU1_SOILA_ENABLE_PIN);
GPIO_PinOutClear(MCU1_SOILB_ENABLE_PORT, MCU1_SOILB_ENABLE_PIN);
}
/**
 * @brief  Enable SOIL A & disable B sensor
 * @details: This function will set the Soil A   pin High
 * @params:Null
 * @return null
 */
void enable_Soil_A_Sense(void) {
/* Set the SOIL A  sense pin High */
GPIO_PinOutSet(MCU1_SOILA_ENABLE_PORT, MCU1_SOILA_ENABLE_PIN);
}

/**
 *  @brief   Enable SOIL B & disable A sensor
 *  @details: This function will set the Soil  B  pin High
 * @params:Null
 * @return null
 */
void enable_Soil_B_Sense(void) {
/* Set the SOIL B  sense pin High */
GPIO_PinOutSet(MCU1_SOILB_ENABLE_PORT, MCU1_SOILB_ENABLE_PIN);
}

/**
 * @brief Get the Buzzer Fault status & set the fault
 * @details: This function will return the Battery fault status & set the fault
 * @param  none
 * @return Battery fault
 */
buzzerfault get_the_buzzer_fault( void )
{
  return( data_Fault );
}

/**
 * @brief Set the Buzzer Fault FB_short_to_VDD
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_FB_Short (void)
{
data_Fault.FB_short_to_VDD = true;

}

/**
 * @brief Set the Buzzer Fault FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_FB_Open (void)
{
data_Fault.FB_HB_open_FB_short_to_VSS_or_HS_HB_short_to_VSS_or_HS = true;

}

/**
 * @brief Set the Buzzer Fault HS_open_or_shorted_to_VSS
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_HS_Open (void)
{
data_Fault.HS_open_or_shorted_to_VSS = true;

}

/**
 * @brief Set the Buzzer Fault HS_short_to_VDD
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_HS_Short (void)
{
data_Fault.HS_short_to_VDD = true;

}

/**
 * @brief Set the Buzzer Fault FB_to_HB_short
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_FB_HB_Short (void)
{
data_Fault.FB_to_HB_short = true;

}

/**
 * @brief Set the Buzzer Fault HB_short_to_VDD
  * @param  none
 * @return none
 */
void set_the_buzzer_fault_HB_Short_VDD (void)
{
data_Fault.HB_short_to_VDD = true;

}


/**
 * @brief Get the CO Fault status
 * @details: This function will return the CO fault status & set the fault
 * @param  none
 * @return none
 */
Co_fault_state get_the_co_fault (void)
{

return co_fault_type;
}

/**
 * @brief set the CO Open  Fault
 * @details: This function will set the fault
 * @param  true = set , false= clear
 * @return none
 */
void set_the_co_Open_circuit_fault (void)
{
  co_fault_type=CO_Open_Circuit;

}

/**
 * @brief set the CO  Closed Fault
 * @details: This function will set the fault
 * @param  true = set , false= clear
 * @return none
 */
void set_the_co_Close_circuit_fault (void)
{
  co_fault_type=CO_Closed_Circuit;

}

bool is_Buzzer_Off()
{
    return buzzer_off;
}

void hal_AFE_AcquireADC(void)
{
	RTOS_ERR err;
	OSMutexPend(&ADC_Mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

void hal_AFE_ReleaseADC(void)
{
	RTOS_ERR err;
	OSMutexPost(&ADC_Mutex, OS_OPT_POST_NONE, &err);
	APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
}

/******************************************************************************/
int32_t hal_AFE_abuf_correct( int32_t mV )
{
  if( mV > abuf_config.offset )
  {
    mV -= abuf_config.offset;

    mV *= ABUF_GAIN_1;
    
    mV /= abuf_config.gain;
  }
  else
  {
    mV = 0u;
  }

  return( mV );
}

/******************************************************************************/
void AFE_dump( void )
{
  uint8_t reg;

  /* Lock access */
  SPIComms_AcquireBus( );

  /* Register 0 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_0, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 0: %2X\r\n", reg );
  
  /* Register 1 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_1, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 1: %2X\r\n", reg );
  
  /* Register 2 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_2, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 2: %2X\r\n", reg );
  
  /* Register 3 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_3, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 3: %2X\r\n", reg );
  
  /* Register 4 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_4, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 4: %2X\r\n", reg );
  
  /* Register 5 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_5, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 5: %2X\r\n", reg );
  
  /* Register 6 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_6, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 6: %2X\r\n", reg );
  
  /* Register 7 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_7, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 7: %2X\r\n", reg );
  
  /* Register 8 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_8, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 8: %2X\r\n", reg );
  
  /* Register 9 */
  reg = SPIComms_ReadAFE( ( uint8_t )AFE_WRITE_REGISTERS_9, ( uint8_t )AFE_DUMMY_DATA );
  DEBUG_AFE_PRINTF( "AFE Register 9: %2X\r\n", reg );
  
  /* Unlock access */
  SPIComms_ReleaseBus( );
}

/******************************************************************************/
void hal_AFE_POR( void )
 {
  /* Lock access */
  SPIComms_AcquireBus( );

  /* Set POR values */
  reset_POR( );

  /* Unlock access */
  SPIComms_ReleaseBus( );
}
