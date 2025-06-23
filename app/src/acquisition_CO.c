/**
 * @file	acquisition_CO.c
 * @brief	CO acquisition
 * @date	3 Sep 2022
 * @author NDI
 *
 */
#include  <kernel/include/os.h>
#include "hal_LETimer.h"
#include "acquisition_Co.h"
#include "app.h"
#include "events.h"
#include "hal_BURTCTimer.h"
#include "hal_LETimer.h"
#include "hal_AFE.h"
#include "temp_humid.h"
#include "Variance.h"
#include "timeHandler.h"
#include "system_events.h"
#include "em_msc.h"
#include "em_gpcrc.h"
#include "em_cmu.h"
#include "sht4x.h"
#include "fault_handler.h"
#include "data_logging.h"

#include "CO_Calibration.h"




#define CRC_POLYNOMIAL 0x00001021

/**
 * @brief ADC High Gain maximum number of counts
 */
#define HIGH_GAIN_COUNTS_MAX (0xFFFul)

/**
 * @brief ADC High Gain maximum value of milli-volts
 *
 * @note This will be the vref used in the conversions
 */
#define HIGH_GAIN_MILLIVOLTS_MAX (2420ul)

/**
 * @brief ADC Low Gain maximum number of counts
 */
#define LOW_GAIN_COUNTS_MAX (0xFFFFul)

/**
 * @brief ADC Low Gain maximum value of milli-volts
 *
 * @note This will be the vref used in the conversions
 */
#define LOW_GAIN_MILLIVOLTS_MAX (1210ul)

//static bool co_periodicity_high = false;


void init_CRC_co(void);


/* logbook_data_Co_Fault : Fault type & Values ( need to update the document )
* 0x02 for Open Circuit Fault
* 0x03 for Closed Circuit Fault
* 0x04 for fatigue fault*/
uint8_t  logbook_data_Co_Fault[8u] = {0};
#define USERDATA ((uint32_t*)USERDATA_BASE)
#ifdef TEST_HUMIDTY_TABLE
uint8_t Humidity_table [20]=
{
0x69, /*5*/
0x6d,/*10*/
0x6d,/*15*/
0x6d,/*20*/
0x71,/*25*/
0x76,/*30*/
0x76,/*35*/
0x76,/*20*/
0x76,/*45*/
0x76,/*50*/
0x76,/*55*/
0x78,/*60*/
0x7d,/*65*/
0x7f,/*70*/
0x82,/*75*/
0x85,/*80*/
0x87,/*85*/
0x8a,/*90*/
0x8c,/*95*/
0x8e/*100*/
};
#endif

static const uint16_t COSENSORTCR[61] = {
    674,   // 0,-10
    685,   // 1,-9
    696,   // 2,-8
    707,   // 3,-7
    718,   // 4,-6
    729,   // 5,-5
    740,   // 6,-4
    750,   // 7,-3
    762,   // 8,-2
    773,   // 9,-1
    783,   // 10,0
    794,   // 11,1
    805,   // 12,2
    816,   // 13,3
    827,   // 14,4
    838,   // 15,5
    849,   // 16,6
    860,   // 17,7
    871,   // 18,8
    882,   // 19,9
    892,   // 20,10
    903,   // 21,11
    914,   // 22,12
    925,   // 23,13
    936,   // 24,14
    947,   // 25,15
    958,   // 26,16
    969,   // 27,17
    980,   // 28,18
    990,   // 29,19
    1000,  // 30,20
    1007,  // 31,21
    1015,  // 32,22
    1022,  // 33,23
    1030,  // 34,24
    1037,  // 35,25
    1045,  // 36,26
    1052,  // 37,27
    1060,  // 38,28
    1067,  // 39,29
    1075,  // 40,30
    1082,  // 41,31
    1090,  // 42,32
    1097,  // 43,33
    1105,  // 44,34
    1112,  // 45,35
    1120,  // 46,36
    1127,  // 47,37
    1135,  // 48,38
    1142,  // 49,39
    1150,  // 50,40
    1157,  // 51,41
    1165,  // 52,42
    1172,  // 53,43
    1180,  // 54,44
    1187,  // 55,45
    1195,  // 56,46
    1202,  // 57,47
    1210,  // 58,48
    1217,  // 59,49
    1225,  // 60,50
};

static FAULT_CIRCUIT coFault;

static bool simulate_co_raw_mode = false;

static bool shunt_switch_on = false;

static uint16_t simulated_co_raw_reading;


static uint8_t  AlarmOutCount;                 
static uint16_t HighLowTempertureTimerCount;   
static uint16_t NormalTempertureTimerCount;    
static uint16_t HighLowTempertureAvg;          
static uint16_t FHighLowTemperture;            
static uint16_t HighLowTempertureCail;         
#define STWCOD_HIGH_TEMPERTURE                  ((int16_t)3500)
#define STWCOD_LOW_TEMPERTURE                   ((int16_t)0)
#define STWCOD_HIGHLOW_TEMPERTURE_TIMER         ((uint16_t)288)      
#define STWCOD_NORMAL_TEMPERTURE_TIMER          ((uint16_t)288)      
static void HighLowTempertureCalibration(void);

static uint8_t  RepaLowHumi;                   
static uint16_t LowHumiInCount;                
static uint16_t LowHumiOutCount;               
#define STWCOD_HUMIDITY_IN_LIMLT                ((uint8_t)20)      
#define STWCOD_HUMIDITY_OUT_LIMLT               ((uint8_t)50)      
#define STWCOD_HUMIDITY_REPA_VALUE              ((uint8_t)5)       


/* This Variable will provide detail of CO Module*/
static struct
{
  uint16_t                cond_signal_zero;                   /**< Zeroed measurement */
  uint16_t                Co_Final_after_compensations;       /**< Compensated CO value*/
  uint16_t                Co_RAW_reading;                     /**< Co raw reading */
  uint16_t                co_calibrated;                      /**< Co calibrated value */
  uint16_t                COHB_Calculation;                   /**< COHB Calculation */
  uint16_t                CO_Low_Level_Peak;                  /**< CO Low level Peak Value */
  LOW_LEVEL_CO_WARNING    co_warning;                         /**< CO Low level warning Alarm Value */
  CO_LEVEL                co_level;                           /**< CO Level Value */
  OPERATING_MODE          op_mode;                            /**< Operating mode  */
  FAULT_CIRCUIT           Co_circuit_fault;                   /**< Co Circuit Fault  */
  uint8_t                 Gain;                               /**< Co gain variable  */
  uint8_t                 PWM_duty_cycle;                     /**< Co PWM duty Cycle  */
  uint8_t                 Overload_Counter;                   /**< Co Overload  Counter Value */
  uint16_t                lowGainHysterisisVal;
  uint16_t                max_Co_Alarm;                       /**< MAX Co value*/
}

CO_Status;
static struct {
		int16_t Co_intelligent_sample[7]; /**< Co intelligent sample buffer  */
		uint16_t Intelligent_SAMPLE_RATE_Count; /**< Intelligent sample rate count flag*/
		bool Is_Intelligent_SAMPLE_RATE_ENABLED; /**<  Intelligent sample rate detection   Flag*/
		bool Intelligent_SAMPLE_RATE_timer_start; /**<  Intelligent sample rate timer start  Flag*/
} CO_intelligentSampleData;
static struct
{
  uint16_t RelativeTemperature;       /**< Relative Temperature flag */
  int16_t Temperature;                /**<  Temperature Calculation */
  uint32_t Co_With_temp_compensation; /**<  Temperature Compensation value*/
} CO_temperatureData;

static struct {
		bool co_Sensor_Overload; /**<  Sensor Overload Flag*/
		bool co_Sensor_Overload_Occurred; /**<  record the overload happened in the previous read */
		bool Is_LOW_CO_Warning_enabled; /**<  Low level Warning  Flag*/
		bool Is_LOW_CO_ALARM_Active;/**<  Low level Warning Alarm Flag*/
		bool Is_SNIFF_CO_ALARM_Active; /**<  Sniff mode   Flag*/
		bool Is_CO_Detected; /**<  Co detection   Flag*/
		bool Is_CO_ALARM_Active; /**<  Co Alarm   Flag*/

		bool Low_Level_timer_start; /**<   Low level CO  timer start  Flag*/
		bool overload_Compensation; /**< Co Overload Compensation flag */
		bool offbase_co_variance; /**< Co variance offbase flag */
		bool disable_co_variance; /**< Co variance disable flag */
		bool disable_Co_overload_Compensation_timer;
		bool AMP_FAULT;
} CO_flags;

/*****************************************************************************************/
/*                               Co module features                                             */
/*****************************************************************************************/
/* New Requirement for Capturing 10 CO values in alarm conditions */
static uint16_t CO_Alarm_Values[10] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };
static uint8_t current_co_Buffer_index = 0u;

Co_fault_state sensor_health;
static uint8_t strike_count_SC = 0;
static uint8_t strike_count_OC = 0;
static uint8_t strike_out_count = 0;

static uint32_t date_of_manufacture = 0;

static uint8_t co_alarm_st = 0u;
static uint8_t co_super_alarm_st = 0u;
static uint16_t peak_co_value = 0u;
/*****************************************************************************************/
/*                               Co module Static Functions                                              */
/*****************************************************************************************/

#if LOW_CO_MONITOR
static void lowCOMonitor(void);
#endif
static void Co_calculation(void);
static void intellegent_Sample_rate_monitor(void);
static void CoHB_Calculations(void);
static uint16_t calculateTempSF(int16_t Temperature);

void Co_Diagnostic();
void set_CO_Level(const CO_LEVEL level);

uint32_t GetPWM_Cycle(void);
void setCOValue(const uint32_t CO_Value);
OPERATING_MODE getmode(void);
bool getIntelligentSampleRate(void);
uint8_t getgain(void);
void setgain(const uint8_t gain);
static void verifyAlarm(void);
FAULT_CIRCUIT SensorTestCarbonMonoxide(void);
static void boot_up_flash_read(void);

/* Co value During alarm*/
static uint16_t Alarm_CO_Val = 0u;
static uint8_t co_Overload_Events = 0u;     /**< Number of Co Overload  events (co>500) */
static bool co_overload_events_exceeded;
static bool is_co_hw_fault_set = false;
static bool co_hw_bist_fault = false;
/**
 * @brief This Function is to setup default CO module variable
 */
void defaultVariableCO(void)
{
	CO_Status.Gain = HIGH_GAIN;
	CO_Status.op_mode = NORMAL_MODE;
	CO_Status.co_level = NO_CO;
	CO_Status.lowGainHysterisisVal = 0u;
	CO_Status.Overload_Counter = 0u;
	CO_Status.CO_Low_Level_Peak = 0u;
	CO_Status.Co_RAW_reading = 0u;
	CO_intelligentSampleData.Intelligent_SAMPLE_RATE_Count = 0U;
	CO_Status.co_warning = NO_LOW_LEVEL_CO;
	CO_Status.Co_circuit_fault = NO_CO_FAULT;
	CO_Status.Co_Final_after_compensations = 0u;
	CO_Status.COHB_Calculation = 0u;
	CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED = false;
	CO_flags.Is_CO_ALARM_Active = false;
	CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start = false;
	CO_flags.Low_Level_timer_start = false;
	CO_flags.co_Sensor_Overload = false;
	CO_flags.Is_LOW_CO_Warning_enabled = false;
	CO_flags.overload_Compensation = false;
	CO_flags.co_Sensor_Overload_Occurred = false;
	CO_flags.disable_Co_overload_Compensation_timer = false;
	
	uint8_t coPeriod = DataLogging_GetCOAcqPeriod();
	if(coPeriod != CO_MEASUREMENT_PERIOD)
	{
	    coPeriod = CO_MEASUREMENT_PERIOD;
	    DataLogging_SetCOAcqPeriod(coPeriod);
	}

	coPeriod = DataLogging_GetCOBistPeriod();
	if(coPeriod != CO_BIST_PERIOD)
	{
	    coPeriod = CO_BIST_PERIOD;
	    DataLogging_SetCOBistPeriod(coPeriod);
	}

  strike_count_OC = DataLogging_GetCOOpenCircuitStrikeCount( );

  if( strike_count_OC != STRIKE_COUNT_FAULTS )
  {
    strike_count_OC = STRIKE_COUNT_FAULTS;
    DataLogging_SetCOOpenCircuitStrikeCount(strike_count_OC);
  }

  strike_count_SC = DataLogging_GetCOShortCircuitStrikeCount( );

  if( strike_count_SC != NEW_SC_STRIKE_COUNT_FAULTS )
  {
    strike_count_SC = NEW_SC_STRIKE_COUNT_FAULTS;
    DataLogging_SetCOShortCircuitStrikeCount(strike_count_SC);
  }

  date_of_manufacture = DataLogging_GetDoM();
  if(date_of_manufacture == 0xFFFFFFFF)
  {
      date_of_manufacture = 0;
  }

  /* Read the flash */
  boot_up_flash_read();

  strike_out_count = 3;

  HighLowTempertureTimerCount = 0;   
  NormalTempertureTimerCount = 0;    
  HighLowTempertureAvg = 0;          
  FHighLowTemperture = 0;            
  HighLowTempertureCail = 0;         

  LowHumiInCount = 0;
  LowHumiOutCount = 0;
  RepaLowHumi = 0;
}

/**
 * @brief This Function is to read flash (Temp Comp +Aging + Const+ CRC values)
 */
static void boot_up_flash_read( void )
{
  /* Initialise co calib */
  cocal_init( );

	/* Make sure calibration data held in FLASH is not corrupted! */
  bool calib_ok = cocal_check_crc( );

  /* Calib all good? */
  if( calib_ok )
  {
    /* Yes, clear fault */
		CO_Status.Co_circuit_fault = NO_CO_FAULT;
		DEBUG_CO("No Calib fault", false, 0ul);
  }
  else  /* Calib bad */
  {
    /* Raise a fault */
    CO_Status.Co_circuit_fault = FLASH_CRC_FAULT;
    DEBUG_CO( "Calib Fault", false, 0ul );
    FaultHandler_FaultSet( EEPROMCalDataCorruptionFault );
    DataLogging_SetMinorFault(FaultCalibrationDataCorrupt, true);
  }
}


/**
 * @brief This Function is use to enable sensor Test short of  CO acquisition circuit
 */
static void SensorShortOn(void)
{
  EnableCO_withshunt_on();
  shunt_switch_on = true;
  DEBUG_CO("Poison switch ON time:", true, get_currentTime());
}

/**
 * @brief This Function is use to disable sensor Test short of  CO acquisition circuit
 */
static void SensorShortOff(void)
{

  DisableCO_withshunt_on();
  shunt_switch_on = false;
  DEBUG_CO("Poison switch OFF time:", true, get_currentTime());
}

/*******************************************************************************
 * @brief   Get CO raw reading
 *
 * @details This function obtains the raw CO reading from the AFE. This reading 
 *          is the delta between the reference and the actual reading. The ref
 *          is typically 300 mV. So the value returned will the number of mV
 *          above the ref.
 *
 * @param[in] setup    AFE setup
 * 
 * @note If simulate mode is enabled, the simulated counts will be returned
 * 
 * @return ADC counts
 */
static uint16_t get_raw_co_reading( const AFE_setup_t setup )
{
  uint16_t value = 0U;

  if( simulate_co_raw_mode == true )
  {
    value = simulated_co_raw_reading;
  }
  else
  {
    OS_MSG_SIZE size;

    RTOS_ERR err;

    hal_AFE_Post( setup, NULL, NULL, false );

    AFERspMessage_t *afeResponse = ( AFERspMessage_t * )OSTaskQPend( 0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err );

    APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

    value = afeResponse->afe_adc_data;
  }

  return( value );
}

/**
 * @brief  This is the function that will be called by the Event task for CO acquisition
 * @req PTR-984  : The CO Sensor Test mode is also known as Sniff Mode.
 *      PTR-1439 : Response and Recovery to a High CO Volume ratio
 *      PTR-687  : Validation Test > AFE > CO sense status
 *      PTR-810 :  AFE > CO Sense > Sample rate
 *      PTR-820 :  AFE > BIST > CO Detection > Periodic
 *      PTR-821:   AFE > CO Detection
 *      PTR-812:   Firmware > CO > COHB
 *      PTR-811:   Firmware > CO > Compensation
 */
void acquisitionCO( bool doCOdiag )
{
  OS_MSG_SIZE size;
  RTOS_ERR err;

  /* When we are in CO overload we stop taking CO readings for 11 to 12 minutes to protect the sensor */
  if( CO_flags.co_Sensor_Overload == false )
  {
    const AFE_setup_t setup = ( CO_Status.Gain == HIGH_GAIN ? setup_Co_High_gain : setup_Co_Low_gain );

    CO_Status.Co_RAW_reading = get_raw_co_reading( setup );

		/***************************Temperature + Humidity + Age Compensation ***************/
	  Co_calculation( );                  /* This will Calculate CO with All compensation */
	  /************************************************************************************/

    if( ( CO_Status.Co_Final_after_compensations >= CO_180PPM ) && ( CO_Status.op_mode == NORMAL_MODE ) )
    {
      CO_Status.op_mode = CO_FALSE_ALARM_MONITOR;

      for( uint8_t false_Alarm = 0U; false_Alarm < 1U; false_Alarm++ )
      {
        CO_Status.Co_RAW_reading = get_raw_co_reading(setup_Co_High_gain);
        LETimer_delay_ms(2U);

        DEBUG_CO("\t false alarm avoidance Co raw reading ", true, CO_Status.Co_RAW_reading );

        Co_calculation( );        /* This will re-Calculate CO with All compensation */
      }
    }

    /* Disable Diagnostic in offbase variance calculation to save power */
    if (CO_flags.offbase_co_variance == false)
    {
      if(CO_Status.Co_Final_after_compensations < CO_Diagnostic_180PPM_THRESHOLD)
      {
        if ( (doCOdiag == true) || (co_hw_bist_fault == true)) 
        {
          behaviour_state_enum_operational_States operational_state = getBehavioural_Operational_State();
          if (state_Smoke_Alarm != operational_state && (state_Heat_Alarm != operational_state) && (state_CO_Alarm != operational_state))
          {
            if ( (CO_temperatureData.Temperature > 500) && (CO_temperatureData.Temperature < 3500) && (CO_flags.overload_Compensation == false))  
            {
              LEDBuzz_AcquireBuzzer();
              Co_Diagnostic();
              LEDBuzz_ReleaseBuzzer();   
            }
          }
        }
      }
    }

    /****************************Intelligent sample rate ****************************** */
    /* Intelligent sample rate shall be de-activated in offbase */
    if (CO_flags.offbase_co_variance == false)
    {
      /* Intelligent sample rate shall be de-activated in case of Local alarm */
      if ((CO_Status.Co_Final_after_compensations >= CO_10PPM) && (CO_flags.Is_CO_ALARM_Active == false))
      {
        if (CO_Status.op_mode != SNIFFMODE)
        { /* No point to get increased sample rate in sniff mode,
             sniffmode sample rate is fixed to 10 second which is
             greater than intelligent sample rate
          */
          intellegent_Sample_rate_monitor();
        }

        /*****************************************************************************************************  */
      }
    }
#ifdef LOW_CO_MONITOR
    /* for offbase co acquisition No Low level calculation */
    if (CO_flags.offbase_co_variance != true){
      if ((CO_Status.op_mode != SNIFFMODE) && (CO_flags.Is_CO_ALARM_Active != true)){

        lowCOMonitor(); /* Low Co  detection shall be de-activated in Local Alarm, Otherwise it will create confusion in Local Alarm */
      }
    }
#endif
    /************************************************ ****************************************************  */

    /************************************************COHB Calculation  **********************************  */
    /* For COHB Calculation */
    /* for offbase co acquisition No COHB calculation */
    if (CO_flags.offbase_co_variance != true)
    {
      if ((CO_Status.op_mode != SNIFFMODE) && (doCOdiag == false)) /* COHB should be disable in Sniff Mode */
      {
        CoHB_Calculations(); /* COHB Calculator */
      }
    }

    /*disable the variance*/
    if (getvariance_status() == false)
    {
      //	DEBUG_CO("\t Enable variance ", DEF_NULL, DEF_NULL);
      (void)DoVariance();
    }

    /* disable Additional Compensation after 7days */
    if (CO_flags.disable_Co_overload_Compensation_timer == true)
    {
      //(void) LETimer_stop(LETIMER_CO_OVERLOAD_COMPENSATION);
      BURTCTimer_Stop(CO_OVERLOAD_event_1);
      CO_flags.disable_Co_overload_Compensation_timer = false;
      DEBUG_CO("\t Disable Overload ", false, 0ul);
    }

    /****************************************************************************************************  */

    /***********************************************Verify Alarm  ***************************************  */
    verifyAlarm(); /* This Module may replace by Behavioural Module */
    /***************************************************************** **********************************  */

    /*********************** This Code May change depend on timing Thread/ ************************************/

    /* This Will be used to switching the  Gain HIGH <-> LOW */
    if ((CO_Status.Co_RAW_reading > (int32_t)(MAX_COUNT)) && (CO_Status.Gain == HIGH_GAIN))
    {
      CO_Status.Gain = LOW_GAIN;
      CO_Status.lowGainHysterisisVal = ((CO_Status.Co_RAW_reading) / 2U) / 10U;
      DEBUG_CO("\tchange gain to Low ", false, 0ul);
    }
    else if ((CO_Status.Co_RAW_reading < (int32_t)(CO_Status.lowGainHysterisisVal)) && (CO_Status.Gain == LOW_GAIN))
    {
      CO_Status.Gain = HIGH_GAIN;
      DEBUG_CO("\tchange gain to High ", false, 0ul);
    }
    else
    {
      /* This else Statement is intentionally  empty to comply MISRA*/
    }

    /* if Predicted CO>180 then set the intelligent  Flag, for quick 12msec Co acq. reading
     * if No Alarm then read 5 times and set intelligent flag to false
     */
    if (CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED == true)
    {
      CO_intelligentSampleData.Intelligent_SAMPLE_RATE_Count++;

      if (CO_intelligentSampleData.Intelligent_SAMPLE_RATE_Count > Intelligent_SAMPLE_RATE_CTR)
      {
        CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED = false;
        CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start = false;
        (void)BURTCTimer_Stop(TMR_CO_IntelligentsampleRate_event_0);
        CO_intelligentSampleData.Intelligent_SAMPLE_RATE_Count = 0U;
        DEBUG_CO(" Intelligent SR Disable ", false, 0ul);
      }
    }
  }

  /* See if we are in overload protection mode? */
  if(doCOdiag == false)
  {
    if (CO_flags.co_Sensor_Overload == true)
    {
      CO_Status.Overload_Counter++;

      if (CO_Status.Overload_Counter > 12U)
      {
        /*reset the counter and enable reading in High Gain */
        CO_Status.Overload_Counter = 0u;
        CO_flags.co_Sensor_Overload = false;
        DEBUG_CO("\tDisable the Short, read Again", false, 0ul);
        SensorShortOff();
        CO_Status.Gain = HIGH_GAIN;
        CO_flags.overload_Compensation = true;
        BURTCTimer_Start(CO_OVERLOAD_event_1, true, OVERLOAD_COMPENSESTION_PERIOD);  
          //post_CO event
        OSFlagPost(&Event_Flags_SubGroup[0],
                  (uint32_t) EVENT_CO_MEASUREMENT_0,
                   OS_OPT_POST_FLAG_SET,
                   &err );
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
      }

      DEBUG_CO("\t T Overload_Counter", true, CO_Status.Overload_Counter);
    }
  }
}

/**
 * @brief  This is the function  will be called by the CO reading for different alarm conditions
 *
 */
static void verifyAlarm(void)
{
	RTOS_ERR err;
	uint8_t  log_peak_co[8u] = {0, };

	if (CO_Status.op_mode != SNIFFMODE)
  {
	    if( (CO_Status.COHB_Calculation >= COHB_THRESHOLD) ||  (true == shunt_switch_on) )
      {
        CO_Status.op_mode = LOCAL_ALARM;		
        CO_Status.COHB_Calculation = COHB_THRESHOLD; /* Should not more than 1200 */

  #ifdef LOWCO
        (void)LETimer_stop(LETIMER_CO_LowLevel); /* Disable  Low level Alarm */
  #endif

        /*Low level warning timer stop + disable the flag */

        CO_flags.Is_LOW_CO_Warning_enabled=false;

        /*record Alarm triggered Co  */
        if( CO_flags.Is_CO_ALARM_Active == false )
        {
          /*reset the trigger value for every new alarm*/
          Alarm_CO_Val = 0u;
          Alarm_CO_Val = ( uint16_t )CO_Status.Co_Final_after_compensations;

          DEBUG_CO("\tRaising CO Alarm:", true, Alarm_CO_Val);

              /*Set Alarm Active Flag*/
          CO_flags.Is_CO_ALARM_Active = true;

          /* set the local CO event */
          DataLogging_SetCOEvent(EVENT_TYPE_LOCAL);

          /* Check co hw fault exists, if no fault then raise alarm */
          if(is_co_hw_fault_set == false)
          {
              OSFlagPost( &Event_Flags_SubGroup [ 0 ], (uint32_t) EVENT_COHB_HIGH_COHB_SUPER_0, OS_OPT_POST_FLAG_SET, &err );
              APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
			  
              if(CO_Status.co_level == HIGH_CO_LESS_THAN_150PPM)
              {
                  co_alarm_st = 2u; // set the co detection state
			  	 
                  DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_START, NULL );
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              }
              else if(CO_Status.co_level == SUPERCO)
              {
                  co_super_alarm_st = 1u; // set the super co level
                  co_alarm_st = 0u;
                  DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_CO_START, NULL );
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
              }
          }
        }
        else
        {
          DEBUG_CO("\tCO Alarm already active:", true, Alarm_CO_Val);
          /*LDRA*/
        }
        /* Disable the intelligent Sample rate */
      }
	    /* Low CO Alarm Warning module will be active Always only clear Low warning flag in case of CO Alarm or CO less than 10 */
	  }

    if (CO_Status.op_mode == SNIFFMODE)
    {
        if (CO_Status.Co_Final_after_compensations >= CO_30PPM)
        {
            /* send the event only once */
            DEBUG_CO("\tFire Sniff Mode Detected", false, 0ul);
        }
    }
    else
    {
        /* This Else Statement is intentionally  empty to comply MISRA*/
    }
 }


/**
 * @brief  This is the function  will be called by the CO reading for increasing sampling frequency In case of High CO value
 *@return flag
 *
 */
static void intellegent_Sample_rate_monitor(void)
{
	int32_t m, c, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
	static int8_t sample_Count = 0;
  int32_t x[SAMPLE_CT] = { 1, 2 }; /* Only Two Samples */
	static uint8_t status = 0;

	if (status == 0u){

		CO_intelligentSampleData.Co_intelligent_sample[sample_Count] = (int16_t) CO_Status.Co_Final_after_compensations;
		sample_Count++;
	}
	else{
		/* read the  previous samples */

		CO_intelligentSampleData.Co_intelligent_sample[0] = (int16_t) CO_intelligentSampleData.Co_intelligent_sample[1];
		CO_intelligentSampleData.Co_intelligent_sample[1] = (int16_t) CO_Status.Co_Final_after_compensations;

		sample_Count = (int8_t) SAMPLE_CT;
	}

	if (sample_Count == (int8_t) SAMPLE_CT){
		/* Additional Protection if second sample is not greater or equal then no need to run*/
		if (CO_intelligentSampleData.Co_intelligent_sample[1] >= CO_intelligentSampleData.Co_intelligent_sample[0]){

			for (int8_t i = 0; i < (int8_t) SAMPLE_CT; i++){
				sum_x += x[i];
				sum_y += CO_intelligentSampleData.Co_intelligent_sample[i];
				sum_xy += x[i] * CO_intelligentSampleData.Co_intelligent_sample[i];
				sum_x2 += (x[i] * x[i]);

			}
			status = 1u;
			if ((sum_x2 != 0) || (sum_x != 0) || (sum_y != 0) || (sum_xy != 0)){

				m = (((2 * sum_xy) - (sum_x * sum_y)) / ((2 * sum_x2) - (sum_x * sum_x)));
				c = (sum_y - (m * sum_x)) / 2;

				/*PRedictions */
				for (uint32_t predict = SAMPLE_CT; predict < (uint32_t) SAMPLE_BFR_CTR; predict++){
					sample_Count += 1;
					CO_intelligentSampleData.Co_intelligent_sample[predict] = (int16_t)((m * (sample_Count)) + c);

				}
				sample_Count = 0;

				for (sample_Count = 0; sample_Count < (int8_t) SAMPLE_BFR_CTR; sample_Count++){
					if (CO_intelligentSampleData.Co_intelligent_sample[sample_Count] >= CO_180PPM){
						CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED = true;

						if (CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start == false)
						{
						  BURTCTimer_Start(TMR_CO_IntelligentsampleRate_event_0, true, CO_INCREASED_SAMPLE_RATE);
						  CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start = true;
						}
						DEBUG_CO("Int sample En", false, 0ul);

						break;
					}

				}

			}
			else{
				CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED = false;
				CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start = false;
			}

		}
		else{

			/* This Else Statement is intentionally  empty to comply MISRA */
		//	DEBUG_CO("\t Not needed ", true, CO_intelligentSampleData.Co_intelligent_sample[1]);
			status = 1u;

		}

	}
	else{
	//	DEBUG_CO("\t Less sample ", true, CO_intelligentSampleData.Co_intelligent_sample[1]);
		/* This Else Statement is intentionally  empty to comply MISRA */
	}

}

/**
 * @brief  This is the function  will be called by the CO reading for  COHB Calculating
 * @notes As per stuart:  COHB=COHB/5 in intelligent sample rate
 */
static void CoHB_Calculations(void)
{
  if (CO_Status.op_mode != SNIFFMODE)
  {
    if ((CO_Status.Co_Final_after_compensations >= CO_37PPM))
    {
      AlarmOutCount = 0;
      CO_flags.Is_CO_Detected = true;
      if ((CO_Status.Co_Final_after_compensations <= CO_73PPM))
      {
        if (CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED != true)
        {
          CO_Status.COHB_Calculation += (uint16_t)CO_ALARM_MIN_74;
        }
        else
        {
          CO_Status.COHB_Calculation += (uint16_t)(CO_ALARM_MIN_74) / COHB_DIV;
        }
      }
      else if ((CO_Status.Co_Final_after_compensations > CO_73PPM) && (CO_Status.Co_Final_after_compensations < (uint32_t)CO_180PPM))
      {

          if( simulate_co_raw_mode == true )
          {
              CO_Status.COHB_Calculation = CO_ALARM_MIN_0;
          }
          else
          {
              if (CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED != true)
              {
                CO_Status.COHB_Calculation += (uint16_t)CO_ALARM_MIN_25;
              }
              else
              {
                CO_Status.COHB_Calculation += (uint16_t)(CO_ALARM_MIN_25) / COHB_DIV;
              }

          }
      }

      else if ((CO_Status.Co_Final_after_compensations > CO_179PPM))
      {
        CO_Status.COHB_Calculation = (uint16_t)CO_ALARM_MIN_0;
      }
      else /* LDRA stuff */
      {
        CO_Status.COHB_Calculation = (uint16_t)CO_ALARM_MIN_0;
      }
    }
    else
    {
      if (CO_Status.COHB_Calculation >= (uint16_t)NO_ALARM)
      {
        CO_Status.COHB_Calculation -= (uint16_t)NO_ALARM; 
        CO_flags.Is_LOW_CO_ALARM_Active = false; /* Stop Local Alarm if CO less than 40 */
        CO_flags.Is_CO_ALARM_Active = false;
        CO_Status.op_mode = NORMAL_MODE;
        RTOS_ERR err;
        if (CO_flags.Is_LOW_CO_Warning_enabled != true)
        {
             if(AlarmOutCount < 3)
            {
                AlarmOutCount++;
            }
            DEBUG_CO("\n1-Removing CO Alarm", false, 0ul);
            DEBUG_CO("\nRemoving CO Alarm", true, co_alarm_st);
            DEBUG_CO("\nRemoving CO Alarm", true, co_super_alarm_st);
            DEBUG_CO("\nRemoving CO Alarm", true, AlarmOutCount);
            if(((co_alarm_st >= 1u)||(co_super_alarm_st == 1)) && (AlarmOutCount >= 2))
            {
                uint8_t logbook_peak_co[8u] = {0, };
                  DEBUG_CO("\n2-Removing CO Alarm", false, 0ul);
                  CO_Status.COHB_Calculation = 0u;
                  /* PTR-1208 update the logbook with peak CO level during the alarm */
                  logbook_peak_co[1] = (uint8_t)((peak_co_value >> 8u) & 0xff);
                  logbook_peak_co[0] = (uint8_t)(peak_co_value & 0xff);

                  peak_co_value = 0u;  //reset the peak before the next alarm starts
                  RTOS_ERR err;
                  if(co_alarm_st == 2u)
                  {
                      DEBUG_CO("CO End sent", false, 0u);
                      co_alarm_st = 0u;
                      DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_END, logbook_peak_co );
                      OSTimeDly(5, OS_OPT_TIME_DLY, &err);
                      OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)EVENT_COHB_NONE_0,
                      OS_OPT_POST_FLAG_SET, &err); /* post the ADS off-base state */
                      APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
                  }
                  if (co_super_alarm_st == 1u)
                  {
                      DEBUG_CO("Super CO End sent", false, 0u);
                      co_super_alarm_st = 0u;
                      DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_CO_END, logbook_peak_co );
                      OSTimeDly(5, OS_OPT_TIME_DLY, &err);
                      OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)EVENT_COHB_NONE_0,
                      OS_OPT_POST_FLAG_SET, &err); /* post the ADS off-base state */
                      APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
                  }
                }
                else
                {
                  DEBUG_CO(" Low level warning is enabled", false, 0ul);
                }
                CO_flags.Is_LOW_CO_ALARM_Active = false; /* Stop Local Alarm if CO less than 40 */
                CO_flags.Is_CO_ALARM_Active = false;
                CO_Status.op_mode = NORMAL_MODE;
            }
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
        }
        else
        {
          CO_Status.COHB_Calculation = 0U;
          CO_flags.co_Sensor_Overload = false;
          CO_flags.Is_CO_Detected = false;
          CO_flags.Is_CO_ALARM_Active = false; /* disable co warning */
          CO_Status.op_mode = NORMAL_MODE;
          /* postFlag(&Event_Flags, EVENT_COHB_LOW);  Not required May Be Low CO warning  */
        }
        /* if Larger CO >449 then Overload activated  short sensor will be high
         *ADC reading shall be continue once COHB is 0 after 18 to 20 minute (after clearing
         * CO ) enable normal reading*/
        /*	postFlag(&Event_Flags, EVENT_COHB_HIGH);  As Per New implementation */
    } /*  else When CO is less than 40 PPM */
  }
  DEBUG_CO("\tCOHB", true, (uint32_t)CO_Status.COHB_Calculation);
}

/**
 * @brief  This is the function  will be called by the CO reading for  TempSF Calculation
 * @param Temperature
 * @return TempSF Value.
 * @note  Reference P0129 code
 *
 */
static uint16_t calculateTempSF(int16_t Temperature)
{
	uint16_t RelativeTemperature;
	uint16_t TempSF;
	/* Relative temperature - as a multiple of 0.5 deg/C, in range -20 to +50
	 * As temperature is in centi degrees celsius:
	 */
	 // DEBUG_CO("C.T", true, Temperature);
	if (Temperature > (int16_t)(MAX_TMPR_VALUE)){
		RelativeTemperature = RelativeTemperatureHi;
	}
	else if (Temperature < (int16_t)(MIN_TMPR_VALUE)){
		RelativeTemperature = RelativeTemperatureLo;
	}
	else{
		RelativeTemperature = (uint16_t)((uint16_t)(Temperature + TEMP_VAL_CAL) / TEMP_VAL_CAL_DEV);
	}

  TempSF = COSENSORTCR[RelativeTemperature];  
  return (TempSF);
}


/*******************************************************************************
 * @brief   Calculate mV
 *
 * @details This function takes the given number of ADC counts and calculates
 *          the ABUF corrected mV
 * 
 * @param[in] counts		  ADC counts
 * @param[in] high_gain		High gain mode
 * 
 * @return Corrected mV
 */
static int32_t counts_to_mv( const uint32_t counts, const bool high_gain )
{
  const uint32_t raw_max  = ( high_gain ? HIGH_GAIN_COUNTS_MAX : LOW_GAIN_COUNTS_MAX );

  const uint32_t mv_max   = ( high_gain ? HIGH_GAIN_MILLIVOLTS_MAX : LOW_GAIN_MILLIVOLTS_MAX );

  const int32_t mv  = ( counts * mv_max ) / raw_max;

  DEBUG_CO( "\tCO mV:         ", true, ( uint32_t )mv );

  const int32_t mvc = hal_AFE_abuf_correct( mv );

  return( mvc );
}

/*******************************************************************************
 * @brief   Calculate nA
 *
 * @details This function takes the given mV and calculates nA
 * 
 * @param[in] mv	mV
 * 
 * @return Corrected nA
 */
static int32_t mv_to_na( const int32_t mv )
{
  int32_t na = (1000 * mv) / 1000;

  return( na );
}

/*******************************************************************************
 * @brief   Calculate ppm
 *
 * @details This function takes the given nA and calculates ppm
 * 
 * @param[in] na	        nA
 * @param[in] ppm_per_na  nA per ppm
 * 
 * @note ppm_per_na is scaled by COCAL_NA_PER_PPM_SCALING_FACTOR
 * 
 * @return Calculated ppm
 */
static int32_t na_to_ppm( const int32_t na, const uint32_t ppm_per_na )
{
  const int32_t ppm = ( na * COCAL_NA_PER_PPM_SCALING_FACTOR ) / ppm_per_na;

  return( ppm );
}

/**
 * @brief  This is the function  will be called by the CO reading for  calculating CO with different Compensation
 * @return compensation value
 */
static void Co_calculation(void)
{
  behaviour_state_enum_System_modes mode = getBehavioural_System_Modes(false);
  if (mode == Functional_Test_Mode) {
     /* Get the value from FTM*/
  }

	uint32_t Faults = FaultHandler_GetFaultFlags();
	//uint16_t gainFactor;

	uint64_t compensations_values; /**<  Compensation value*/
	uint64_t Co_DIV_Factor;
	int16_t calibTemp;
	uint8_t logbook_peak_co[8u] = {0, };

	/* Default Temperature  & Humidity in case of HTU20D chip failure & Sniffmode (Alarm >30)*/
	if ((((Faults & DEF_TEMP_SENSOR_HW_FAULT)|| (Faults & DEF_HUMIDITY_SENSOR_HW_FAULT)) != 0u) ||  (CO_Status.op_mode == SNIFFMODE)){
		CO_temperatureData.Temperature = 2300;
	}
	else{

  		CO_temperatureData.Temperature=SHT41_get_temperature();
	    DEBUG_CO("Temp v: ", true, (uint32_t)((CO_temperatureData.Temperature)));
	}

  calibTemp = (int16_t) cocal_get_co_calib_temperature( );
#ifdef CALIB
	/*Calculate All Compensation  factor Value */
	calibration_verifyData();

	calibTemp = (int16_t) calibration_getTempValue();
	calibTemp = ((calibTemp + 200) * 10);
#endif

	if (CO_Status.Co_RAW_reading != 0u)
  {

		/* Calculation
		 * During Bootup Flash will be read and copy in buffer
		 * if Anything get corrupted or temp sensor became faulty then below calculation
		 * will use the default values
		 * default values are Temp 2300, Humidity 51 ,CO_CAL_factor=20
		 */

    uint32_t co_raw_reading = CO_Status.Co_RAW_reading;

    if(FHighLowTemperture == 1)
    {
      if(co_raw_reading > HighLowTempertureCail)
      {
        co_raw_reading = co_raw_reading - HighLowTempertureCail;  
      }
      else
      {
        co_raw_reading = 0;  
      }
    }

    DEBUG_CO("\tCO Raw:        ", true, co_raw_reading);

    /* Take the raw reading and calculate the mV */
    const int32_t milli_volts = counts_to_mv( co_raw_reading, ( CO_Status.Gain == HIGH_GAIN ? true : false ) );
    
		DEBUG_CO( "\tCO mV ABUF:    ", true, ( uint32_t )milli_volts );

    /* Using the mV value, calculate the number of milli-volts */
    const int32_t nano_amps = mv_to_na( milli_volts );

    /* Get the number of nano amps per ppm configuration for the sensor */
    const uint32_t co_na_per_ppm = cocal_get_co_na_per_ppm( );

		DEBUG_CO( "\tCO nA:         ", true, ( uint32_t )nano_amps );
		DEBUG_CO( "\tCO nA/ppm:     ", true, ( uint32_t )co_na_per_ppm );

    /* Using the number of nano amps, calculate the CO value in ppm */
    const int32_t co_ppm = na_to_ppm( nano_amps, co_na_per_ppm );

		DEBUG_CO( "\tCO ppm:        ", true, ( uint32_t )co_ppm );

    /* Save as the CO calibrated value */
		CO_Status.co_calibrated = co_ppm;

		DEBUG_CO( "\tCO Cal:        ", true, ( uint32_t )CO_Status.co_calibrated );

		/* Update Humidity only after 24 hours Avg */


    uint16_t relative_temperature = calculateTempSF( ( int16_t )CO_temperatureData.Temperature );

    uint16_t relative_temperature_calib = calculateTempSF( calibTemp );

    // CO_humidityData.humidity_Correction_factor_Calib = 1;
    // CO_humidityData.humidity_Correction_factor_Val = 1;
    uint8_t humidity_value;
    humidity_value = get_HumidityVal();
    if(humidity_value < STWCOD_HUMIDITY_IN_LIMLT)
    {
      LowHumiOutCount = 0;
      if(RepaLowHumi == 0)
      {
        LowHumiInCount++;
        if(LowHumiInCount >= 8640)
        {
          LowHumiInCount = 8640;
          RepaLowHumi = 1;
          DEBUG_CO("Co low humi in", false, 0);
        }
      }
    }
    else
    {
      LowHumiInCount = 0;
      if((RepaLowHumi == 1) && (humidity_value > STWCOD_HUMIDITY_OUT_LIMLT))
      {
        LowHumiOutCount++;
        if(LowHumiOutCount >= 1728)
        {
          LowHumiOutCount = 1728;
          RepaLowHumi = 0;
          DEBUG_CO("Co low humi out", false, 0);
        }
      }
    }
    uint32_t curr_time;
    volatile uint32_t curr_hour12;
    curr_time = get_currentTime();

    if ((date_of_manufacture == 0u) || (date_of_manufacture > curr_time))
    {
        curr_time = 0u;
    }
    else
    {
        curr_time = curr_time - date_of_manufacture;
    }

    if(curr_time < 43200)
    {
      curr_hour12 = 0;
    }
    else
    {
      curr_hour12 = (curr_time / 43200);
    }
    uint32_t aging,aging_calib;
    float aging_sf;
    /* Change to 40000 as per DCR number 0155*/
    aging_calib = (40000U - curr_hour12);
    aging = 40000U;
    aging_sf = (float)(aging) / (float)(aging_calib);
    DEBUG_CO("\thourcount:     ", true, (uint32_t)curr_hour12);
   
    const uint32_t top = (relative_temperature_calib);
    const uint32_t bottom = (relative_temperature);
    DEBUG_CO("\tCO Top:        ", true, (uint32_t)top);
    DEBUG_CO("\tCO Bottom:     ", true, (uint32_t)bottom);
    if( ( CO_Status.co_calibrated > 0 ) && ( top > 0 ) && ( bottom > 0 ) )
    {
      CO_Status.Co_Final_after_compensations = (CO_Status.co_calibrated * top * aging_sf) / bottom;
    }
    else
    {
      CO_Status.Co_Final_after_compensations = CO_Status.co_calibrated;
    }

    HighLowTempertureCalibration();
    /* WorkArround: We have to additionally compensate CO for 24 Hours.  ( if CO >5000, After Sensor short, Sensor became unresponsive for certain time)*/
    if (CO_flags.overload_Compensation == true)
    {
      /* As Suggested %25 PPM  to current reading for 24 Hours*/
      CO_Status.Co_Final_after_compensations = CO_Status.Co_Final_after_compensations + 5;
    }
    if(RepaLowHumi == 1)
    {
      DEBUG_CO("Co low humi repa", false, 0);
      CO_Status.Co_Final_after_compensations = CO_Status.Co_Final_after_compensations + 5;
    }
    DEBUG_CO("Co with Compensation =", true, (uint32_t) CO_Status.Co_Final_after_compensations);

    RTOS_ERR err;

    if( simulate_co_raw_mode == true )
    {
        CO_Status.Co_Final_after_compensations = simulated_co_raw_reading;
    }

	/* keep record of the peak CO level during the alarm condition */
	if(CO_Status.Co_Final_after_compensations > peak_co_value)
	{
	  peak_co_value = CO_Status.Co_Final_after_compensations;
	}
	
    if ((CO_Status.Co_Final_after_compensations >= CO_150PPM) && (CO_Status.Co_Final_after_compensations <= CO_2000PPM))
  {
      if( CO_Status.co_level == NO_CO )
      {
        CO_Status.co_level = SUPERCO;
		co_super_alarm_st = 1u;
		co_alarm_st = 0u;
      }
      else if(CO_Status.co_level == HIGH_CO_LESS_THAN_150PPM)
      {
          CO_Status.co_level = SUPERCO;
          if ((CO_flags.Is_CO_ALARM_Active == true) && (co_super_alarm_st == 0u))
          {
              co_super_alarm_st = 1u;

              if(co_alarm_st == 2u)
              {
                co_alarm_st = 0u;

                /* PTR-1208 update the logbook with peak CO level during the alarm */
                logbook_peak_co[1] = (uint8_t)((peak_co_value >> 8u) & 0xff);
                logbook_peak_co[0] = (uint8_t)(peak_co_value & 0xff);

                DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_END, logbook_peak_co );
                OSTimeDly(5, OS_OPT_TIME_DLY, &err);

                peak_co_value = 0u; // reset the peak CO value
              }
              else
              {
                DEBUG_CO("This is not a CO practical condition", false, 0u);
              }

              DataLogging_SetCOEvent(EVENT_TYPE_LOCAL);
              OSTimeDly(1, OS_OPT_TIME_DLY, &err);

              OSFlagPost( &Event_Flags_SubGroup [ 0 ], (uint32_t) EVENT_COHB_HIGH_COHB_SUPER_0, OS_OPT_POST_FLAG_SET, &err );
              APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );

              DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_CO_START, NULL );
              OSTimeDly(5, OS_OPT_TIME_DLY, &err);

              DEBUG_CO("\n Afte CO end super CO start", false, 0u);
          }
          else
          {
              /* let the verifyAlarm() send the event log this time */
          }
      }
    }
    else if( ( CO_Status.Co_Final_after_compensations > CO_10PPM ) && ( CO_Status.Co_Final_after_compensations < CO_150PPM ) )
    {
        if(( CO_Status.co_level == SUPERCO) && (CO_flags.Is_CO_ALARM_Active == true))
        {
            CO_Status.co_level = HIGH_CO_LESS_THAN_150PPM;

            if (co_super_alarm_st == 1u)
            {
              co_super_alarm_st = 0u;
              DEBUG_CO("\n before enter Super CO", false, 0u);

              /* PTR-1208 update the logbook with peak CO level during the alarm */
              logbook_peak_co[1] = (uint8_t)((peak_co_value >> 8u) & 0xff);
              logbook_peak_co[0] = (uint8_t)(peak_co_value & 0xff);

              DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_CO_END, logbook_peak_co );
              OSTimeDly(5, OS_OPT_TIME_DLY, &err);

              peak_co_value = 0u; // reset the peak before next alarm event starts

              if(co_alarm_st == 0u)
              {
                  co_alarm_st = 2u;
                  DataLogging_SetCOEvent(EVENT_TYPE_LOCAL);
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);
                  DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_START, NULL );

                  OSFlagPost( &Event_Flags_SubGroup [ 0 ], (uint32_t) EVENT_COHB_HIGH_COHB_SUPER_0, OS_OPT_POST_FLAG_SET, &err );
                  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
              }
              else
              {
                  DEBUG_CO("This CO condition is not practical", false, 0u);
              }
            }
            else
            {
              /* let the verifyAlarm() send the event log this time */
            }
        }
        else if(CO_Status.co_level == NO_CO)
        {
            CO_Status.co_level = HIGH_CO_LESS_THAN_150PPM;
            /* let the verifyAlarm() send the event log this time */
			co_alarm_st = 1u;
			co_super_alarm_st = 0u;
        }
    }
    else if( CO_Status.co_level != NO_CO )
    {
        if(is_co_hw_fault_set == false)
        {
            CO_Status.co_level = NO_CO;

            /* PTR-1208 update the logbook with peak CO level during the alarm */
            logbook_peak_co[1] = (uint8_t)((peak_co_value >> 8u) & 0xff);
            logbook_peak_co[0] = (uint8_t)(peak_co_value & 0xff);

            if((co_super_alarm_st == 1u) && (CO_Status.COHB_Calculation >= NO_ALARM ))
            {
                co_super_alarm_st = 0u;
                DataLogging_SetEventLogbookRecord( DEF_LBE_SUPER_CO_END, logbook_peak_co );
                OSTimeDly(5, OS_OPT_TIME_DLY, &err);

                peak_co_value = 0u;

                if(co_alarm_st == 0u)
                {
                    /* CO HB accumulation is still there */
                  co_alarm_st = 2u;
                  DataLogging_SetCOEvent(EVENT_TYPE_LOCAL);
                  OSTimeDly(1, OS_OPT_TIME_DLY, &err);

                  DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_START, NULL );
                  OSTimeDly(5, OS_OPT_TIME_DLY, &err);

                  OSFlagPost( &Event_Flags_SubGroup [ 0 ], (uint32_t) EVENT_COHB_HIGH_COHB_SUPER_0, OS_OPT_POST_FLAG_SET, &err );
                  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
                }
           }
        }
    }
    else
    {
        /* Do nothing */
    }
  }
  else
  {
    if (co_super_alarm_st == 1u)
    {
      DEBUG_CO("Siterwell ELSE enter", false, 0u);
      RTOS_ERR err;
      co_super_alarm_st = 0u;
      logbook_peak_co[1] = (uint8_t)((peak_co_value >> 8u) & 0xff);
      logbook_peak_co[0] = (uint8_t)(peak_co_value & 0xff);
      DataLogging_SetEventLogbookRecord(DEF_LBE_SUPER_CO_END, logbook_peak_co);
      OSTimeDly(5, OS_OPT_TIME_DLY, &err);
      peak_co_value = 0u;  // reset the peak before next alarm event starts
      if (co_alarm_st == 0u)
      {
        co_alarm_st = 2u;
        DataLogging_SetCOEvent(EVENT_TYPE_LOCAL);
        OSTimeDly(1, OS_OPT_TIME_DLY, &err);
        DataLogging_SetEventLogbookRecord(DEF_LBE_CO_DET_START, NULL);
        OSTimeDly(1, OS_OPT_TIME_DLY, &err);
        OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t)FLAGS_BIT_INDEX(TMR_COHB_HIGH_SUPER_event_0), OS_OPT_POST_FLAG_SET, &err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
      }
      else
      {
        DEBUG_CO("This CO condition is not practical", false, 0u);
      }
    }
    else if( (CO_Status.co_level != NO_CO) && (CO_flags.Is_CO_ALARM_Active == false))
    {
      CO_Status.co_level = NO_CO;
    }
		CO_Status.Co_Final_after_compensations = 0U;
  }

	/* created simple Array Round Robin buffer of size 10 and maintaining current index */
	CO_Alarm_Values[current_co_Buffer_index] = (uint16_t) CO_Status.Co_Final_after_compensations;
	current_co_Buffer_index++;
	if (current_co_Buffer_index == BUFFER_INDEX_VAL)
	{
		current_co_Buffer_index = 0U;
	}
    if ((CO_Status.Co_Final_after_compensations >= CO_600PPM))
    {
		CO_flags.co_Sensor_Overload = true;
		CO_flags.co_Sensor_Overload_Occurred = true;
		SensorShortOn();
	}

	/* Read maximum CO values when device in Alarm*/
	if (CO_flags.Is_CO_ALARM_Active == true)
	{
		if (CO_Status.Co_Final_after_compensations > Alarm_CO_Val)
		{
			Alarm_CO_Val = (uint16_t) CO_Status.Co_Final_after_compensations;
		}
	}
}

static void HighLowTempertureCalibration(void)
{
  if ((CO_temperatureData.Temperature < STWCOD_LOW_TEMPERTURE) || (STWCOD_HIGH_TEMPERTURE < CO_temperatureData.Temperature))
  {
    if (HighLowTempertureTimerCount < (STWCOD_HIGHLOW_TEMPERTURE_TIMER + 16))  
    {
      HighLowTempertureTimerCount++;  
    }
    if (HighLowTempertureTimerCount >= 3)  
    {
      NormalTempertureTimerCount = 0;
    }
  }
  else
  {
    if (NormalTempertureTimerCount < (STWCOD_NORMAL_TEMPERTURE_TIMER + 10))  
    {
      NormalTempertureTimerCount++;  
    }
    if (NormalTempertureTimerCount >= 3)  
    {
      HighLowTempertureTimerCount = 0;
    }
  }
  if (HighLowTempertureTimerCount == STWCOD_HIGHLOW_TEMPERTURE_TIMER)  
  {
    HighLowTempertureAvg = 0;  
  }

  if ((HighLowTempertureTimerCount >= STWCOD_HIGHLOW_TEMPERTURE_TIMER) && (HighLowTempertureTimerCount < (STWCOD_HIGHLOW_TEMPERTURE_TIMER + 8)))
  {
    HighLowTempertureAvg += CO_Status.Co_RAW_reading;
  }
  if (HighLowTempertureTimerCount == (STWCOD_HIGHLOW_TEMPERTURE_TIMER + 8))
  {
    HighLowTempertureAvg >>= 3;

    if (CO_Status.Co_Final_after_compensations <= 10)  
    {
      FHighLowTemperture = 1;
      HighLowTempertureCail = HighLowTempertureAvg;
      DEBUG_CO("HLTemperCail enter=", true, (uint32_t)HighLowTempertureCail);
    }
  }

  if (NormalTempertureTimerCount >= STWCOD_NORMAL_TEMPERTURE_TIMER)
  {
    FHighLowTemperture = 0;
    HighLowTempertureCail = 0;
    DEBUG_CO("HLTemperCail out=", false, 0);
  }
}

#ifdef LOWCO
/**
 * @brief  This is the function will monitor LOW CO
 *
 */
static void lowCOMonitor(void)
{
	static uint32_t Low_Co_Period;
	static uint32_t timer_period;
	/* For Low Level CO Warning */
	if ((CO_Status.Co_Final_after_compensations >= CO_10PPM)&&
	    (false == GetTempAbove50DegFlag())){
		CO_flags.Is_LOW_CO_Warning_enabled = true;

		if (CO_Status.Co_Final_after_compensations <= CO_20PPM){
			CO_Status.co_warning = LOW_LEVEL_CO_WARNING_4HRS;
			Low_Co_Period = LO_LEVEL_CO_TIMEOUT_4_HR;

		}
		else if ((CO_Status.Co_Final_after_compensations > CO_20PPM)
		        && (CO_Status.Co_Final_after_compensations <= CO_30PPM)){
			CO_Status.co_warning = LOW_LEVEL_CO_WARNING_35MIN;
			Low_Co_Period = LO_LEVEL_CO_TIMEOUT_35_MIN;

		}

		else if ((CO_Status.Co_Final_after_compensations > CO_30PPM)
		        && (CO_Status.Co_Final_after_compensations <= CO_80PPM)){

			CO_Status.co_warning = LOW_LEVEL_CO_WARNING_20MIN;
			Low_Co_Period = LO_LEVEL_CO_TIMEOUT_20_MIN;
		} /* If the CO more than 80 No need of CO Warning
		 It will generate Alarm using COHB calculations; suggested By Stuart */
		else if (CO_Status.Co_Final_after_compensations > CO_80PPM){

			CO_Status.co_warning = NO_LOW_LEVEL_CO;
			Low_Co_Period = 0U;

			if (CO_flags.Low_Level_timer_start == true){
				(void) LETimer_stop(LETIMER_CO_LowLevel);
				CO_flags.Low_Level_timer_start = false;
				CO_flags.Is_LOW_CO_Warning_enabled = false;
			}

		}
		else{
			/* CO_Status.co_warning = NO_LOW_LEVEL_CO; */
			Low_Co_Period = 0U;
		}

	}
	else /* IF CO is set to >10 and suddenly fall below 10 need to clear the Flags */
	{
		Low_Co_Period = 0U;
		CO_flags.Is_LOW_CO_Warning_enabled = false;
		CO_flags.Is_CO_ALARM_Active=false; /* Set the Flag No Alarm Active, if co <10*/
		CO_Status.co_warning = NO_LOW_LEVEL_CO;

		if (CO_flags.Low_Level_timer_start == true){
			(void) LETimer_stop(LETIMER_CO_LowLevel);
			CO_flags.Low_Level_timer_start = false;
		}
		DEBUG_CO("Dis LLW as No CO", DEF_NULL, DEF_NULL);

		RTOS_ERR err;
	  OSFlagPost(&Event_Flags_SubGroup[0], (uint32_t) EVENT_COHB_NONE_0,
	  OS_OPT_POST_FLAG_SET, &err); /* post the ADS off-base state */
	  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
		/*   Check error code.                                  */

		APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
	}
	/* Low level Timer Will Start & Stop in behavioural */
	if ((CO_Status.co_warning != NO_LOW_LEVEL_CO)){
		if ((CO_flags.Low_Level_timer_start == false)){
			CO_flags.Low_Level_timer_start = true;
			timer_period = Low_Co_Period;
			CO_Status.CO_Low_Level_Peak = (uint16_t) CO_Status.Co_Final_after_compensations;

			DEBUG_CO("CO LLW timer =", true, (uint32_t) timer_period);

			LETimer_start(LETIMER_CO_LowLevel, timer_period);

		}

		if ((timer_period != Low_Co_Period)){
			/*  Get the Timer Count */
			/* Set The New Time Period */
			timer_period = Low_Co_Period;

			LETimer_start(LETIMER_CO_LowLevel, timer_period);
			DEBUG_CO("New Time ", true, (uint32_t) timer_period);

		}
		/* New requirement Added find Low level Peak Values*/
		if (CO_flags.Low_Level_timer_start == true){
			if (CO_Status.CO_Low_Level_Peak < CO_Status.Co_Final_after_compensations){
				CO_Status.CO_Low_Level_Peak = (uint16_t) CO_Status.Co_Final_after_compensations;
			}
		}
	}
}
#endif /* PRODUCT != 234U */

/**
 * @brief  Performs a number of checks to determine if the Co reading Circuit is performing correctly.
 * @req PTR-813  AFE > BIST> CO Detection> Faults
 * @return Fault Circuit
 */
void Co_Diagnostic()
{
	coFault = NO_CO_FAULT;
#if 0
	static co_fault_counter =0u;

	if(strike_count_SC >= NEW_SC_STRIKECNT_ON_HIGHCO_SC)
	{
		co_fault_counter++;
		if(co_fault_counter >= NEW_SC_STRIKECNT_ON_HIGHCO_SC)
		{
			strike_count_SC = NEW_SC_STRIKE_COUNT_FAULTS;
			co_fault_counter = 0;
		}			
	}
	else
	{
		/* normal BIST */
	}
#endif

	/* in case of Overload(Co>449PPM): to protect sensor no Diagnostic & Acquisition as Sensor Short is enabled */
	if (CO_flags.co_Sensor_Overload == false)
    {
		/* configured to Push-pull output, idle low */
			coFault = SensorTestCarbonMonoxide();
	}
}

/**
 * @brief  This function disable/enable variance from product
 *@param disable=true
 */
void disable_variance(bool status_flag)
{
	CO_flags.disable_co_variance = status_flag;

}
/**
 * @brief  This function return variance on/off flag status
 *
 */
bool getvariance_status(void)
{
	return CO_flags.disable_co_variance;
}

/**
 * @brief  Performs a number of checks to determine if the CO hardware is performing correctly.
 * @return Fault Value
 * logbook_data_Co_Fault : Fault type & Values
 * 0x02 for Open Circuit Fault
 * 0x03 for Closed Circuit Fault
 * 0x04 for fatigue fault
 */
FAULT_CIRCUIT SensorTestCarbonMonoxide(void)
{
  RTOS_ERR err;
  OS_MSG_SIZE size;

	static uint32_t strike_counter_OC   = 0u;
	static uint32_t strike_counter_SC = 0u;
  static uint8_t strike_out_counter = 0;

	hal_AFE_Post(setup_CoSensorTest, NULL, NULL, false);
  AFERspMessage_t *afeResponse = (AFERspMessage_t *)OSTaskQPend(0, OS_OPT_PEND_BLOCKING, &size, DEF_NULL, &err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
    //CO_Status.Co_RAW_reading  = afeResponse->afe_adc_data;

	uint32_t Faults = FaultHandler_GetFaultFlags();
    
	/* Get the Fault status*/
	sensor_health=get_the_co_fault();
	if ((sensor_health==CO_Open_Circuit) && ((Faults & DEF_CO_SENSOR_HW_FAULT) == 0u))
	{
      /* default fault values based on  experiment */
	  strike_counter_OC++;
    strike_out_counter = 0;
    co_hw_bist_fault = true;
    if(BURTCTimer_Get_Event_Enable(TMR_CO_BIST_event_0) == true)
    {
      BURTCTimer_Stop(TMR_CO_BIST_event_0);
    }
      if (strike_counter_OC >= strike_count_OC)
      {
          if( CO_Status.Co_circuit_fault != OC_CO_FAULT )
          {
              CO_Status.Co_circuit_fault = OC_CO_FAULT;
              logbook_data_Co_Fault[0]=0x02u;
              DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_HW_ERR_START, logbook_data_Co_Fault );
              FaultHandler_FaultSet( COSenserHwFault );
              is_co_hw_fault_set = true;
              DEBUG_CO("XXXX Open Circuit Fault Reached The Stike CT XXX", false, 0ul);
          }
      }
      DEBUG_CO("XXXX Open Circuit Fault XXX", false, 0ul);
	}
	else if ((sensor_health==CO_Closed_Circuit) && ((Faults & DEF_CO_SENSOR_HW_FAULT) == 0u))
  {
	  strike_counter_SC++;
    strike_out_counter = 0;
    co_hw_bist_fault = true;
    if(BURTCTimer_Get_Event_Enable(TMR_CO_BIST_event_0) == true)
    {
      BURTCTimer_Stop(TMR_CO_BIST_event_0);
    }
	    if (strike_counter_SC >= strike_count_SC)
	    {
	        if( CO_Status.Co_circuit_fault != SC_CO_FAULT )
	        {
	            CO_Status.Co_circuit_fault = SC_CO_FAULT;
	            logbook_data_Co_Fault[ 0 ] = 0x03u;
	            DataLogging_SetEventLogbookRecord( DEF_LBE_CO_DET_HW_ERR_START, logbook_data_Co_Fault );
	            FaultHandler_FaultSet(COSenserHwFault);
	            is_co_hw_fault_set = true;
	            DEBUG_CO(" XXXXX Closed Circuit Fault  Reached The Stike CT XXXXXXX", false, 0ul);
	        }
	    }
	    DEBUG_CO(" XXXXX Closed Circuit Fault  XXXXXXX", false, 0ul);
	}
	else
  {
    	if((Faults & DEF_CO_SENSOR_HW_FAULT) != 0u)
    	{
    	    /* do nothing */
    	}
    	else
    	{
    	    /* no fault observed */
          strike_counter_SC = 0U;
          strike_counter_OC = 0U;
        if (is_co_hw_fault_set == true)
        {
          strike_out_counter++;
          if (strike_out_counter >= strike_out_count)
          {
            co_hw_bist_fault = false;
            strike_out_counter = strike_out_count;
            CO_Status.Co_circuit_fault = NO_CO_FAULT;
            is_co_hw_fault_set = false;
            if (BURTCTimer_Get_Event_Enable(TMR_CO_BIST_event_0) == false)
            {
              BURTCTimer_Start(TMR_CO_BIST_event_0, periodical, CO_BIST_PERIOD * 5);
            }
            DEBUG_CO(" XXXX No CO Circuit Fault XXXXX", false, 0ul);
          }
        }
        else
        {
          co_hw_bist_fault = false;
          CO_Status.Co_circuit_fault = NO_CO_FAULT;
          is_co_hw_fault_set = false;
          if (BURTCTimer_Get_Event_Enable(TMR_CO_BIST_event_0) == false)
          {
            BURTCTimer_Start(TMR_CO_BIST_event_0, periodical, CO_BIST_PERIOD * 5);
          }
          DEBUG_CO(" XXXX No CO Circuit Fault XXXXX", false, 0ul);
        }
    	}
  }
	return (CO_Status.Co_circuit_fault);
}

/*****  interface functions  For BIST Module OR for Other Modules  *** */

/**
 *@brief  function will set the Gain of Co Circuit.
 * @param gain value (High or Low)
 */
void setgain(const uint8_t gain)
{
	if (gain == 0U){
		CO_Status.Gain = 0U;
	}
	else{
		CO_Status.Gain = 1U; /* By default CO Gain Will be High */
	}

}

/**
 * @brief  function will return the Gain of Co Circuit.  * High=1, Low=0
 * @return gain value
 */
uint8_t getgain(void)
{

	return (CO_Status.Gain);
}

/**
 * @brief  Function will return the 10 CO Values
 * @param input Buffer array of size 10
 * @return current index of Buffer
 */
uint8_t get_Co_values(uint16_t *result)
{

	if (result != NULL){
		for (uint8_t Co_index = 0U; Co_index < 10U; Co_index++){
			result[Co_index] = CO_Alarm_Values[Co_index];
		}

	}

	return current_co_Buffer_index;

}

/**
 * @brief  This Function will Set the Sample rate flag of Co Circuit.
 * @param   flag to enable or disable
 */
void SetIntelligentSampleRate(bool state)
{
  CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED = state;

  if (state == false)
  {
    CO_intelligentSampleData.Intelligent_SAMPLE_RATE_timer_start = state;
    CO_intelligentSampleData.Intelligent_SAMPLE_RATE_Count = 0u;
    /* Stop The BURTCTimer*/
    (void)BURTCTimer_Stop(TMR_CO_IntelligentsampleRate_event_0);
  }
}

/**
 * @brief  This Function will return the Sample rate enabled or not
 *
 *  @return Intelligent sample rate flag
 */
bool getIntelligentSampleRate(void)
{
	const bool state = CO_intelligentSampleData.Is_Intelligent_SAMPLE_RATE_ENABLED;

	return (state);
}

/**
 * @brief  This Function will Set the  Mode of CO module
 * @param Co mode's
 */
void SetMode(const OPERATING_MODE mode_Co)
{
	CO_Status.op_mode = mode_Co;

}

/**
 * @brief  This Function will return the  Mode of Co Circuit
 *  NORMAL_MODE=0,
 *  REMOTE_ALARM,
 *  LOCAL_ALARM,
 *  SNIFFMODE,
 *  CO_TEST
 *  @return  mode value
 */
OPERATING_MODE getmode(void)
{
	return (CO_Status.op_mode);
}

/**
 * @brief  This Function will return the Raw CO.
 *  @return CO raw values
 */
uint16_t getRawCo(void)
{

	return (CO_Status.Co_RAW_reading);
}

/**
 * @brief  This Function Will Set the CO Value.(Which include Compensation )
 * @param CO Value
 */
void setCOValue(const uint32_t CO_Value)
{

	CO_Status.Co_Final_after_compensations = CO_Value;

}

/**
 * @brief  This Function will return the CO Value.(Which include Compensation )
 * @return  Co value
 */
uint32_t getCoAfterCompensation(void)
{

	return CO_Status.Co_Final_after_compensations;
}

/**
 * @brief  This Function will return CO Circuit PWM Cycle Value
 * @return  PWM Values
 */
uint32_t GetPWM_Cycle(void)
{

	return CO_Status.PWM_duty_cycle;
}

/**
 * @brief  This Function will  return LOW CO warning Type
 * @return  CO warning  Values
 */
LOW_LEVEL_CO_WARNING Get_Low_CO_Warning(void)
{

	return CO_Status.co_warning;
}

/**
 * @brief  This Function Will  Set LOW CO warning Type
 * @param state_warining ,
 * @param Low_Level_timer_start
 */
void Set_Low_Co_Warning(const LOW_LEVEL_CO_WARNING state_warining, bool Low_Level_timer_start)
{
	if (Low_Level_timer_start == false){

		CO_flags.Low_Level_timer_start = Low_Level_timer_start;

	}
	CO_Status.co_warning = state_warining;

}

/**
 * @brief  This Function will  return  CO Level
 * @return  CO level
 */
CO_LEVEL get_CO_Level(void)
{

	return CO_Status.co_level;
}

/**
 * @brief  This Function will  Acknowledge  CO Overload events (i.e High CO event)
 *  Only Alarm memory will call this function and Acknowledge the High Co event by setting
 *  co_Sensor_Overload_Occurred= False and increment the Overload count
 *
 */
void Ack_CO_Overload_events(void)
{
	CO_flags.co_Sensor_Overload_Occurred = false;
	co_Overload_Events++;
	DEBUG_CO(" Ack_CO_Overload_events", true, co_Overload_Events);
}
/**
 * @brief  This Function will  return  CO Overload Status
 * @return  CO  Overload Status
 */
bool get_CO_Overload_Status(void)
{

	return CO_flags.co_Sensor_Overload_Occurred;
}
/**
 * @brief  This Function Will  Set  CO Level
 * @param Co level
 */
void set_CO_Level(const CO_LEVEL level)
{

	CO_Status.co_level = level;
}

/**
 * @brief  This Function will return Peak value during Low Alarm
 * @return Low Level CO peak Value
 */
uint16_t GetLowLevel_CO_PeakValue(void)
{

	return CO_Status.CO_Low_Level_Peak;
}

/**
 * @brief  This Function will return Max CO  during in Alarm
 * @return Max CO Value in PPM
 */
uint16_t GetMaxAlarmCO(void)
{

	DEBUG_CO("\tMAX CO ", true, (uint32_t) Alarm_CO_Val);
	return Alarm_CO_Val;
}
/**
 * @brief  This Function will Set overload Compensation flag off
 *
 */
void SetOverLoadFlag(void)
{
  CO_flags.overload_Compensation = false;
  CO_flags.disable_Co_overload_Compensation_timer = true;
  DEBUG_CO("\t END High Comp", false, 0);
}

/**
 * @brief  This Function will Set CO Fault
 *
 */
void SetCO_Fault_(void)
{
}



/**
 * @brief  This Function will set co variance flag on base and off base
 * @param flag
 *
 */
void setoffbaseCO_variance(bool flag)
{
	CO_flags.offbase_co_variance = flag;
}


/*
 * This is an example of how to use the CRC module
 *    GPCRC_Start(GPCRC); //Start a new CRC calculation and set to initial value.
 *    for (i = 0; i < packetIndex; i++) {
 *      GPCRC_InputU8(GPCRC, packet[i]); //input each byte into the calculator
 *    }
 *    uint16_t crc = GPCRC_DataReadBitReversed(GPCRC); //read out the final CRC value
 */

/**
 * @brief Initialise the CRC peripheral
 * @details CRC configured for CRC-16 XMODEM with 0x1021 poly and 0x0000 initial value.
 * Use GPCRC_DataReadBitReversed(GPCRC) to get calculated CRC in correct format
 */
void init_CRC_co(void)
{
  /* Enable GPCRC clock */
  CMU_ClockEnable(cmuClock_GPCRC, true);
  GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;

  init.crcPoly    = CRC_POLYNOMIAL;
  init.initValue    = 0x0000FFFF;
  init.reverseBits  = true;
  init.enable     = true;
  GPCRC_Init(GPCRC, &init);
}

/**
 * @brief Calculate the CRC
 * @param CRC, data
 * @details
 *
 */
uint16_t calc_crc16(uint16_t crc,uint8_t data)
{

    crc = ((crc >> 8u) | (crc << 8u)) & 0xffff;
    crc ^= data;
    crc ^= (crc & 0xff) >> 4u;
    crc ^= ((crc << 12u) & 0xffff);
    return crc ^ ((crc & 0xff) << 5u);

}

/**
 * @brief Get the CRC
 * * @param  data
 *
 */
uint16_t get_crc_flash(uint8_t data2[])
{
  uint16_t crc;
  crc = 0x0000u;
  for (uint16_t i=0u;i<CRC_CAL_SIZE;i++)
    crc = calc_crc16(crc, data2[i]);
  return crc;
}

/**
 * @brief:  Return the Co HW fault
 * @details FTM calls this module to read the CO HW Fault status;
 * @return  coFault
 *
 */
FAULT_CIRCUIT getFTM_CO_HW_Fault(void)
{
  return coFault;
}

/**
 * @brief:  Return the Co raw data
 * @details FTM calls this module to read the CO raw data;
 * @return  CO_Status.Co_RAW_reading
 *
 */
uint16_t getFTM_CO_RawData(void)
{
  return CO_Status.Co_RAW_reading;
}


/**
 * @brief:  Return the Co After Compensation data
 * @details FTM calls this module to read the CO compensation data;
 * @return  CO_Status.Co_Final_after_compensations
 *
 */
uint16_t getFTM_CO_AfterCompData(void)
{
  return CO_Status.Co_Final_after_compensations;
}

/******************************************************************************/
void acqco_simulate_raw_co_reading( const bool sim_mode )
{
  simulate_co_raw_mode = sim_mode;
}

/******************************************************************************/
void acqco_simulated_raw_co_reading( const uint16_t co_raw )
{
  simulated_co_raw_reading = co_raw;

  simulate_co_raw_mode = true;
}

/******************************************************************************/
void acqco_simulated_ppm_co_reading( const uint16_t co_ppm )
{
  const uint16_t co_raw = co_ppm;

  simulated_co_raw_reading = co_raw;

  simulate_co_raw_mode = true;
}

/******************************************************************************/
void set_ftm_strike_count(void)
{
  strike_count_SC = 0u;
  strike_count_OC = 0u;
  strike_out_count = 0u;
}

/******************************************************************************/
void restore_co_bist_strike_count(void)
{
  strike_count_SC = NEW_SC_STRIKE_COUNT_FAULTS;
  strike_count_OC = STRIKE_COUNT_FAULTS;
  strike_out_count = CO_OUT_STRIKE_COUNT_FAULTS;
}


/**
 * Return the manufacture time
 * @return
 */
uint32_t get_manufactureDate(void)
{
  uint32_t manufactDate = date_of_manufacture;  //Get a Local Copy

  return manufactDate;
}


/**
 * @brief co_demount_init
 * @desc function to initialise the co global/static variables during remounting
 * @param none
 * @return none
 */
void co_demount_init(void)
{
	co_alarm_st = 0u;
    co_super_alarm_st = 0u;  
	CO_flags.Is_CO_Detected = false;
	CO_Status.Gain = HIGH_GAIN;
	CO_Status.op_mode = NORMAL_MODE;
	CO_Status.co_level = NO_CO;
	CO_Status.lowGainHysterisisVal = 0u;
	CO_Status.Co_RAW_reading = 0u;
	CO_Status.co_warning = NO_LOW_LEVEL_CO;
	CO_Status.COHB_Calculation = 0u;	
	CO_flags.Is_CO_ALARM_Active = false;
	CO_flags.co_Sensor_Overload = false;
	CO_flags.overload_Compensation = false;
	CO_flags.co_Sensor_Overload_Occurred = false;
    SensorShortOff();
}
