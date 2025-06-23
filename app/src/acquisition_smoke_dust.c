/*******************************************************************************
 *
 * @file    degraded_chamber
 *
 * @brief   degraded chamber source file
 *
 * @date    15 Aug 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include "acquisition_Smoke.h"
#include "acquisition_smoke_dust.h"
#include "data_logging.h"
#include "fault_handler.h"
#include "hal_BURTCTimer.h"

/*******************************************************************************
 * @brief Smoke rolling average array initialisation value
 */
#define SMOKE_ROLLING_AVERAGE_INIT (0xFFFF)

/*******************************************************************************
 * @brief Maximum number of rolling averages per day
 */
#define SMOKE_ROLLING_AVERAGES_DAY_MAX (24UL)

/*******************************************************************************
 * @brief Maximum number of rolling averages per week
 */
#define SMOKE_ROLLING_AVERAGES_WEEK_MAX (SMOKE_ROLLING_AVERAGES_DAY_MAX*(7UL))

/*******************************************************************************
 * @brief Initial value for dust correction factor
 *
 * @note This value perform no correction
 */
#define SMOKE_CORR_FACTOR_VALUE_INIT (100UL)

/*******************************************************************************
 * @brief Dust correction time
 *
 * @note This value specfices the dust correction time of 1 hour
 */

#define DUST_CORR_HOUR ( ( 60 * 60 ) / ( SMOKE_MEASUREMENT * BURTC_PERIOD ) )


/*******************************************************************************
 * @brief Dust correction time
 *
 * @note This value specfices the dust correction time of 1 minute
 * 
 * @warning FOR TESTING ONLY
 */
#define DUST_CORR_MINUTE ( 60 / ( SMOKE_MEASUREMENT * BURTC_PERIOD ) )

/*******************************************************************************
 * @brief Dust correction time
 *
 * @note This value specfices the dust correction time of 10 seconds
 * 
 * @warning FOR TESTING ONLY
 */
#define DUST_CORR_ALWAYS ( 1 )

/*******************************************************************************
 * @brief Correction factor minimum limit
 * 
 * @note This is really 0.8
 */
#define SMOKE_CORR_FACTOR_MIN (80U)

/*******************************************************************************
 * @brief Correction factor maximum limit
 * 
 * @note This is really 1.2
 */
#define SMOKE_CORR_FACTOR_MAX (120U)

/*******************************************************************************
 * @brief Dust correction threshold
 * 
 * @note This is the percentage limit that when the smoke corrected 24 hr
 *       average changes by to trigger a calculation of the new dust correction
 *       factor
 */
#define SMOKE_DUST_CORR_CALC_THRESHOLD (5)

/*******************************************************************************
 * @brief Cal reflection threshold limit
 */
#define CAL_REFLECTION_THRESHOLD_LIMIT (200)

/*******************************************************************************
 * @brief Dust speed testing
 * 
 * Set to (1) to enable
 */
#define SMOKE_DUST_SPEEDTESTING (0U)

/*******************************************************************************
 * @brief Dust minor fault counter
 *
 */
#define SMOKE_DUST_STRIKE_COUNT (3U)


/*******************************************************************************
 * @brief Average value structure
 */
typedef struct
{
  uint16_t    ave;          /**< Average value */
  uint16_t    samples;      /**< Number of samples in the total */
  uint32_t    total;        /**< Summation of all values */
}
AVE_VALUE;

/*******************************************************************************
 * @brief Average array structure
 */
typedef struct
{
  uint16_t    ave;          /**< Average value of all items in the array */
  uint16_t    pos;          /**< Position of next free location */
  uint16_t    sz;           /**< Maximum number of items in the buffer */
  uint16_t  * buf;          /**< Pointer to the array storage */
}
AVE_ARRAY;

/*******************************************************************************
 * @brief Array storage for smoke value weekley averages
 */
static uint16_t smoke_average_week_buf[ SMOKE_ROLLING_AVERAGES_WEEK_MAX ];

/*******************************************************************************
 * @brief Array for smoke raw weekley averages
 * 
 * @note Values are saved every hour
 */
static AVE_ARRAY smoke_average_week_arr = { 0U, 0U, SMOKE_ROLLING_AVERAGES_WEEK_MAX, smoke_average_week_buf };

/*******************************************************************************
 * @brief Dust correction factor
 * 
 * @note Factor is multiplied by 100, 0.8 to 1.2 becomes 80 to 120
 */
static uint32_t dust_corr_factor;

/*******************************************************************************
 * @brief Cal reflection threshold
 */
static uint32_t cal_reflection_threshold;

/*******************************************************************************
 * @brief Dust correction test mode
 */
static bool test_mode;

/*******************************************************************************
 * @brief Dust correction raw test value
 */
static uint32_t smoke_raw_test_value = 3885;

/*******************************************************************************
 * @brief Dust correction dark test value
 */
static uint32_t cal_reflection_test_threshold = 4250;

/*******************************************************************************
 * @brief Dust correction minor fault status
 */
static bool chamber_minor_fault_status = false;

/*******************************************************************************
 * @brief   Inialise array object
 *
 * This function initialises the specfied array object for use.
 *
 * @param[in,out]   arr       Array to initialse
 * @param[in]       value     Initialisation value
 */
static void init_array( AVE_ARRAY * const arr, const uint16_t value )
{
  arr->pos = arr->ave = 0U;

  for( uint16_t i = 0; i < arr->sz; i++ )
  {
    arr->buf[ i ] = value;
  }
}

/*******************************************************************************
 * @brief   Average a value
 *
 * This function averages a new value specified by the average object. When the 
 * reset flag is set, the overall average is calculated and the samples and total
 * set to zero.
 *
 * @param[in,out]   item    Average object
 * @param[in]       val     New value to average
 * @param[in]       reset   Reset
 *
 * @return Average before reset
 */
static uint16_t average_value( AVE_VALUE * const item, const uint16_t val, const bool reset )
{
  item->total += val;
  item->samples++;

  item->ave = item->total / item->samples;

  if( reset )
  {
    item->samples  = 0U;
    item->total    = 0UL;
  }

  return( item->ave );
}

/*******************************************************************************
 * @brief Average values in an array
 *
 * This function adds a new value to the specfied array and then calculates the
 * average of all items contained in the array. The full flag, if required, is
 * set to true if the array containes a complete set of averages.
 * 
 * @param[in,out]   arr    Array object
 * @param[in]       val    New value to average
 * @param[out]      full   Full flag
 *
 * @note Values of SMOKE_ROLLING_AVERAGE_INIT are ignored
 *
 * @see SMOKE_ROLLING_AVERAGE_INIT
 *
 * @return Array average
 */
static uint32_t average_array( AVE_ARRAY * const arr, const uint16_t val, bool * full )
{
  uint32_t average = 0UL;     /* Array average */
  uint32_t samples = 0UL;     /* Number of valid samples found */

  /* Ful flasg required? */
  if( full )
  {
    /* Yes, default is full */
    *full = true;
  }

  /* Add new value to buffer */
  arr->buf[ arr->pos++ ] = val;

  /* Is buffer full? */
  if( arr->pos == arr->sz )
  {
    /* Wrap */
    arr->pos = 0U;
  }

  /* Examine all array items */
  for( uint16_t i = 0; i < arr->sz; i++ )
  {
    /* Only average legal values */
    if( arr->buf[ i ] != SMOKE_ROLLING_AVERAGE_INIT )
    {
      average += arr->buf[ i ];     /* Totals */

      samples++;                    /* Samples */
    }
    else if( full )     /* Does caller want this infomation? */
    {
      *full = false;    /* Array has empty items */
    }
    else
    {
      // Nothing
    }
  }

  /* When there are valid average values ... */
  if( samples > 0UL )
  {
    /* Calculate average */
    average = average / samples;
  }

  /* Save arrage */
  arr->ave = average;

  /* Return to caller */
  return( arr->ave );
}

/*******************************************************************************
 * @brief Calculate dust correction factor
 *
 * This function given the provided smoke average and calibrated reflection
 * threshold calculates a dust correction factor. The factor is clipped using
 * SMOKE_CORR_FACTOR_MIN and SMOKE_CORR_FACTOR_MAX limits.
 *
 * @param smoke_average                 Smoke average
 *
 * @see SMOKE_CORR_FACTOR_MIN SMOKE_CORR_FACTOR_MAX
 *
 * @note Calculated factor is multiplied by 100
 *
 * @return Dust correction factor
 */
static uint32_t calc_dust_corr_factor( const uint32_t smoke_average )
{
  /* The correction factor is multiplied by 100 */
  uint32_t cf = ( cal_reflection_threshold * 100 ) / smoke_average;

  if( cf < SMOKE_CORR_FACTOR_MIN )
  {
    cf = SMOKE_CORR_FACTOR_MIN;
  }
  else if( cf > SMOKE_CORR_FACTOR_MAX )
  {
    cf = SMOKE_CORR_FACTOR_MAX;
  }
  else
  {
    /* Do nothing */
  }

  return( cf );
}

/*******************************************************************************
 * @brief Apply a dust correction factor
 *
 * This function takes a raw smoke value and applies the given dust correction
 * factor.
 *
 * @param smoke_raw_value       Raw smoke value
 * @param dust_corr_factor      Dust correction factor
 *
 * @return Corrected smoke value
 */
static uint32_t apply_dust_corr_factor( const uint32_t smoke_raw_value,
                                        const uint32_t dust_corr_factor )
{
  /* Using the dust correction factor correct the raw smoke value */
  const uint32_t smoke_raw_corr = ( smoke_raw_value * dust_corr_factor ) / 100UL;

  /* Resulting corrected raw smoke value */
  return( smoke_raw_corr );
}

static uint32_t get_run_rate( void )
{
#if SMOKE_DUST_SPEEDTESTING == 1
  const uint32_t speed_rate = DUST_CORR_MINUTE;
#else
  const uint32_t speed_rate = DUST_CORR_HOUR;
#endif  
  return( speed_rate );
}

/*******************************************************************************
 * @brief Is there no dust
 *
 * This function takes does a crude check to see if no dust is present by
 * checking the given smoke value in relation to the clean air limit.
 * 
 * @note This is used to determine sudden/large dust removal 
 *
 * @param smoke_raw_value       Raw smoke value
 *
 * @return True if there is no or insignifcant dust
 */
static bool is_no_dust_present( const uint32_t smoke_raw_value )
{
  const uint32_t limit = cal_reflection_threshold + CAL_REFLECTION_THRESHOLD_LIMIT;

  return( smoke_raw_value <= limit );
}

/******************************************************************************/
uint32_t smoke_dust_perform_correction( uint32_t smoke_raw_value )
{
  /* Smoke raw value average, this is averaged over an hour */
  static AVE_VALUE smoke_hour_average;

  /* Number of time function has been called */
  static uint32_t call_count;

  /* Number of hours since last change in dust correction factor */
  static uint8_t hours_since_last_change;

  /* Last smoke average */
  static uint32_t last_smoke_average;

#if SMOKE_DUST_SPEEDTESTING == 1
  static uint32_t smoke_raw_value_fixed;

  if( cal_reflection_threshold == 0UL )
  {
    cal_reflection_threshold = smoke_raw_value;
  }
#endif

  /* First time? */
  if( last_smoke_average == 0UL )
  {
    last_smoke_average = cal_reflection_threshold;
  }

#if SMOKE_DUST_SPEEDTESTING == 1
  static uint32_t dust_sim      = 0UL;
  static uint32_t dust_sim_inc  = 0UL;

  uint32_t smoke_raw_reading = smoke_raw_value;

  if( dust_sim_inc == 0UL )
  {
    smoke_raw_reading = smoke_raw_value = smoke_raw_value_fixed;
  }
  else
  {
    smoke_raw_value += dust_sim;
  }
#endif

  /* Keep track of number of times we have been called */
  call_count++;

  /* Is there some dust present? */
  const bool dust_not_present = is_no_dust_present( smoke_raw_value );

  /* When there no dust or there was dust and it's now removed, the corrected smoke value
     we will restart the 7 day dust tracking. */
  if( ( dust_corr_factor != SMOKE_CORR_FACTOR_VALUE_INIT ) && dust_not_present )
  {
    dust_corr_factor = SMOKE_CORR_FACTOR_VALUE_INIT;

    last_smoke_average = cal_reflection_threshold;

    init_array( &smoke_average_week_arr, SMOKE_ROLLING_AVERAGE_INIT );

    DEBUG_SMOKE_DUST("\nDUST CORRECTION - DUST REMOVED", false, 0u);
  }

  /* Using the current dust correction factor, correct the raw smoke value */
  uint32_t smoke_raw_corr = apply_dust_corr_factor( smoke_raw_value, dust_corr_factor );

  DEBUG_SMOKE_DUST("\nDUST CORRECTION", false, 0u);

#if SMOKE_DUST_SPEEDTESTING == 1
  DEBUG_SMOKE_DUST("\nRaw Smoke Reading:         ", true, smoke_raw_reading);
#endif
  DEBUG_SMOKE_DUST("\nRaw Smoke Value:           ", true, smoke_raw_value);
  DEBUG_SMOKE_DUST("\nReflection threshold:      ", true, cal_reflection_threshold );
  DEBUG_SMOKE_DUST("\nDust Correction Factor:    ", true, dust_corr_factor);
  DEBUG_SMOKE_DUST("\nCorrected Smoke Value:     ", true, smoke_raw_corr);

  const uint32_t dust = ( smoke_raw_value > cal_reflection_threshold ? smoke_raw_value - cal_reflection_threshold : 0UL );

#if SMOKE_DUST_SPEEDTESTING == 1
  DEBUG_SMOKE_DUST("\nDust Value:                ", true, dust);
  //DEBUG_SMOKE_DUST("\nDust Sim:                  ", true, dust_sim);

  dust_sim += dust_sim_inc;
#endif

  /* Get rate value */
  const uint32_t run_rate = get_run_rate( );

  /* An hour is up when we have been called the specfied number of times */
  const bool hour = ( call_count >= run_rate );

  /* Have we another hours worth of samples? */  
  if( hour )
  {
    /* Set to true when we have at least 7 days rolling average values */
#if SMOKE_DUST_SPEEDTESTING == 1
    bool have_week_smoke_averages = true;
#else
    bool have_week_smoke_averages = false;
#endif
    /* Get current smoke raw average value and then start a new average */
    const uint16_t smoke_hour_average_value = average_value( &smoke_hour_average, smoke_raw_value, hour );

    /* Each smoke corrected hour average is kept for the 7 days */
#if SMOKE_DUST_SPEEDTESTING == 1
    const uint16_t smoke_week_average_value = average_array( &smoke_average_week_arr, smoke_hour_average_value, NULL );
#else
    const uint16_t smoke_week_average_value = average_array( &smoke_average_week_arr, smoke_hour_average_value, &have_week_smoke_averages );
#endif

    DEBUG_SMOKE_DUST("\nDUST CORRECTION - HOUR CHECK", false, 0u);

    DEBUG_SMOKE_DUST("\nSmoke Week Average:        ", true, smoke_week_average_value);
    DEBUG_SMOKE_DUST("\nSmoke Hour Average:        ", true, smoke_hour_average_value);

    /* Have week enough averages to perform dust correction? */
    if( have_week_smoke_averages || test_mode )
    {
      hours_since_last_change++;

      const bool day = ( hours_since_last_change == SMOKE_ROLLING_AVERAGES_DAY_MAX );

      const int32_t change = ( ( smoke_hour_average_value * 100U ) / last_smoke_average ) - 100L;

      const uint32_t change_abs = abs( change );

      if( change > 0 )
      {
        DEBUG_SMOKE_DUST("\nSmoke Hour Ave Change Up:  ", true, change_abs);
      }
      else if( change < 0 )
      {
        DEBUG_SMOKE_DUST("\nSmoke Hour Ave Change Down:", true, change_abs);
      }
      else
      {
        DEBUG_SMOKE_DUST("\nSmoke Hour Ave Change None:", true, change_abs);
      }

      DEBUG_SMOKE_DUST("\nSmoke Hour Ave Last:       ", true, last_smoke_average);

      /* Need to re-calculated dust correction factor? */
      if( day || ( change > SMOKE_DUST_CORR_CALC_THRESHOLD ) )
      {
        /* Using the average smoke dark reading over the week and the calibrated reflection threshold, calculate the new dust correction factor */
        dust_corr_factor = calc_dust_corr_factor( smoke_week_average_value );

        last_smoke_average = smoke_hour_average_value;

        DEBUG_SMOKE_DUST("\nDust Re-Corr Factor:       ", true, dust_corr_factor );

#if SMOKE_DUST_SPEEDTESTING == 1
        //dust_sim = 0;
        dust_sim_inc = 0;
        smoke_raw_value_fixed = smoke_raw_value;
#endif
        hours_since_last_change = 0u;
      }

      DEBUG_SMOKE_DUST("\nRe-calculation hours:      ", true, SMOKE_ROLLING_AVERAGES_DAY_MAX - hours_since_last_change);
    }

    /* Reset to so we wait for another hour */
    call_count = 0;
  }
  else
  {
    /* Keep average of the raw smoke value */
    ( void )average_value( &smoke_hour_average, smoke_raw_value, hour );
  }

  /* Resulting corrected raw smoke value */
  return( smoke_raw_corr );
}

/******************************************************************************/
void smoke_dust_handle_chamber_errors( const uint32_t smoke_level_corr)
{
  /* Chamber is initially good */
  static bool degraded = false;
  static uint8_t cham_minor_fault_counter = 0U;

  /* read the current fault status */
    uint32_t Faults = FaultHandler_GetFaultFlags();

  /* !!! This limit is based on ST630, but this needs confirming for the P0200 !!! */
  const uint32_t limit = cal_reflection_threshold / 4UL;

  /* Is smoke chamber degraded? */
  if( smoke_level_corr < limit )
  {
	  if ((Faults & DEF_DEG_SMOKE_CHAMBER_FAULT) == 0u)
	  {
	  	 /* Yes, but is it currently identified as degraded? */
      	 cham_minor_fault_counter++;
      	 set_cham_minor_fault(true);
         if(cham_minor_fault_counter >= SMOKE_DUST_STRIKE_COUNT)
         {
            if( degraded == false )
           {
              /* No, set fault */
              FaultHandler_FaultSet( DegradedSmokeChamberFault );
              /* ... and save in log */
              DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_CHAM_CONT_START, NULL );
              /* Its now degraded */
              degraded = true;
              DataLogging_SetMinorFault(FaultDegradedSmokeChamber, degraded);
           }
         }
	  }
   }
  else  /* No, chamber is now ok */
  {
      cham_minor_fault_counter = 0U;
      set_cham_minor_fault(false);
      /* Was it degraded? */
      if( degraded == true )
      {
          /* Yes, clear fault condition */
          FaultHandler_FaultClear( DegradedSmokeChamberFault );
          /* ... and save in log */
          DataLogging_SetEventLogbookRecord( DEF_LBE_SMOKE_CHAM_CONT_END, NULL );
          /* Its now not degraded */
          degraded = false;
          DataLogging_SetMinorFault(FaultDegradedSmokeChamber, degraded);
      }
   }
}

/******************************************************************************/
void smoke_dust_init( const uint32_t threshold )
{
  cal_reflection_threshold = threshold;

  dust_corr_factor = SMOKE_CORR_FACTOR_VALUE_INIT;

  init_array( &smoke_average_week_arr, SMOKE_ROLLING_AVERAGE_INIT );

  // FOR TESTING ONLY !!!
  // smoke_dust_set_test_mode( true );
}

/******************************************************************************/
uint8_t smoke_dust_get_correction_factor( void )
{
  uint8_t dust_factor = dust_corr_factor;

  if(dust_corr_factor > SMOKE_CORR_FACTOR_MAX)
  {
      dust_factor = SMOKE_CORR_FACTOR_MAX;
  }

  if(dust_corr_factor < SMOKE_CORR_FACTOR_MIN)
  {
      dust_factor = SMOKE_CORR_FACTOR_MIN;
  }

  if(dust_factor > 100U)
  {
      dust_factor = (( dust_factor - 100U) * 5U);
  }
  else
  {
      dust_factor = ((100U - dust_factor) * 5U);
  }

  return( ( uint8_t )dust_factor );
}

/******************************************************************************/
bool smoke_dust_is_test_mode( void )
{
  return( test_mode );
}

/******************************************************************************/
void smoke_dust_set_test_mode( const bool mode )
{
  test_mode = mode;
}

/******************************************************************************/
uint32_t smoke_dust_get_raw_test_value( void )
{
  return( smoke_raw_test_value );
}

/******************************************************************************/
void smoke_dust_set_raw_test_value( const uint32_t value )
{
  smoke_raw_test_value = value;
}

/******************************************************************************/
uint32_t smoke_dust_get_calref_test_value( void )
{
  return( cal_reflection_test_threshold );
}

/******************************************************************************/
void smoke_dust_set_calref_test_value( const uint32_t value )
{
  cal_reflection_test_threshold = value;
}

/******************************************************************************/
void set_cham_minor_fault(bool status)
{
  chamber_minor_fault_status = status;
}

/******************************************************************************/
bool get_cham_minor_fault(void)
{
  return chamber_minor_fault_status;
}

