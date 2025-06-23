/*******************************************************************************
 *
 * @file    CO_Calibration.c
 *
 * @brief   CO Calibration file
 *
 * @date    26 Oct 2023
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include "user_info_page.h"
#include "CO_Calibration.h"
#include "data_logging.h"

/**
 * @brief Number of bytes for a CRC value
 */
#define CRC_BYTES (2)

/**
 * @brief Number of padding bytes
 */
#define PADDING_BYTES (1)

/**
 * @brief Size in bytes of the temperature compensation table
*/
#define TEMPERATURE_COMP_SIZE (141)

/**
 * @brief Size in bytes of the aging sf table
*/
#define AGING_SF_SIZE (60)

/**
 * @brief CO CAL default value
 * 
 * @note THIS DATA WAS TAKEN FROM CO SENSOR SERIAL NUMBER 2233470013
*/
#define CO_CAL_DEF (120U)

/**
 * @brief CO variance threshold default value
 * 
 * @note This value is used when the EEPROM has not been programmed
*/
#define CO_VAR_THRESH_DEF (171)

/**
 * @brief CO nA per ppm default value
 * 
 * This value is used when the EEPROM has not been programmed
 * 
 * @note THIS WILL PROBABLY NEED SCALING
*/
#define CO_NA_PER_PPM_DEF (8u * COCAL_NA_PER_PPM_SCALING_FACTOR)

/**
 * @brief Programming buffer size
*/
#define PROG_BUF_SIZE (256)

/**
 * @brief Swapped access to CO calibration table
 * 
 * Set this to a (1) to mangle the table, see below:
 * 
 * 0 1 2 3 4 5 6 7 ........     Normal
 * 3 2 1 0 7 6 5 4 ........     Swapped
*/
#ifndef CO_TABLES_SWAPPED
#define CO_TABLES_SWAPPED (0)
#endif

/**
 * @brief Swap macro
 * 
 * Used to insert values into default calibration tables
*/
#if CO_TABLES_SWAPPED == 0
#define SWAP(a,b,c,d) (a),(b),(c),(d)
#else
#define SWAP(a,b,c,d) (d),(c),(b),(a)
#endif

/**
 * @brief Pointer to the start of the User Data area in the FLASH
 * 
 * @note This is accessed on a byte basis
*/
static const uint8_t * user_data_ptr = ( uint8_t * )USERDATA_BASE;

/**
 * @brief Default calibration data table
 * 
 * The table consists of temperature compenstation and aging sf data. It is used
 * when the table in the User Data area of the flash is missing or corrupt.
 * 
 * The size of the temperature compensation table is 141.
 * The size of the aging compensation table is 60.
 * 
 * The table has a CRC at the end which is manually calculated.
 * 
 * @note THIS DATA WAS TAKEN FROM CO SENSOR SERIAL NUMBER 2233470013
 * 
 * @warning The byte order of the table is swapped depending on the value of CO_TABLES_SWAPPED
*/
static const uint8_t def_calibration_data[ ] = \
{
  /*       1     2     3     4             5     6     7     8             9    10    11    12            13    14    15    16               */
  /* TEMPRATURE @ 21.2 DegC ------------------------------------------------------------------------------------------------->               */
  SWAP( 0xF3, 0xF1, 0xEF, 0xEE ), SWAP( 0xED, 0xEB, 0xE9, 0xE7 ), SWAP( 0xE5, 0xE3, 0xE1, 0xE0 ), SWAP( 0xDE, 0xDC, 0xDA, 0xD8 ),     /* 16  */  
  SWAP( 0xD6, 0xD5, 0xD3, 0xD2 ), SWAP( 0xD0, 0xCE, 0xCC, 0xCA ), SWAP( 0xC8, 0xC6, 0xC5, 0xC3 ), SWAP( 0xC1, 0xBF, 0xBD, 0xBC ),     /* 32  */
  SWAP( 0xBA, 0xB9, 0xB7, 0xB5 ), SWAP( 0xB3, 0xB1, 0xAF, 0xAD ), SWAP( 0xAC, 0xAA, 0xA8, 0xA6 ), SWAP( 0xA4, 0xA3, 0xA1, 0xA0 ),     /* 48  */
  SWAP( 0x9E, 0x9C, 0x9A, 0x98 ), SWAP( 0x96, 0x94, 0x93, 0x91 ), SWAP( 0x8F, 0x8D, 0x8B, 0x8A ), SWAP( 0x88, 0x87, 0x85, 0x83 ),     /* 64  */
  SWAP( 0x81, 0x7F, 0x7D, 0x7B ), SWAP( 0x79, 0x78, 0x76, 0x74 ), SWAP( 0x72, 0x71, 0x6F, 0x6D ), SWAP( 0x6C, 0x6A, 0x68, 0x66 ),     /* 80  */
  SWAP( 0x64, 0x62, 0x60, 0x5F ), SWAP( 0x5D, 0x5B, 0x59, 0x58 ), SWAP( 0x58, 0x58, 0x58, 0x57 ), SWAP( 0x56, 0x56, 0x55, 0x55 ),     /* 96  */
  SWAP( 0x54, 0x53, 0x53, 0x52 ), SWAP( 0x52, 0x51, 0x50, 0x50 ), SWAP( 0x4F, 0x4F, 0x4E, 0x4D ), SWAP( 0x4D, 0x4C, 0x4B, 0x4B ),     /* 112 */
  SWAP( 0x4A, 0x4A, 0x49, 0x48 ), SWAP( 0x48, 0x47, 0x47, 0x46 ), SWAP( 0x45, 0x45, 0x44, 0x44 ), SWAP( 0x43, 0x42, 0x42, 0x41 ),     /* 128 */
  SWAP( 0x41, 0x40, 0x3F, 0x3F ), SWAP( 0x3F, 0x3F, 0x3E, 0x3D ), SWAP( 0x3D, 0x3C, 0x3C, 0x3B ), SWAP( 0x3A, 0xC1, 0xBE, 0xBC ),     /* 144 */
  /* TEMPRATURE @ 21.2 DegC ------------------------------------------------------------------------------->  AGING SF ------>               */
  SWAP( 0xBA, 0xB8, 0xB6, 0xB4 ), SWAP( 0xB2, 0xB0, 0xAF, 0xAD ), SWAP( 0xAC, 0xA8, 0xA4, 0xA1 ), SWAP( 0x9F, 0x9C, 0x9A, 0x98 ),     /* 160 */
  SWAP( 0x96, 0x94, 0x93, 0x91 ), SWAP( 0x8F, 0x8E, 0x8D, 0x8C ), SWAP( 0x8A, 0x89, 0x88, 0x87 ), SWAP( 0x86, 0x85, 0x84, 0x84 ),     /* 176 */
  SWAP( 0x83, 0x82, 0x81, 0x80 ), SWAP( 0x80, 0x7F, 0x7E, 0x7E ), SWAP( 0x7D, 0x7C, 0x7C, 0x7B ), SWAP( 0x7B, 0x7A, 0x7A, 0x79 ),     /* 192 */
  SWAP( 0x79, 0x78, 0x78, 0x77 ), SWAP( 0x77, 0x76, 0x76, 0x75 ), SWAP( 0x75, 0xF7, 0x26, 0x00 )                                      /* 204 */
  /* AGING SF ------------------------------------------------------------->  CRC   CRC   PADDING                                            */
};

/**
 * @brief Programming buffer
*/
static uint8_t prog_buff[ PROG_BUF_SIZE ];

/**
 * @brief In-use na per ppm value
*/
static volatile uint16_t co_na_per_ppm = 1800;
/**
 * @brief In-use CO CAL value
 * 
 * @warning THIS IS NO LONGER USED, BUT BE PRESENT FOR REFERENCE
*/
static uint16_t co_cal = CO_CAL_DEF;

/**
 * @brief In-use CO variance value
*/
static uint16_t co_var_thresh = CO_VAR_THRESH_DEF;

/**
 * @brief In-use CO calibration temperature value
*/
static volatile uint8_t co_calib_temperature = COCAL_CALIB_TEMP_CODE;

/**
 * @brief In-use CO calibration humidity value
*/
static volatile uint8_t co_calib_humidity = COCAL_CALIB_HUMIDITY;

/**
 * @brief CRC status when last checked
*/
static bool crc_ok = false;

/*******************************************************************************
 * @brief   Get corrected index
 *
 * @details This function returns the corrected index value depending whether or
 *          not the table is byte swapped
 * 
 * @param[in] index    Requested table index
 * 
 * @see CO_TABLES_SWAPPED
 * 
 * @return Corrected table index
*/
static uint8_t get_index( uint8_t index )
{
#if CO_TABLES_SWAPPED == 1
  const size_t block_start = index - ( index % 4 );
  index = block_start + ( 3 - ( index % 4 ) );
#endif

  return( index );
}

/*******************************************************************************
 * @brief   Put item into table
 *
 * @details This function puts the given item into the specified table
 * 
 * @param[in] ptr    Pointer to table
 * @param[in] item   Logical item index
 * @param[in] value  Value to write
 * 
 * @return True if written, otherwise false
*/
static bool put_item( uint8_t * const ptr, const uint8_t item, const uint8_t value )
{
  bool ok = false;

  const size_t table_size = sizeof( def_calibration_data );

  size_t index = get_index( item );

  if( index < table_size )
  {
    ptr[ index ] = value;

    ok = true;
  }
  
  return( ok );
}

/*******************************************************************************
 * @brief   Get item from table
 *
 * @details This function gets the given item from the specified table
 * 
 * @param[in] ptr    Pointer to table
 * @param[in] item   Logical item index
 * 
 * @note Zero is returned if the item index is invalid
 * 
 * @return Table item
*/
static uint8_t get_item( const uint8_t * const ptr, uint8_t item )
{
  const size_t table_size = sizeof( def_calibration_data );

  size_t index = get_index( item );

  return( index >= table_size ? 0U : ptr[ index ] );
}

/*******************************************************************************
 * @brief   Calculate 16 CRC value
 *
 * @details This function calculates a new CRC value given the current CRC value
 *          and the new data item to CRC
 * 
 * @param[in] arr   Pointer to the area
 * @param[in] sz    Size of the area in bytes
 * 
 * @return Updated CRC value
*/
static uint16_t calc_crc16( uint16_t crc, const uint8_t data )
{
    crc = ( ( crc >> 8u ) | ( crc << 8u ) ) & 0xffff;
    crc ^= data;
    crc ^= ( crc & 0xff ) >> 4u;
    crc ^= ( ( crc << 12u ) & 0xffff );

    return crc ^ ( ( crc & 0xff ) << 5u );
}

/*******************************************************************************
 * @brief   Calculate 16 CRC buffer value
 *
 * @details This function calculates a new CRC value for the spefied area
 * 
 * @param[in] ptr   Pointer to table
 * @param[in] len   Size of the area in bytes
 * 
 * @return Calculated CRC value
*/
static uint16_t calc_crc( const uint8_t * const ptr, const size_t len )
{
  uint16_t crc = 0x0000u;

  for( uint32_t i = 0UL; i< len ; i++ )
  {
    const uint8_t v = get_item( ptr, i );

    crc = calc_crc16( crc, v );
  }

  return crc;
}

/******************************************************************************/
bool cocal_check_def_crc( void )
{
  const size_t len = sizeof( def_calibration_data ) - PADDING_BYTES;

  const uint16_t crc = calc_crc( def_calibration_data, len );

  return( crc == 0U );
}

/******************************************************************************/
bool cocal_check_crc( void )
{
  const size_t len = sizeof( def_calibration_data ) - PADDING_BYTES;

  const uint16_t crc = calc_crc( user_data_ptr, len );

  crc_ok = ( crc == 0U );

  return( crc_ok );
}

/******************************************************************************/
bool cocal_progam_defaults( const bool force )
{
  bool ok = cocal_check_def_crc( );

  if( ok )
  {
    bool program = force;

    const size_t len = sizeof( def_calibration_data );

    if( program == false )
    {
      const bool blank = uip_is_region_erase( 0ul, len );

      if( blank )
      {
        program = true;
      }
    }

    if( program )
    {
      ok = uip_program( 0, def_calibration_data, len );

      if( ok )
      {
        co_na_per_ppm = CO_NA_PER_PPM_DEF;

        co_cal = CO_CAL_DEF;

        co_calib_temperature = COCAL_CALIB_TEMP_CODE;

        co_calib_humidity = COCAL_CALIB_HUMIDITY;

        data_logging_set_na_per_ppm( co_na_per_ppm );
        DataLogging_SetCOCal( co_cal );
        DataLogging_SetCOCalibTemperature( co_calib_temperature );
        DataLogging_SetCOCalibHumidity( co_calib_humidity );

        const bool eeprom_ok = data_logging_is_eeprom_ok( );

        /* Do not re-validate CRC if there is exisiting corruption */
        if( eeprom_ok )
        {
          DataLogging_SetCRC( );
        }
      }
    }
  }

  return( ok );
}

/******************************************************************************/
bool cocal_update_prog_buf_ascii( const uint8_t * src )
{
  bool ok = false;

  /* Next free position in the programming buffer */
  static uint8_t pos = 0U;

  /* Is this the start of a new update? */
  if( src == NULL )
  {
    pos = 0U;   /* Yes, go to start of the buffer */

    ok = true;
  }
  else
  {
    /* Buffer is ascii so we can dtermine it's length */
    const size_t len = strlen( ( const char * )src );

    /* Data provided must be an even length, i.e 2 ascii characters per byte */
    if( ( len % 2 ) != 0 )
    {
      return( ok );
    }

    /* Process input ascii character stream */
    for( uint8_t i = 0; i < len; i += 2, pos++ )
    {
      /* This is a buffer for strtol, NULL terminated */
      const uint8_t buf[ 3 ] = { src[ i ], src[ i + 1 ], 0 };

      /* Get binary value from buffer */
      const uint8_t value = strtol( ( const char * )buf, NULL, 16 );

      /* Add value to the programming buffer */
      ok = put_item( prog_buff, pos, value );
    }
  }

  return( ok );
}

/******************************************************************************/
bool cocal_program_calib( void )
{
  bool ok = false;

  /* Get size of the calibration tables including the crc bytes and padding */
  const size_t sz = sizeof( def_calibration_data );

  /* Check if the data in the programming buffer is ok, excluding padding bytes */
  const uint16_t crc = calc_crc( prog_buff, sz - PADDING_BYTES );

  /* Buffer contents are ok when crc is zero, i.e crc check includes the crc itself */
  if( crc == 0U )
  {
    /* Program flash with new calibration tables */
    ok = uip_program( 0, prog_buff, sz );
  }

  /* Return status to caller */
  return( ok );
}

/******************************************************************************/
uint8_t cocal_get_temperature_comp( const uint8_t item )
{
  uint8_t value = 0u;

  if( crc_ok )
  {
    value = ( item >= TEMPERATURE_COMP_SIZE ? 0U : get_item( user_data_ptr, item ) );
  }

  return( value );
}

/******************************************************************************/
uint8_t cocal_get_aging_sf( const uint8_t item )
{
  uint8_t value = 0u;

  if( crc_ok )
  {
    value = ( get_item( user_data_ptr, TEMPERATURE_COMP_SIZE + item ) );
  }

  return( value );
}

/******************************************************************************/
void cocal_read_eeprom_values( void )
{
  uint16_t value;

  value = data_logging_get_na_per_ppm( );
  
  co_na_per_ppm = ( ( ( value != 0xFFFFU ) && ( value != 0U ) ) ? value : co_na_per_ppm );

  value = DataLogging_GetCOCal( );
  
  co_cal = ( ( ( value != 0xFFFF ) && ( value != 0U ) ) ? value : co_cal );

  value = DataLogging_GetCOCalibTemperature( );
  
  co_calib_temperature = ( ( ( uint8_t )value != 0xFF ) ? ( uint8_t )value : co_calib_temperature );

  value = DataLogging_GetCOCalibHumidity( );
  
  co_calib_humidity = ( ( ( ( uint8_t )value != 0xFF ) && ( (uint8_t)value != 0U ) ) ? ( uint8_t )value : co_calib_humidity );
}

/******************************************************************************/
uint16_t cocal_get_co_na_per_ppm( void )
{
  return( co_na_per_ppm );
}

/******************************************************************************/
uint16_t cocal_get_co_cal( void )
{
  return( co_cal );
}

/******************************************************************************/
uint16_t cocal_get_co_var_thresh( void )
{
  return( co_var_thresh );
}

/******************************************************************************/
uint16_t cocal_get_co_calib_temperature( void )
{
  const uint16_t value = ( ( co_calib_temperature + 200U ) * 10U );

  return( value );
}

/******************************************************************************/
uint8_t cocal_get_co_calib_humidity( void )
{
  return( co_calib_humidity );
}

/******************************************************************************/
void cocal_init( void )
{
  /* Get eeprom integrity? */
  const bool eeprom_ok = data_logging_is_eeprom_ok( );

  /* Can eeprom be trusted? */
  if( eeprom_ok )
  {
    cocal_read_eeprom_values( );
  }
}

/******************************************************************************/
