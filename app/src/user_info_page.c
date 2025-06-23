/*******************************************************************************
 *
 * @file    user_info_page.c
 *
 * @brief   User info page file
 *
 * @date    16 Feb 2024
 *
 * @author  Roger Amstell
 *
 ******************************************************************************/

#include "os.h"

#include "user_info_page.h"

#include "em_msc.h"

/*******************************************************************************
 * @brief Programming mutex
 */
static OS_MUTEX mutex;

/******************************************************************************/
/* Forward declarations */

static void save_to_prog_buf( void );
static void lock( void );
static void unlock( void );

/*******************************************************************************
 * @brief Pointer to the start of the User Data area in the FLASH
 * 
 * @note This is accessed on a byte basis
*/
static const uint8_t * user_data_ptr = ( uint8_t * )USERDATA_BASE;

/*******************************************************************************
 * @brief Programming buffer
*/
static uint8_t prog_buf[ UIP_SIZE ];

/*******************************************************************************
 * @brief   Save contents of information page
 *
 * @details This function reads contents of in-use information page into the
 *          internal programming buffer.
 */
static void save_to_prog_buf( void )
{
  memcpy( prog_buf, user_data_ptr, sizeof( prog_buf ) );
}

/*******************************************************************************
 * @brief   Lock programming
 *
 * @details This function locks programming for exclusive use
 */
static void lock( void )
{
  RTOS_ERR err;

  OSMutexPend( &mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

/*******************************************************************************
 * @brief   Unlock programming
 *
 * @details This function frees up the programming for other tasks
 */
static void unlock( void )
{
  RTOS_ERR err;

  OSMutexPost( &mutex, OS_OPT_POST_NONE, &err);
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

/******************************************************************************/
void uip_init( void )
{
  RTOS_ERR err;

  /* Create programming mutex */
  OSMutexCreate( &mutex, UIP_MUTEX_NAME, &err );
  APP_RTOS_ASSERT_DBG( ( RTOS_ERR_CODE_GET( err ) == RTOS_ERR_NONE ), 1 );
}

/******************************************************************************/
uint32_t uip_get_size( void )
{
  return( ( uint32_t )UIP_SIZE );
}

/******************************************************************************/
bool uip_is_region_erase( uint32_t start, const uint32_t len )
{
  bool erased = true;

  lock( );

  start %= UIP_SIZE;

  uint32_t end = start + len;

  if( end > UIP_SIZE )
  {
    end = UIP_SIZE;
  }

  const size_t sz = ( size_t )UIP_SIZE;

  for( size_t i = start; ( i < end ) && erased; i++ )
  {
    if( user_data_ptr[ i ] != 0xFF )
    {
      erased = false;
    }
  }

  unlock( );

  return( erased );
}

/******************************************************************************/
bool uip_is_erased( void )
{
  const bool erased = uip_is_region_erase( 0, UIP_SIZE );

  return( erased );
}

/******************************************************************************/
bool uip_erase( void )
{
  lock( );

  MSC_Init( );

  const MSC_Status_TypeDef rc = MSC_ErasePage( ( uint32_t * )user_data_ptr );

  MSC_Deinit( );

  unlock( );

  return( rc == mscReturnOk );
}

/******************************************************************************/
bool uip_program( const uint32_t start, const uint8_t * data, const uint32_t data_len )
{
  bool ok = false;

  lock( );
  
  save_to_prog_buf( );

  if( ( start + data_len ) <= ( uint32_t )UIP_SIZE )
  {
    memcpy( &prog_buf[ start ], data, data_len );

    MSC_Init( );

    MSC_Status_TypeDef rc = MSC_ErasePage( ( uint32_t * )user_data_ptr );

    if( rc == mscReturnOk )
    {
      rc = MSC_WriteWord( ( uint32_t * )user_data_ptr, prog_buf, UIP_SIZE );

      ok = ( rc == mscReturnOk );
    }

    MSC_Deinit( );
  }

  unlock( );

  return( ok );
}

/******************************************************************************/
bool uip_read( const uint32_t start, const uint8_t * data, const uint32_t data_len )
{
  bool ok = false;

  lock( );

  save_to_prog_buf( );

  if( ( start + data_len ) <= ( uint32_t )UIP_SIZE )
  {
    memcpy( data, &prog_buf[ start ], data_len );

    ok = true;
  }

  unlock( );
  
  return( ok );
}

/******************************************************************************/
