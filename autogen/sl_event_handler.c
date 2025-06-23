#include "sl_event_handler.h"

#include "em_chip.h"
#include "sl_device_init_nvic.h"
#include "sl_device_init_dcdc.h"
#include "sl_device_init_hfrco.h"
//#include "sl_hfxo_manager.h"
#include "sl_device_init_lfxo.h"
#include "sl_device_init_clocks.h"
#include "sl_device_init_emu.h"
#include "sl_sleeptimer.h"
#include "gpiointerrupt.h"
#include "sl_i2cspm_instances.h"
#include "cpu.h"
#include "cmsis_os2.h"
#include "sl_power_manager.h"

void sl_platform_init(void)
{
  CHIP_Init();
  sl_device_init_nvic();
  sl_device_init_dcdc();
  sl_device_init_hfrco();
  //sl_hfxo_manager_init_hardware();
  sl_device_init_lfxo();
  sl_device_init_clocks();
  sl_device_init_emu();
  CPU_Init();
  osKernelInitialize();
  sl_power_manager_init();
}

void sl_kernel_start(void)
{
  osKernelStart();
}

void sl_driver_init(void)
{
  GPIOINT_Init();
  sl_i2cspm_init_instances();
}

void sl_service_init(void)
{
  sl_sleeptimer_init();
  //sl_hfxo_manager_init();
}

void sl_stack_init(void)
{
}

void sl_internal_app_init(void)
{
}

