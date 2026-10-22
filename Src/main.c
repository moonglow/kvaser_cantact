#include "stm32f0xx_hal.h"
#include "xcan_timestamp.h"
#include "xcan_led.h"
#include "xcan_protocol.h"
#include "xcan_usb.h"

void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* EXTERNAL CLOCK */
#if EXTERNAL_CLOCK
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

#if( HSE_VALUE == 16000000u )
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
#elif( HSE_VALUE == 8000000u )
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
#else
#error invalid HSE_VALUE
#endif
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );
}
#else
/* internal clock with USB SOF sync */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_CRSInitTypeDef pInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /** Configures CRS */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE( 48000000, 1000 );
  pInit.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  pInit.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

  HAL_RCCEx_CRSConfig( &pInit );
}
#endif

void SysTick_Handler( void )
{
  HAL_IncTick();
}

#if defined(STM32F072xB)
/* Reboot into the ST system bootloader (DFU) without the BOOT0 jumper: the
   request survives the reset in .noinit RAM and is acted on before any
   peripheral is set up. */
#define BOOTLOADER_MAGIC        (0xB007C0DEu)
#define SYSTEM_MEMORY_BASE      (0x1FFFC800u)

static volatile uint32_t bootloader_request[2] __attribute__((section(".noinit")));

static void jump_to_bootloader_if_requested( void )
{
  const uint32_t *vectors = (const uint32_t *)SYSTEM_MEMORY_BASE;

  if( bootloader_request[0] != BOOTLOADER_MAGIC || bootloader_request[1] != ~BOOTLOADER_MAGIC )
    return;
  bootloader_request[0] = 0;
  bootloader_request[1] = 0;

  /* the bootloader's interrupt vectors must appear at address 0 */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  __set_MSP( vectors[0] );
  ((void (*)( void ))vectors[1])();
}
#endif

void xcan_reset_to_bootloader( void )
{
#if defined(STM32F072xB)
  bootloader_request[0] = BOOTLOADER_MAGIC;
  bootloader_request[1] = ~BOOTLOADER_MAGIC;
#endif
  NVIC_SystemReset();
}

int main( void )
{
#if defined(STM32F072xB)
  jump_to_bootloader_if_requested();
#endif
  HAL_Init();

  SystemClock_Config();
  
  xcan_usb_init();
  xcan_led_init();
  xcan_timestamp_init();
  xcan_protocol_init();

  for(;;)
  {
    xcan_usb_poll();
    xcan_led_poll();
    xcan_protocol_poll();
  }
}
