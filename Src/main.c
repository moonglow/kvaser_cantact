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

int main( void )
{
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
