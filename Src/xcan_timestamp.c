#include <assert.h>
#include <stm32f0xx_hal.h>

#define TIM_BUS_FREQ (48000000)

void xcan_timestamp_init( void )
{

  switch( TIM_BUS_FREQ )
  {
    case 48000000:
      /* TIM2 32bit timer */
      __HAL_RCC_TIM2_CLK_ENABLE();
      __HAL_RCC_TIM2_FORCE_RESET();
      __HAL_RCC_TIM2_RELEASE_RESET();

      TIM2->PSC = (48-1); /* => tick = 1us */
      /* set clock division to zero: */
      TIM2->CR1 &= (uint16_t)(~TIM_CR1_CKD);
      TIM2->CR1 |= TIM_CLOCKDIVISION_DIV1;
      TIM2->EGR |= TIM_EGR_UG;
      /* enable timer */
      TIM2->CR1 |= TIM_CR1_CEN;
    break;
    default:
      assert( 0 );
    break;
  }
}

uint32_t xcan_timestamp_millis( void )
{
  return HAL_GetTick();
}

uint32_t xcan_timestamp_us( void )
{
  return TIM2->CNT;
}

void xcan_timestamp_ticks( uint16_t *ptime )
{
  uint64_t ticks = xcan_timestamp_us();
  /* !!!!! tick clock SWOPTION_24_MHZ_CLK */
  ticks *= 24u;

  ptime[0] = ( ticks & 0xFFFF );
  ptime[1] = ( (ticks>>16u) & 0xFFFF );
  ptime[2] = ( (ticks>>32u) & 0xFFFF );
}

void xcan_timestamp_ticks_from_ts( uint16_t *ptime, uint32_t ts )
{
  uint64_t ticks = ts;
  ticks *= 24u;

  ptime[0] = ( ticks & 0xFFFF );
  ptime[1] = ( (ticks>>16u) & 0xFFFF );
  ptime[2] = ( (ticks>>32u) & 0xFFFF );
}
