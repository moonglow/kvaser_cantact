#include <assert.h>
#include <stm32f0xx_hal.h>

#define TIM_BUS_FREQ (48000000)
static volatile uint32_t tim_overflow = 0;

void TIM2_IRQHandler( void )
{
  TIM2->SR = ~TIM_DIER_UIE;
  ++tim_overflow;
}

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
      /* EGR_UG will also provide unwanted UPDATE event, fix it */
      TIM2->EGR |= TIM_EGR_UG;
      __NOP();
      __NOP();
      __NOP();
      __NOP();
      TIM2->SR = 0u;
      __NOP();
      TIM2->CNT = 0u;
      /* enable timer overflow handler */
      TIM2->DIER |= TIM_DIER_UIE;
			HAL_NVIC_EnableIRQ( TIM2_IRQn );	
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

uint64_t xcan_timestamp_us( void )
{
  uint32_t tim_lo, tim_hi;
  do
  {
    tim_hi = tim_overflow;
    tim_lo = TIM2->CNT;
  }
  while( tim_hi != tim_overflow );

  return ((uint64_t)tim_hi<<32u)|tim_lo;
}

uint32_t xcan_timestamp32_us( void )
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

void xcan_timestamp_ticks_from_ts( uint16_t *ptime, uint64_t ts )
{
  uint64_t ticks = ts;
  ticks *= 24u;

  ptime[0] = ( ticks & 0xFFFF );
  ptime[1] = ( (ticks>>16u) & 0xFFFF );
  ptime[2] = ( (ticks>>32u) & 0xFFFF );
}
