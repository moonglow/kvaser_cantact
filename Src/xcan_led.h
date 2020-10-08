#pragma once
#include <stdint.h>

enum e_xcan_led
{
  LED0,
  LED1,

  LED_TOTAL,
};

enum e_xcan_led_mode
{
  LED_MODE_NONE,
  LED_MODE_ON,
  LED_MODE_OFF,
  LED_MODE_BLINK_FAST,
  LED_MODE_BLINK_SLOW,
};

void xcan_led_init( void );
void xcan_led_set_mode( int led, int mode, uint16_t arg );
void xcan_led_poll( void );
