#pragma once
#include <stdint.h>

void xcan_timestamp_init( void );
uint32_t xcan_timestamp_millis( void );
uint64_t xcan_timestamp_us( void );
uint32_t xcan_timestamp32_us( void );
void xcan_timestamp_ticks( uint16_t *ptime );
void xcan_timestamp_ticks_from_ts( uint16_t *ptime, uint64_t ts );

