#pragma once
#include <stdint.h>

void xcan_protocol_init( void );
void xcan_protocol_poll( void );
void xcan_protocol_process_data( uint8_t *ptr, uint16_t size );
