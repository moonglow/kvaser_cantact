#pragma once
#include <stdint.h>

void xcan_protocol_init( void );
void xcan_protocol_poll( void );
/* drop all per-session state (stop the channel, flush queues, empty the response buffer); safe before xcan_protocol_init */
void xcan_protocol_reset( void );
/* main.c: reset; on the F072 the next boot enters the ST DFU bootloader */
void xcan_reset_to_bootloader( void );
