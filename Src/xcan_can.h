#pragma once
#include <stdint.h>

#define CAN_FLAG_OVERRUN (0x10) /* frames were lost before this one */
#define CAN_FLAG_ECHO  (0x20)
#define CAN_FLAG_RTR   (0x40)
#define CAN_FLAG_EXTID (0x80)

#define CAN_ERROR_FLAG_BUSOFF    (1<<0)
#define CAN_ERROR_FLAG_RX_OVF    (1<<1)
#define CAN_ERROR_FLAG_TX_ERR    (1<<2)

typedef struct
{
  uint32_t  id;
  uint8_t   data[8];
  uint8_t   dlc;
  uint8_t   flags;
  uint8_t   dummy;
  uint64_t  timestamp;
}
can_message_t;

void xcan_can_init( void );
void xcan_can_poll( void );
void xcan_can_set_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw );
int xcan_can_send_message( const can_message_t *msg );
int xcan_can_tx_full( void );
int xcan_can_tx_make_room( void );
/* rx/tx callbacks return 0 when the host-bound response queue is full; the
   frame (or tx ack) is then retried on a later poll instead of being dropped */
void xcan_can_install_rx_callback( int (*cb)( can_message_t * ) );
void xcan_can_install_tx_callback( int (*cb)( can_message_t * ) );
void xcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) );
void xcan_can_set_bus_active( uint16_t mode );
void xcan_can_flush_queues( void );
void xcan_can_set_silent( uint8_t silent_mode );
void xcan_can_set_loopback( uint8_t loopback );
