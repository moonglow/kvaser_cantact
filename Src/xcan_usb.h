#pragma once
#include <stdint.h>

#define XCAN_USB_EP_MSGOUT  0x02
#define XCAN_USB_EP_MSGIN   0x82

struct t_class_data
{
  int tx_flow_control_in_use;
  uint8_t ep_tx_in_use[15];
  uint8_t data1_ep_buffer[64];
};

struct t_m2h_fsm
{
  uint8_t   state;
  uint8_t   ep_addr;
  uint8_t   *pdbuf;
  int       dbsize;
  uint32_t  total_tx;
};

int xcan_flush_ep( uint8_t ep );
int xcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size );
void xcan_usb_init( void );
void xcan_usb_poll( void );
