#include <stm32f0xx_hal.h>
#include <string.h>
#include <assert.h>
#include "xcan_can.h"
#include "xcan_timestamp.h"
#include "boards.h"

#define CAN_TX_FIFO_SIZE (64)
/* filled by the CAN interrupt, drained by the main loop; power of two.
   The F042's 6K of RAM only leaves room for a short ring. */
#if defined(STM32F072xB)
#define CAN_RX_RING_SIZE (128)
#else
#define CAN_RX_RING_SIZE (8)
#endif
static CAN_HandleTypeDef g_hcan = { .Instance = CAN };
#define INTERNAL_CAN_IT_FLAGS          (  CAN_IT_TX_MAILBOX_EMPTY |\
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |\
                                          CAN_IT_BUSOFF |\
                                          CAN_IT_ERROR_WARNING |\
                                          CAN_IT_ERROR_PASSIVE |\
                                          CAN_IT_BUSOFF |\
                                          CAN_IT_LAST_ERROR_CODE |\
                                          CAN_IT_ERROR )

static struct
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;
  uint64_t tx_progress_us; /* last time the tx fifo made progress, or became non-empty */

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  can_message_t tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;

  /* single producer (ISR) / single consumer (main loop) */
  can_message_t rx_ring[CAN_RX_RING_SIZE];
  volatile uint32_t rx_head;
  volatile uint32_t rx_tail;
  /* set by the ISR when frames are lost, consumed with the next delivered frame */
  volatile uint8_t rx_overrun;
  /* error flags latched by the ISR, forwarded to err_cb from the main loop */
  volatile uint8_t pending_err;

  int (*rx_cb)(can_message_t *);
  int (*tx_cb)(can_message_t *);
  void (*err_cb)( uint8_t err, uint8_t rx_err, uint8_t tx_err );
}
can_dev = { 0 };

void CEC_CAN_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &g_hcan );
}

/* reconfiguring the peripheral must not race the receive interrupt */
static void xcan_can_reinit( void )
{
  HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );
}

void xcan_can_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  CAN_FilterTypeDef filter = { 0 };

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

#ifdef IOPIN_CAN_S_PIN
  /* hold the transceiver out of standby */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin( IOPIN_CAN_S_PORT, IOPIN_CAN_S_PIN, GPIO_PIN_RESET );
  GPIO_InitStruct.Pin = IOPIN_CAN_S_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init( IOPIN_CAN_S_PORT, &GPIO_InitStruct );
#endif

  HAL_CAN_DeInit( &g_hcan );

  g_hcan.Instance = CAN;
  g_hcan.Init.Prescaler = 16;
  g_hcan.Init.Mode = CAN_MODE_NORMAL;
  g_hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  g_hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  g_hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  g_hcan.Init.TimeTriggeredMode = DISABLE;
  g_hcan.Init.AutoBusOff = ENABLE;
  g_hcan.Init.AutoWakeUp = DISABLE;
  g_hcan.Init.AutoRetransmission = ENABLE;
  g_hcan.Init.ReceiveFifoLocked = DISABLE;
  g_hcan.Init.TransmitFifoPriority = ENABLE;

  if( HAL_CAN_Init( &g_hcan ) != HAL_OK )
  {
    assert( 0 );
  }

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.SlaveStartFilterBank = 0;
  
  if( HAL_CAN_ConfigFilter( &g_hcan, &filter ) != HAL_OK )
  {
    assert( 0 );
  }

  if( HAL_CAN_ActivateNotification( &g_hcan, INTERNAL_CAN_IT_FLAGS ) != HAL_OK )
  {
    assert( 0 );
  }

  /* highest priority: the hardware FIFO is only 3 frames deep */
  HAL_NVIC_SetPriority( CEC_CAN_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );
}

void xcan_can_set_bitrate( uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  static const uint32_t sjw_table[] = 
  { 
    CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ 
  };
  static const uint32_t tseg1_table[] = 
  { 
    CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ, 
    CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ, 
    CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
    CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ
  };
  static const uint32_t tseg2_table[] =
  { 
    CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
    CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ
  };

  if( sjw > 4 )
    sjw = 4;
  if( tseg1 > 16 )
    tseg1 = 16;
  if( tseg2 > 8 )
    tseg2 = 8;

  /* CAN bus freq is 48 */
  g_hcan.Init.Prescaler = brp;

  g_hcan.Init.SyncJumpWidth = sjw_table[sjw - 1];
  g_hcan.Init.TimeSeg1 = tseg1_table[tseg1 - 1];
  g_hcan.Init.TimeSeg2 = tseg2_table[tseg2 - 1];

  xcan_can_reinit();
}

void xcan_can_install_rx_callback( int (*cb)( can_message_t * ) )
{
  can_dev.rx_cb = cb;
}

void xcan_can_install_tx_callback( int (*cb)( can_message_t * ) )
{
  can_dev.tx_cb = cb;
}

void xcan_can_install_error_callback( void (*cb)( uint8_t, uint8_t, uint8_t ) )
{
  can_dev.err_cb = cb;
}

static int xcan_try_send_message( const can_message_t *p_msg )
{
  CAN_TxHeaderTypeDef msg = { .TransmitGlobalTime = DISABLE };
  uint32_t txMailbox = 0;

  if( p_msg->flags & CAN_FLAG_EXTID )
  {
    msg.ExtId = p_msg->id & 0x1FFFFFFF;
    msg.IDE = CAN_ID_EXT;
  }
  else
  {
    msg.StdId = p_msg->id & 0x7FF;
    msg.IDE = CAN_ID_STD;
  }
  
  msg.DLC = p_msg->dlc;
  msg.RTR = (p_msg->flags & CAN_FLAG_RTR)?CAN_RTR_REMOTE:CAN_RTR_DATA;
  
  if( HAL_CAN_AddTxMessage( &g_hcan, &msg, (void*)p_msg->data, &txMailbox ) != HAL_OK )
    return -1;

  return txMailbox;
}

static void xcan_can_flush_tx( void )
{
  can_message_t *p_msg;

  while( can_dev.tx_head != can_dev.tx_tail )
  {
    /* the vendored HAL_CAN_AddTxMessage only uses mailbox 0 (TME1/TME2 checks
       are commented out to keep frames in order), so wait for that one */
    if( HAL_CAN_IsTxMessagePending( &g_hcan, CAN_TX_MAILBOX0 ) )
      return;

    p_msg = &can_dev.tx_fifo[can_dev.tx_tail];
    /* queue the host ack first: a frame sent without its ack leaks a driver tx context */
    if( can_dev.tx_cb && !can_dev.tx_cb( p_msg ) )
      return;
    /* cannot fail: mailbox 0 is free and only this loop fills it */
    (void)xcan_try_send_message( p_msg );
    can_dev.tx_progress_us = xcan_timestamp_us();
    can_dev.tx_tail = (can_dev.tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
  }
}

int xcan_can_tx_full( void )
{
  return ((can_dev.tx_head+1)&(CAN_TX_FIFO_SIZE-1)) == can_dev.tx_tail;
}

/* A full tx fifo normally makes the host command wait for space. On a bus nobody acknowledges (no peer, or the node
   is isolated) the head frame is retried forever and the fifo never drains; waiting then blocks every later host
   command (stop, flush, reset...) behind the stuck frame. After XCAN_TX_STALL_US without progress the oldest queued
   frame is dropped, as a controller that has gone bus-off would, so the command stream keeps moving.
   Returns 1 if there is room for one more frame. */
#define XCAN_TX_STALL_US (50000ULL)

int xcan_can_tx_make_room( void )
{
  if( !xcan_can_tx_full() )
    return 1;
  if( xcan_timestamp_us() - can_dev.tx_progress_us < XCAN_TX_STALL_US )
    return 0;

  HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
  HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  can_dev.tx_tail = (can_dev.tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
  ++can_dev.tx_ovfs;
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );
  can_dev.tx_progress_us = xcan_timestamp_us();
  return 1;
}

int xcan_can_send_message( const can_message_t *p_msg )
{
  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (can_dev.tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == can_dev.tx_tail )
  {
    ++can_dev.tx_ovfs;
    return -1;
  }

  if( can_dev.tx_head == can_dev.tx_tail )
    can_dev.tx_progress_us = xcan_timestamp_us();
  can_dev.tx_fifo[can_dev.tx_head] = *p_msg;
  can_dev.tx_head = tx_head_next;

  return 0;
}

void xcan_can_set_silent( uint8_t silent_mode )
{
  g_hcan.Init.Mode = silent_mode ? CAN_MODE_SILENT: CAN_MODE_NORMAL;
  xcan_can_reinit();
}

void xcan_can_set_loopback( uint8_t loopback )
{
  g_hcan.Init.Mode = loopback ? CAN_MODE_LOOPBACK: CAN_MODE_NORMAL;
  xcan_can_reinit();
}

/* Drop everything queued in either direction, as the real hardware does when the
   driver flushes or stops the channel; otherwise frames queued while the bus
   was unacknowledged go out the next time the interface comes up. */
void xcan_can_flush_queues( void )
{
  HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
  HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  can_dev.tx_tail = can_dev.tx_head;
  can_dev.rx_tail = can_dev.rx_head;
  can_dev.rx_overrun = 0;
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );
}

void xcan_can_set_bus_active( uint16_t mode )
{
  HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
  if( mode )
  {
    HAL_CAN_Start( &g_hcan );
    HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  }
  else
  {
    HAL_CAN_AbortTxRequest( &g_hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
    HAL_CAN_Stop( &g_hcan );
  }
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );

  if( !mode )
  {
    xcan_can_flush_queues();
  }
}

/* ISR context: move every frame waiting in the hardware FIFO into the ring */
static void xcan_can_rx_frames( CAN_HandleTypeDef *hcan, uint32_t fifo )
{
  CAN_RxHeaderTypeDef hdr;

  while( HAL_CAN_GetRxFifoFillLevel( hcan, fifo ) )
  {
    uint32_t head = can_dev.rx_head;
    uint32_t head_next = (head+1)&(CAN_RX_RING_SIZE-1);
    can_message_t *msg = &can_dev.rx_ring[head];

    if( head_next == can_dev.rx_tail )
    {
      /* ring full: release the hardware slot and flag the loss */
      uint8_t discard[8];
      (void)HAL_CAN_GetRxMessage( hcan, fifo, &hdr, discard );
      ++can_dev.rx_ovfs;
      can_dev.rx_overrun = 1;
      continue;
    }

    if( HAL_CAN_GetRxMessage( hcan, fifo, &hdr, msg->data ) != HAL_OK )
      return;

    msg->flags = 0;
    if( hdr.IDE == CAN_ID_STD )
    {
      msg->id = hdr.StdId;
    }
    else
    {
      msg->id = hdr.ExtId;
      msg->flags |= CAN_FLAG_EXTID;
    }
    if( hdr.RTR == CAN_RTR_REMOTE )
    {
      msg->flags |= CAN_FLAG_RTR;
    }
    msg->dlc = hdr.DLC;
    msg->timestamp = xcan_timestamp_us();

    can_dev.rx_head = head_next;
    ++can_dev.rx_msgs;
  }
}

/* main loop: hand ring frames to the protocol while it has room for them */
static void xcan_can_flush_rx( void )
{
  while( can_dev.rx_tail != can_dev.rx_head )
  {
    can_message_t *msg = &can_dev.rx_ring[can_dev.rx_tail];

    if( can_dev.rx_overrun )
    {
      /* masked so a loss flagged by the ISR between test and clear is not lost */
      HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
      can_dev.rx_overrun = 0;
      HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );
      msg->flags |= CAN_FLAG_OVERRUN;
    }
    if( can_dev.rx_cb && !can_dev.rx_cb( msg ) )
    {
      /* keep the frame; the flag is already on it for the retry */
      return;
    }
    can_dev.rx_tail = (can_dev.rx_tail+1)&(CAN_RX_RING_SIZE-1);
  }
}

static void xcan_can_flush_errors( void )
{
  uint8_t err;

  if( !can_dev.pending_err )
    return;

  HAL_NVIC_DisableIRQ( CEC_CAN_IRQn );
  err = can_dev.pending_err;
  can_dev.pending_err = 0;
  HAL_NVIC_EnableIRQ( CEC_CAN_IRQn );

  if( can_dev.err_cb )
  {
    can_dev.err_cb( err, can_dev.tx_errs & 0xFF, can_dev.rx_errs );
  }
}

void xcan_can_poll(void)
{
  xcan_can_flush_errors();
  xcan_can_flush_rx();
  xcan_can_flush_tx();
}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  xcan_can_rx_frames( hcan, CAN_RX_FIFO0 );
}

void HAL_CAN_RxFifo1MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  xcan_can_rx_frames( hcan, CAN_RX_FIFO1 );
}

void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.tx_msgs;
}

void HAL_CAN_RxFifo0FullCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.rx_ovfs;
}

void HAL_CAN_RxFifo1FullCallback( CAN_HandleTypeDef *hcan )
{
  UNUSED( hcan );
  ++can_dev.rx_ovfs;
}

void HAL_CAN_SleepCallback( CAN_HandleTypeDef *hcan ){ UNUSED( hcan ); }
void HAL_CAN_WakeUpFromRxMsgCallback( CAN_HandleTypeDef *hcan ){ UNUSED( hcan ); }

void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan )
{
  /* handle errors */
  uint32_t err = HAL_CAN_GetError( hcan );
  uint8_t  can_err = 0;

  if ( err & ( HAL_CAN_ERROR_TX_TERR0 | HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2 ) ) 
  {
    ++can_dev.tx_errs;
    can_err |= CAN_ERROR_FLAG_TX_ERR;
  }

  if( err & HAL_CAN_ERROR_BOF )
  {
    can_err |= CAN_ERROR_FLAG_BUSOFF;
  }

  if( err & ( HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1 ) )
  {
    can_err |= CAN_ERROR_FLAG_RX_OVF;
    /* hardware FIFO overrun: frames were lost before the next one we deliver */
    ++can_dev.rx_ovfs;
    can_dev.rx_overrun = 1;
  }

  /* ISR context: the main loop forwards these to err_cb */
  can_dev.pending_err |= can_err;

  HAL_CAN_ResetError( hcan );
}
