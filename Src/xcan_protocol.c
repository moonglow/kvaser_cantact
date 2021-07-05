#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "xcan_can.h"
#include "xcan_usb.h"
#include "xcan_led.h"
#include "xcan_timestamp.h"
#include "filo_cmds.h"

#define XCAN_CHANNEL_MAX (1)

static struct
{
  struct
  {
    /* error handling related */
    uint8_t   err;
    uint8_t   ecc;
    uint8_t   rx_err;
    uint8_t   tx_err;

    /* config, bitrate */
    uint32_t bitrate;
    uint8_t  tseg1;
    uint8_t  tseg2;
    uint8_t  sjw;
    uint8_t  no_samp;

    uint8_t   silient;
    uint8_t   loopback;

    uint8_t   driver_mode;
    uint8_t   bus_active;

    char sz_name[24];
  }
  can[XCAN_CHANNEL_MAX];
}
xcan_device =
{
  .can[0] = 
  {
    .loopback = 0,
    .bus_active = 0,
    .driver_mode = DRIVERMODE_OFF,

    /* dummy settings */
    .bitrate = 500000,
    .tseg1 = 4,
    .tseg2 = 3,
    .sjw = 1,
    .no_samp = 1,

    /* channel name */
    .sz_name = "Moonglow"
  },
};

static uint8_t temp_resp_buffer[732] = { 0 };
static uint8_t resp_buffer[sizeof(temp_resp_buffer)];
static int resp_buffer_pos = 0;

static struct t_m2h_fsm resp_fsm = 
{
  .state = 0,
  .ep_addr = XCAN_USB_EP_MSGIN,
  .pdbuf = temp_resp_buffer,
  .dbsize = sizeof( temp_resp_buffer ),
};

void *xcan_alloc_resp( int size )
{
  void *p;

  if( (resp_buffer_pos+size) > ((int)sizeof( resp_buffer ) -1 ) )
    return (void*)0;

  p = &resp_buffer[resp_buffer_pos];
  memset( p, 0x00, size );
  resp_buffer_pos += size;
  return p;
}

static void xcan_rx_message( can_message_t *pmsg )
{
  cmdRxCanMessage *p_resp = xcan_alloc_resp( sizeof( cmdRxCanMessage ) );
  if( !p_resp )
    return;
  
  /* valid CAN message, clear error flags ? */
  xcan_device.can[0].err = 0;

  p_resp->cmdLen = sizeof( cmdRxCanMessage );
  p_resp->cmdNo = ( pmsg->flags & CAN_FLAG_EXTID )? CMD_RX_EXT_MESSAGE:CMD_RX_STD_MESSAGE;
  p_resp->channel = 0;
  p_resp->flags = 0;
  
  if( pmsg->flags & CAN_FLAG_RTR )
  {
    p_resp->flags |= MSGFLAG_REMOTE_FRAME;
  }

  xcan_timestamp_ticks_from_ts( p_resp->time, pmsg->timestamp );

  if( pmsg->flags & CAN_FLAG_EXTID )
  {
    p_resp->rawMessage[0] = ((pmsg->id >> 24) & 0x1F);
    p_resp->rawMessage[1] = ((pmsg->id >> 18) & 0x3F);
    p_resp->rawMessage[2] = ((pmsg->id >> 14) & 0x0F);
    p_resp->rawMessage[3] = ((pmsg->id >>  6) & 0xFF);
    p_resp->rawMessage[4] = ((pmsg->id      ) & 0x3F);
  }
  else
  {
    p_resp->rawMessage[0] = ((pmsg->id >>  6) & 0x1F);
    p_resp->rawMessage[1] = ((pmsg->id      ) & 0x3F);
  }
  
  p_resp->rawMessage[5] = pmsg->dlc&0x0F;
  memcpy( &p_resp->rawMessage[6], pmsg->data, 8 );

  xcan_led_set_mode( LED1, LED_MODE_BLINK_FAST, 237 );
}

/* not real tx ok cb */
static void xcan_tx_message( can_message_t *msg )
{
  cmdTxAck *p_resp = xcan_alloc_resp( sizeof( cmdTxAck ) );
  if( !p_resp )
    return;

  p_resp->cmdLen = sizeof( cmdTxAck );
  p_resp->cmdNo = CMD_TX_ACKNOWLEDGE;
  p_resp->channel = 0;
  p_resp->transId = msg->dummy;

  xcan_timestamp_ticks_from_ts( p_resp->time, msg->timestamp );
  xcan_led_set_mode( LED0, LED_MODE_BLINK_FAST, 237 );
}

static void xcan_can_error( uint8_t err, uint8_t rx_err, uint8_t tx_err )
{
  (void)rx_err;
  (void)tx_err;
  xcan_device.can[0].err = err;
}

#define WAIT_FOR_TXSLOTS 1
int xcan_handle_tx_message( cmdTxCanMessage *pmsg, uint8_t ext_id )
{
  can_message_t msg = { 0 };

  msg.id = pmsg->rawMessage[0] & 0x1F;
  msg.id <<= 6;
  msg.id |= pmsg->rawMessage[1] & 0x3F;

  if( ext_id )
  {
    msg.id <<= 4;
    msg.id |= pmsg->rawMessage[2] & 0x0F;
    msg.id <<= 8;
    msg.id |= pmsg->rawMessage[3] & 0xFF;
    msg.id <<= 6;
    msg.id |= pmsg->rawMessage[4] & 0x3F;
    msg.flags |= CAN_FLAG_EXTID;
  }

  if( pmsg->flags & MSGFLAG_REMOTE_FRAME )
  {
    msg.flags |= CAN_FLAG_RTR;
  }

  msg.dlc = pmsg->rawMessage[5]&0x0F;
  memcpy( msg.data, &pmsg->rawMessage[6], 8 );

  /* use transID for txAck */
  msg.dummy = pmsg->transId;
  msg.timestamp = xcan_timestamp_us();

#if WAIT_FOR_TXSLOTS
  const uint32_t ts_poll = xcan_timestamp32_us();
  while( xcan_can_send_message( &msg ) < 0 )
  {
    xcan_can_poll();
    if( xcan_device.can[0].err & CAN_ERROR_FLAG_BUSOFF )
      return -1;
    uint32_t ts_diff = xcan_timestamp32_us() - ts_poll;
    if( ts_diff >= 1000000u )
      return -1;
  }
#else
  if( xcan_can_send_message( &msg ) < 0 )
  {
    /* tx queue overflow ? */
    return -1;
  }
#endif
  return 0;
}

void xcan_handle_command( filoCmd *pcmd, int size )
{
  if( size < 2 )
    return;

  switch( pcmd->head.cmdNo )
  {
    case CMD_GET_CARD_INFO_REQ:
    {
      {
        cmdUsbThrottle *p_resp = xcan_alloc_resp( sizeof( cmdUsbThrottle ) );
        if( !p_resp )
          break;
        p_resp->cmdLen = sizeof( cmdUsbThrottle );
        p_resp->cmdNo = CMD_USB_THROTTLE;
        p_resp->throttle = 1000;
      }
      {
        cmdGetCardInfo2Resp *p_resp = xcan_alloc_resp( sizeof( cmdGetCardInfo2Resp ) );
        if( !p_resp )
          break;
        p_resp->cmdLen = sizeof( cmdGetCardInfo2Resp );
        p_resp->cmdNo = CMD_GET_CARD_INFO_2;
        p_resp->transId = pcmd->getCardInfoReq.transId;
        strcpy( (void*)p_resp->pcb_id, "685v3.0" );
        p_resp->oem_unlock_code = 0;
      }
      cmdGetCardInfoResp *p_resp = xcan_alloc_resp( sizeof( cmdGetCardInfoResp ) );
      if( !p_resp )
        break;
      
       p_resp->cmdLen = sizeof( cmdGetCardInfoResp );
       p_resp->cmdNo = CMD_GET_CARD_INFO_RESP;

       p_resp->transId = pcmd->getCardInfoReq.transId;
       p_resp->channelCount = 1;
       p_resp->serialNumber = 666u;
       p_resp->padding1 = 0;
       p_resp->clockResolution = 0;
       p_resp->mfgDate = 1523452686u; /* unix time */
       p_resp->EAN[0] = 0x50;
       p_resp->EAN[1] = 0x68;
       p_resp->EAN[2] = 0x00;
       p_resp->EAN[3] = 0x30;
       p_resp->EAN[4] = 0x01;
       p_resp->EAN[5] = 0x33;
       p_resp->EAN[6] = 0x07;
       p_resp->EAN[7] = 0x00;
       p_resp->hwRevision = 1;
       p_resp->usbHsMode = 0;
       p_resp->hwType = 48;
       p_resp->canTimeStampRef = CAN_TIME_STAMP_REF_INTERRUPT;
    }
    break;
    case CMD_GET_SOFTWARE_INFO_REQ:
    {
      cmdGetSoftwareInfoResp *p_resp = xcan_alloc_resp( sizeof( cmdGetSoftwareInfoResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdGetSoftwareInfoResp );
      p_resp->cmdNo = CMD_GET_SOFTWARE_INFO_RESP;
      p_resp->transId = pcmd->getSoftwareInfoReq.transId;
      p_resp->swOptions = SWOPTION_CAP_REQ | SWOPTION_24_MHZ_CLK;
      p_resp->firmwareVersion = (4<<24)|(15<<16)|(409);
      p_resp->maxOutstandingTx = 64;
    }
    break;
    case CMD_GET_CAPABILITIES_REQ:
    {
      cmdCapabilitiesResp *p_resp = xcan_alloc_resp( sizeof( cmdCapabilitiesResp ) );
      if( !p_resp )
        break;
      p_resp->cmdLen = sizeof( cmdCapabilitiesResp );
      p_resp->cmdNo = CMD_GET_CAPABILITIES_RESP;
      p_resp->pad = 0x4242;
      p_resp->subCmdNo = pcmd->capabilitiesReq.subCmdNo;
      p_resp->status = CAP_STATUS_NOT_IMPLEMENTED;

      switch( pcmd->capabilitiesReq.subCmdNo )
      {
        case CAP_SUB_CMD_SILENT_MODE:
          p_resp->status = CAP_STATUS_OK;
          p_resp->silentMode.mask = 0x00000001;
          p_resp->silentMode.value = 0x00000000;
        break;
        case CAP_SUB_CMD_ERRFRAME:
          p_resp->status = CAP_STATUS_OK;
          p_resp->errframeCap.mask = 0x00000001;
          p_resp->errframeCap.value = 0x00000000;
        break;
        case CAP_SUB_CMD_BUS_STATS:
          p_resp->status = CAP_STATUS_OK;
          p_resp->busstatCap.mask = 0x00000001;
          p_resp->busstatCap.value = 0x00000000;
        break;
        case CAP_SUB_CMD_ERRCOUNT_READ:
          p_resp->status = CAP_STATUS_OK;
          p_resp->errcountCap.mask = 0x00000001;
          p_resp->errcountCap.value = 0x00000000;
        break;
        case CAP_SUB_CMD_SYNC_TX_FLUSH:
          p_resp->status = CAP_STATUS_OK;
          p_resp->syncTxFlushCap.mask = 0x00000001;
          p_resp->syncTxFlushCap.value = 0x00000001;
        break;
        case CAP_SUB_CMD_HAS_LOGGER:
          p_resp->status = CAP_STATUS_OK;
          p_resp->loggerCap.mask = 0x00000001;
          p_resp->loggerCap.value = 0x00000000;
        break;
        case CAP_SUB_CMD_HAS_REMOTE:
          p_resp->status = CAP_STATUS_OK;
          p_resp->remoteCap.mask = 0x00000001;
          p_resp->remoteCap.value = 0x00000000;
        break;
        case CAP_SUB_CMD_HAS_SCRIPT:
          p_resp->status = CAP_STATUS_OK;
          p_resp->scriptCap.mask = 0x00000001;
          p_resp->scriptCap.value = 0x00000000;
        break;
        default:
        break;
      }
    }
    break;
    case CMD_CHECK_LICENSE_REQ:
    {
      cmdCheckLicenseResp *p_resp = xcan_alloc_resp( sizeof( cmdCheckLicenseResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdCheckLicenseResp );
      p_resp->cmdNo = CMD_CHECK_LICENSE_RESP;
      p_resp->transId = pcmd->checkLicenseReq.transId;
      p_resp->padding = 0x42;
      p_resp->licenseMask = 0x00000000;
      p_resp->kvaserLicenseMask = 0x00000000;
    }
    break;
    case CMD_GET_DEVICE_NAME_REQ:
    {
      cmdGetDeviceNameResp *p_resp = xcan_alloc_resp( sizeof( cmdGetDeviceNameResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdGetDeviceNameResp );
      p_resp->cmdNo = CMD_GET_DEVICE_NAME_RESP;
      p_resp->index = 0;
    }
    break;
    case CMD_SOUND:
    /* do nothing */
    break;
    case CMD_SOFTSYNC_ONOFF:
    {
      (void)pcmd->softSyncOnOff.onOff;
      /* return previus state ? */
      cmdSoftSyncOnOff *p_resp = xcan_alloc_resp( sizeof( cmdSoftSyncOnOff ) );
      if( !p_resp )
        break;
      p_resp->cmdLen = sizeof( cmdSoftSyncOnOff );
      p_resp->cmdNo = CMD_SOFTSYNC_ONOFF;
      p_resp->onOff = SOFTSYNC_NOT_STARTED;
    }
    break;
    case CMD_READ_USER_PARAMETER:
    {
      cmdReadUserParameter *p_resp = xcan_alloc_resp( sizeof( cmdReadUserParameter ) );
      if( !p_resp )
        break;
      /* DUMMY response */
      p_resp->cmdLen = sizeof( cmdReadUserParameter );
      p_resp->cmdNo = CMD_READ_USER_PARAMETER;
      p_resp->status = MEMO_STATUS_EOF;

      switch( pcmd->readUserParameter.userNo )
      {
        case 199: /* channel name ID */
        {
          uint8_t param_index = pcmd->readUserParameter.paramNo;
          if( param_index < 5 )
          {
            param_index -= 2;
            memcpy( p_resp->data, &xcan_device.can[0].sz_name[param_index*8], 8 );
            p_resp->status = MEMO_STATUS_SUCCESS;
          }
        }
        break;
      }
      
    }
    break;
    case CMD_STOP_CHIP_REQ:
    {
      cmdStopChipResp *p_resp = xcan_alloc_resp( sizeof( cmdStopChipResp ) );
      if( !p_resp )
        break;
      p_resp->cmdLen = sizeof( cmdStopChipResp );
      p_resp->cmdNo = CMD_STOP_CHIP_RESP;
      p_resp->transId = pcmd->stopChipReq.transId;
      p_resp->channel = pcmd->stopChipReq.channel;

      xcan_device.can[pcmd->stopChipReq.channel].bus_active = 0;
      xcan_can_set_bus_active( 0 );
    }
    break;
    /* TODO: */
    case CMD_RESET_CHIP_REQ:
    break;
    /* TODO: */
    case CMD_RESET_CARD_REQ:
    break;
    case CMD_START_CHIP_REQ:
    {
      cmdStartChipResp *p_resp = xcan_alloc_resp( sizeof( cmdStartChipResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdStartChipResp );
      p_resp->cmdNo = CMD_START_CHIP_RESP;
      p_resp->transId = pcmd->startChipReq.transId;
      p_resp->channel = pcmd->startChipReq.channel;

      xcan_device.can[pcmd->stopChipReq.channel].bus_active = 1;
      xcan_can_set_bus_active( 1 );
    }
    break;
    case CMD_GET_CHIP_STATE_REQ:
    {
      cmdChipStateEvent *p_resp = xcan_alloc_resp( sizeof( cmdChipStateEvent ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdChipStateEvent );
      p_resp->cmdNo = CMD_CHIP_STATE_EVENT;
      p_resp->transId = pcmd->getChipStateReq.transId;
      p_resp->channel = pcmd->getChipStateReq.channel;
      xcan_timestamp_ticks( p_resp->time );
      p_resp->busStatus = 0;
    }
    break;
    case CMD_GET_BUSLOAD_REQ:
    {
      cmdGetBusLoadResp *p_resp = xcan_alloc_resp( sizeof( cmdGetBusLoadResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdGetBusLoadResp );
      p_resp->cmdNo = CMD_GET_BUSLOAD_RESP;
      p_resp->transId = pcmd->getBusLoadReq.transId;
      p_resp->channel = pcmd->getBusLoadReq.channel;
      xcan_timestamp_ticks( p_resp->time );
    }
    break;
    /* no answer needed */
    case CMD_SET_BUSPARAMS_REQ:
    {
      uint16_t brp;

      assert( pcmd->setBusparamsReq.channel < XCAN_CHANNEL_MAX );

      brp = 16000000u/(pcmd->setBusparamsReq.bitRate*(1 + pcmd->setBusparamsReq.tseg1 + pcmd->setBusparamsReq.tseg2));
      if( brp <= 1 || brp > 256 )
        break;
      /* adjust for 48MHz */
      brp *= 3;
      
      xcan_can_set_bitrate( brp, pcmd->setBusparamsReq.tseg1, pcmd->setBusparamsReq.tseg2, pcmd->setBusparamsReq.sjw );

      /* save for future use */
      uint8_t ch = pcmd->setBusparamsReq.channel;
      xcan_device.can[ch].bitrate = pcmd->setBusparamsReq.bitRate;
      xcan_device.can[ch].tseg1 = pcmd->setBusparamsReq.tseg1;
      xcan_device.can[ch].tseg2 = pcmd->setBusparamsReq.tseg2;
      xcan_device.can[ch].sjw = pcmd->setBusparamsReq.sjw;
      xcan_device.can[ch].no_samp = pcmd->setBusparamsReq.noSamp;
    }
    break;
    case CMD_GET_BUSPARAMS_REQ:
    {
      cmdGetBusparamsResp *p_resp = xcan_alloc_resp( sizeof( cmdGetBusparamsResp ) );
      if( !p_resp )
        break;

      assert( pcmd->setBusparamsReq.channel < XCAN_CHANNEL_MAX );
      
      p_resp->cmdLen = sizeof( cmdGetBusparamsResp );
      p_resp->cmdNo = CMD_GET_BUSPARAMS_RESP;
      p_resp->transId = pcmd->getBusparamsReq.transId;
      p_resp->channel = pcmd->getBusparamsReq.channel;

      uint8_t ch = pcmd->setBusparamsReq.channel;

      p_resp->bitRate = xcan_device.can[ch].bitrate;
      p_resp->tseg1 = xcan_device.can[ch].tseg1;
      p_resp->tseg2 = xcan_device.can[ch].tseg2;
      p_resp->sjw = xcan_device.can[ch].sjw ;
      p_resp->noSamp = xcan_device.can[ch].no_samp;
    }
    break;
    case CMD_GET_DRIVERMODE_REQ:
    {
      cmdGetDrivermodeResp *p_resp = xcan_alloc_resp( sizeof( cmdGetDrivermodeResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdGetDrivermodeResp );
      p_resp->cmdNo = CMD_GET_DRIVERMODE_RESP;
      p_resp->transId = pcmd->getDrivermodeReq.transId;
      p_resp->channel = pcmd->getDrivermodeReq.channel;
   
      p_resp->driverMode = xcan_device.can[pcmd->getDrivermodeReq.channel].driver_mode;
    }
    break;
    case CMD_SET_DRIVERMODE_REQ:
      if( xcan_device.can[pcmd->setDrivermodeReq.channel].bus_active )
      {
        /* we need to stop CAN before change settings */
        xcan_can_set_bus_active( 0 );
      }
      switch( pcmd->setDrivermodeReq.driverMode )
      {
        case DRIVERMODE_NORMAL:
          xcan_can_set_silent( 0 );
        break;
        case DRIVERMODE_SILENT:
          xcan_can_set_silent( 1 );
        break;
        case DRIVERMODE_SELFRECEPTION:
          xcan_can_set_loopback( 1 );
        break;
        case DRIVERMODE_OFF:
          /* TODO: */
        break;
      }
      if( xcan_device.can[pcmd->setDrivermodeReq.channel].bus_active )
      {
        /* and we can start it again :) */
        xcan_can_set_bus_active( 1 );
      }
      xcan_device.can[pcmd->setDrivermodeReq.channel].driver_mode = pcmd->setDrivermodeReq.driverMode;
    break;
    case CMD_GET_TRANSCEIVER_INFO_REQ:
    {
      cmdGetTransceiverInfoResp *p_resp = xcan_alloc_resp( sizeof( cmdGetTransceiverInfoResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdGetTransceiverInfoResp );
      p_resp->cmdNo = CMD_GET_TRANSCEIVER_INFO_RESP;
      p_resp->channel = pcmd->getTransceiverInfoReq.channel;

      p_resp->transceiverCapabilities = 0;
      p_resp->transceiverStatus = 0;
      p_resp->transceiverType = FILO_TRANSCEIVER_TYPE_251;
    }
    break;
    case CMD_READ_CLOCK_REQ:
    {
      cmdReadClockResp *p_resp = xcan_alloc_resp( sizeof( cmdReadClockResp ) );
      if( !p_resp )
        break;

      p_resp->cmdLen = sizeof( cmdReadClockResp );
      p_resp->cmdNo = CMD_READ_CLOCK_RESP;
      p_resp->transId = pcmd->readClockReq.transId;
      xcan_timestamp_ticks( p_resp->time );
    }
    break;
    case CMD_LED_ACTION_REQ:
    {
      cmdLedActionResp *p_resp = xcan_alloc_resp( sizeof( cmdLedActionResp ) );
      if( !p_resp )
        break;
      /* TODO: dummy response */
      p_resp->cmdLen = sizeof( cmdLedActionResp );
      p_resp->cmdNo = CMD_LED_ACTION_RESP;
      p_resp->transId = pcmd->ledActionReq.transId;
      p_resp->subCmd = pcmd->ledActionReq.subCmd;

      switch( p_resp->subCmd )
      {
        case LED_SUBCOMMAND_ALL_LEDS_OFF:
        case LED_SUBCOMMAND_ALL_LEDS_ON:
          xcan_led_set_mode( LED0, ( p_resp->subCmd == LED_SUBCOMMAND_ALL_LEDS_ON ) ? LED_MODE_ON:LED_MODE_OFF, 
                                  pcmd->ledActionReq.timeout );
          xcan_led_set_mode( LED1, ( p_resp->subCmd == LED_SUBCOMMAND_ALL_LEDS_ON ) ? LED_MODE_ON:LED_MODE_OFF, 
                                  pcmd->ledActionReq.timeout );
        break;
        case LED_SUBCOMMAND_LED_0_ON:
        case LED_SUBCOMMAND_LED_0_OFF:
          xcan_led_set_mode( LED0, ( p_resp->subCmd == LED_SUBCOMMAND_LED_0_ON )?LED_MODE_ON:LED_MODE_OFF
                    , pcmd->ledActionReq.timeout );
        break;
        case LED_SUBCOMMAND_LED_1_ON:
        case LED_SUBCOMMAND_LED_1_OFF:
          xcan_led_set_mode( LED1, ( p_resp->subCmd == LED_SUBCOMMAND_LED_1_ON )?LED_MODE_ON:LED_MODE_OFF
                    , pcmd->ledActionReq.timeout );
        break;
      }
    }
    break;
    /* RX TX CAN QUEUE */
    /* flush tx queue */
    /* TODO: check capabilities */
    case CMD_FLUSH_QUEUE:
    {
      cmdFlushQueueResp *p_resp = xcan_alloc_resp( sizeof( cmdFlushQueueResp ) );
      if( !p_resp )
        break;
      p_resp->cmdLen = sizeof( cmdFlushQueueResp );
      p_resp->cmdNo = CMD_FILO_FLUSH_QUEUE_RESP;
      p_resp->transId = pcmd->flushQueue.transId;
      p_resp->channel = pcmd->flushQueue.channel;
      p_resp->flags = pcmd->flushQueue.flags;
    }
    break;
    case CMD_TX_STD_MESSAGE:
    case CMD_TX_EXT_MESSAGE:
      (void)xcan_handle_tx_message( &pcmd->txCanMessage, pcmd->head.cmdNo == CMD_TX_EXT_MESSAGE );
    break;
    default:
      assert( 0 );
    break;
  }
}

void xcan_protocol_process_data( uint8_t *ptr, uint16_t size )
{
  filoCmd *pcmd = 0;

  for( int i = 0; i < size; )
  {
    if( !ptr[i] )
      break;
    pcmd = (void*)&ptr[i];
#if 0
    /* next cmd aligned to 64 byte bound, we can just wait for next data chunk */
    if( pcmd->head.cmdLen == 0 )
    {
      //i = (i+63)&(~63);
      //continue;
      break;
    }
#endif
    xcan_handle_command( pcmd, size - i );
    i += pcmd->head.cmdLen;
  }
}

void xcan_protocol_init( void )
{
  xcan_can_init();
  xcan_can_install_rx_callback( xcan_rx_message  );
  xcan_can_install_tx_callback( xcan_tx_message  );
  xcan_can_install_error_callback( xcan_can_error );
}

void xcan_protocol_poll( void )
{
  if( resp_buffer_pos > 1 )
  {
    /* add zero cmd len as finish flag */
    resp_buffer[resp_buffer_pos] = 0;
    int res = xcan_flush_data( &resp_fsm, resp_buffer, resp_buffer_pos + 1 );
    //int res = xcan_flush_data( &resp_fsm, resp_buffer, resp_buffer_pos );
    if( res )
    {
      resp_buffer_pos = 0;
    }
  }

  xcan_can_poll();
}
