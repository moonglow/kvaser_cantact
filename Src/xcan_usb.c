#include <stm32f0xx_hal.h>
#include <assert.h>
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_conf.h"
#include "usbd_helper.h"
#include "xcan_protocol.h"
#include "xcan_led.h"
#include "xcan_usb.h"

static struct t_class_data xcan_data = { 0 };
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;

struct t_xcan_description
{
  USB_CONFIGURATION_DESCRIPTOR con0;
  USB_INTERFACE_DESCRIPTOR     if0;
  USB_ENDPOINT_DESCRIPTOR      ep1;
  USB_ENDPOINT_DESCRIPTOR      ep2;
};

__ALIGN_BEGIN static const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua __ALIGN_END = 
{
  .bLength            = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR ),
  .bDescriptorType    = USB_QUALIFIER_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0100, /* 1.0 */
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = 64,
  .bNumConfigurations = 1,
  .bReserved          = 0,
};

__ALIGN_BEGIN static struct t_xcan_description xcan_usb_dev __ALIGN_END = 
{
  .con0 =
  {
    .bLength              = sizeof( USB_CONFIGURATION_DESCRIPTOR ),
    .bDescriptorType      = USB_CONFIGURATION_DESCRIPTOR_TYPE,
    .wTotalLength         = sizeof( struct t_xcan_description ), 
    .bNumInterfaces       = 1,
    .bConfigurationValue  = 1,
    .iConfiguration       = 0,
    .bmAttributes         = USB_CONFIG_BUS_POWERED | USB_CONFIG_SELF_POWERED,
    .MaxPower             = 45, /* = 90mA */
  },
  .if0 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 0,
    .bAlternateSetting    = 0,
    .bNumEndpoints        = 2,
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 0,
  },
  .ep1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = XCAN_USB_EP_MSGIN, /* PC IN frames */
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,
    .bInterval            = 1,
  },
  .ep2 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = XCAN_USB_EP_MSGOUT, 
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,/* PC OUT frames */
    .wMaxPacketSize       = 64,
    .bInterval            = 1,
  },
};


static uint8_t device_init( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR *p_ep = &xcan_usb_dev.ep1;

  UNUSED( cfgidx );
  
  for( int i = 0; i < xcan_usb_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    
    if( p_ep[i].bmAttributes == USB_ENDPOINT_TYPE_BULK )
    {
      if( pdev->dev_speed == USBD_SPEED_FULL )
        ;
      else if( pdev->dev_speed == USBD_SPEED_HIGH )
        ;
      else
        assert( 0 );
    }
    
    USBD_LL_OpenEP( pdev, ep_addr,
                          p_ep[i].bmAttributes,
                          p_ep[i].wMaxPacketSize );
    
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 1;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 1;
  }
    
  pdev->pClassData = (void*)&xcan_data;

  USBD_LL_PrepareReceive( pdev, XCAN_USB_EP_MSGOUT, xcan_data.data1_ep_buffer, sizeof( xcan_data.data1_ep_buffer ) );
  
  return USBD_OK;
}

static uint8_t device_deinit( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR const *p_ep = &xcan_usb_dev.ep1;

  UNUSED( cfgidx );
  
  for( int i = 0; i < xcan_usb_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    USBD_LL_CloseEP( pdev, ep_addr );
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 0;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 0;
  }
  
  pdev->pClassData = (void*)0;
  return USBD_OK;
}

void xcan_usb_poll( void )
{
  HAL_PCD_IRQHandler( &hpcd_USB_FS );
}

static uint8_t  device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{
  UNUSED( pdev );
  switch( req->bRequest )
  {
    default:
      break;
  }
  return USBD_OK;
}

static uint8_t device_ep0_rx_ready( USBD_HandleTypeDef *pdev )
{
  UNUSED( pdev );
  return USBD_OK;
}

/* data was sent to PC */
static uint8_t device_data_in( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  struct t_class_data *p_data = (void*)pdev->pClassData;
  
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
/* use ZLP */
#if 1
  PCD_HandleTypeDef *hpcd = pdev->pData;
  uint32_t len = pdev->ep_in[epnum].total_length;
  /* packet is multiple of maxpacket, so tell host what all transfer is done */
  if( len && ( len % hpcd->IN_ep[epnum].maxpacket ) == 0U )
  {
    /* update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;
    /* send ZLP */
    USBD_LL_Transmit( pdev, epnum, NULL, 0U );
  }
  else
  {
    /* tx done, no active transfer */
    p_data->ep_tx_in_use[epnum] = 0;
  }
#else
  pdev->ep_in[epnum].total_length = 0U;
  p_data->ep_tx_in_use[epnum] = 0;
#endif
  return USBD_OK;
}

/* data was received from PC */
static uint8_t device_data_out( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  int size;
   
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
  size = USBD_LL_GetRxDataSize( pdev, epnum );
  
  if( epnum == XCAN_USB_EP_MSGOUT )
  {
    xcan_protocol_process_data( xcan_data.data1_ep_buffer, size );
    USBD_LL_PrepareReceive( pdev, XCAN_USB_EP_MSGOUT, xcan_data.data1_ep_buffer, sizeof( xcan_data.data1_ep_buffer ) );
  }
  else
  {
    return USBD_FAIL;
  }

  return USBD_OK;
}

static uint8_t *device_get_hs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_xcan_description );
  return (void*)&xcan_usb_dev;
}

static uint8_t *device_get_fs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_xcan_description );
  return (void*)&xcan_usb_dev;
}

static uint8_t *device_get_other_speed_cfg( uint16_t *length )
{
  *length = sizeof( struct t_xcan_description );
  return (void*)&xcan_usb_dev;
}

static uint8_t *device_get_device_qualifier( uint16_t *length )
{
  *length = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR );
  
  return (void*)&dev_qua;
}

USBD_ClassTypeDef usbd_xcan =
{
  .Init = device_init,
  .DeInit = device_deinit,
  .Setup = device_setup,
  .EP0_TxSent = 0,
  .EP0_RxReady = device_ep0_rx_ready,
  .DataIn = device_data_in,
  .DataOut = device_data_out,
  .SOF = 0,
  .IsoINIncomplete = 0,
  .IsoOUTIncomplete = 0,
  .GetHSConfigDescriptor = device_get_hs_cfg,
  .GetFSConfigDescriptor = device_get_fs_cfg,
  .GetOtherSpeedConfigDescriptor = device_get_other_speed_cfg,
  .GetDeviceQualifierDescriptor = device_get_device_qualifier,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  .GetUsrStrDescriptor = 0,
#endif
};

void xcan_usb_init( void )
{
  if( USBD_Init( &hUsbDeviceFS, &FS_Desc, DEVICE_FS ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_RegisterClass( &hUsbDeviceFS, &usbd_xcan ) != USBD_OK )
  {
    assert( 0 );
  }

  if( USBD_Start(&hUsbDeviceFS) != USBD_OK )
  {
    assert( 0 );
  }
}

int xcan_flush_ep( uint8_t ep )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  p_data->ep_tx_in_use[ep&0x0F] = 0;
  return USBD_LL_FlushEP( pdev, ep ) == USBD_OK;
}

int xcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
  USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  if( !p_data )
    return 0;

  switch( pfsm->state )
  {
    case 1:
      if( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] )
        return 0;
      pfsm->state = 0;
      /* fall through */
    case 0:
      assert( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] == 0 );
      //size = (size+(64-1))&(~(64-1));
      if( size > pfsm->dbsize )
        break;
      memcpy( pfsm->pdbuf, src, size );
      p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] = 1;
      /* prepare data transmit */
      pdev->ep_in[pfsm->ep_addr & EP_ADDR_MSK].total_length = size;
      USBD_LL_Transmit( pdev, pfsm->ep_addr, pfsm->pdbuf, size );
      
      pfsm->total_tx += size;
      pfsm->state = 1;
      return 1;
  }
  
  return 0;
}
