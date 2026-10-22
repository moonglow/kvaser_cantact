#include "usbd_core.h"
#include "usbd_conf.h"

#define USBD_VID                    0x0bfd
#define USBD_PID_FS                 0x0120   
#define USBD_LANGID_STRING          1033
#define USBD_MAX_STR_DESC_SIZ       0x100U

__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  255,                        /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x01,                       /*bcdDevice*/
  0x00,
  1,                         /*Index of manufacturer  string*/
  2,                          /*Index of product string*/
  3,                          /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

/* internal string descriptor */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

uint8_t * USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t * USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

uint8_t* USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString( (uint8_t *)"Xvaser", USBD_StrDesc, length );

  return USBD_StrDesc;
}

uint8_t* USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString( (uint8_t *)"Xvaser LL v2", USBD_StrDesc, length );

  return USBD_StrDesc;
}

/* 96-bit STM32 unique ID as 24 hex digits, the format candleLight reports */
uint8_t* USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  static const char hex[] = "0123456789ABCDEF";
  const uint32_t *uid = (const uint32_t *)UID_BASE;
  char serial[25];

  UNUSED(speed);
  for( int w = 0; w < 3; w++ )
  {
    for( int n = 0; n < 8; n++ )
    {
      serial[w*8 + n] = hex[(uid[w] >> (28 - 4*n)) & 0x0F];
    }
  }
  serial[24] = 0;
  USBD_GetString( (uint8_t *)serial, USBD_StrDesc, length );

  return USBD_StrDesc;
}

uint8_t * USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  
  USBD_GetString((uint8_t *)"Config", USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t * USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  
  USBD_GetString((uint8_t *)"Interface", USBD_StrDesc, length);
  return USBD_StrDesc;
}

USBD_DescriptorsTypeDef FS_Desc =
{
  .GetDeviceDescriptor = USBD_FS_DeviceDescriptor,
  .GetLangIDStrDescriptor = USBD_FS_LangIDStrDescriptor,
  .GetManufacturerStrDescriptor = USBD_FS_ManufacturerStrDescriptor,
  .GetProductStrDescriptor = USBD_FS_ProductStrDescriptor,
  .GetSerialStrDescriptor = USBD_FS_SerialStrDescriptor,
  .GetConfigurationStrDescriptor = USBD_FS_ConfigStrDescriptor,
  .GetInterfaceStrDescriptor = USBD_FS_InterfaceStrDescriptor,
};
