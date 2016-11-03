/**
 * @file    usb_lib.c
 * @brief   USB library
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "RTL.h"
#include "rl_usb.h"
#include "usb.h"

#pragma thumb
#pragma O3


/*------------------------------------------------------------------------------
 *      Library for usb_config.c
 *----------------------------------------------------------------------------*/

#ifdef  __USB_CONFIG__

/*------------------------------------------------------------------------------
 *      USB Device Configuration
 *----------------------------------------------------------------------------*/

#if    (USBD_ENABLE)

U8 USBD_AltSetting[USBD_IF_NUM];
U8 USBD_EP0Buf[USBD_MAX_PACKET0];
const U8 usbd_power = USBD_POWER;
const U8 usbd_hs_enable = USBD_HS_ENABLE;
const U16 usbd_if_num = USBD_IF_NUM;
const U8 usbd_ep_num = USBD_EP_NUM;
const U8 usbd_max_packet0 = USBD_MAX_PACKET0;


/*------------------------------------------------------------------------------
 *      USB Device Class Configuration
 *----------------------------------------------------------------------------*/

#if    (!USBD_HID_BINTERVAL)
#define USBD_HID_INTERVAL                1
#else
#define USBD_HID_INTERVAL                USBD_HID_BINTERVAL
#endif
#if    (!USBD_HID_HS_BINTERVAL)
#define USBD_HID_HS_INTERVAL             1
#else
#define USBD_HID_HS_INTERVAL            (2 << ((USBD_HID_HS_BINTERVAL & 0x0F)-1))
#endif

#if    (USBD_HID_ENABLE)
const U8 usbd_hid_if_num = USBD_HID_IF_NUM;
const U8 usbd_hid_ep_intin = USBD_HID_EP_INTIN;
const U8 usbd_hid_ep_intout = USBD_HID_EP_INTOUT;
#endif

#ifndef USBD_CDC_ACM_ENABLE
#if    (USBD_CDC_ENABLE == 1)
#error "Please update usb_config.c file with new definitions for CDC, as new CDC is incompatible with the old one!"
#else
#define USBD_CDC_ACM_ENABLE  0
#endif
#endif

#if     (USBD_CDC_ACM_ENABLE)
const U8 usbd_cdc_acm_cif_num = USBD_CDC_ACM_CIF_NUM;
const U8 usbd_cdc_acm_dif_num = USBD_CDC_ACM_DIF_NUM;
#endif

/*------------------------------------------------------------------------------
 *      CDC Sizes
 *----------------------------------------------------------------------------*/
#define CDC_HEADER_SIZE                         5
#define CDC_CALL_MANAGEMENT_SIZE                5
#define CDC_ABSTRACT_CONTROL_MANAGEMENT_SIZE    4
#define CDC_UNION_SIZE                          5

/*------------------------------------------------------------------------------
 *      USB Device Descriptors
 *----------------------------------------------------------------------------*/
#define USBD_MSC_DESC_LEN                 (USB_INTERFACE_DESC_SIZE + 2*USB_ENDPOINT_DESC_SIZE)
#define USBD_CDC_ACM_DESC_LEN             (USBD_MULTI_IF * USB_INTERFACE_ASSOC_DESC_SIZE                                                        + \
                                           /* CDC Interface 1 */                                                                                  \
                                           USB_INTERFACE_DESC_SIZE + CDC_HEADER_SIZE + CDC_CALL_MANAGEMENT_SIZE                                 + \
                                           CDC_ABSTRACT_CONTROL_MANAGEMENT_SIZE + CDC_UNION_SIZE + USB_ENDPOINT_DESC_SIZE                       + \
                                           /* CDC Interface 2 */                                                                                  \
                                           USB_INTERFACE_DESC_SIZE + USB_ENDPOINT_DESC_SIZE + USB_ENDPOINT_DESC_SIZE)
#define USBD_HID_DESC_LEN                 (USB_INTERFACE_DESC_SIZE + USB_HID_DESC_SIZE                                                          + \
                                          (USB_ENDPOINT_DESC_SIZE*(1+(USBD_HID_EP_INTOUT != 0))))
#define USBD_HID_DESC_OFS                 (USB_CONFIGUARTION_DESC_SIZE + USB_INTERFACE_DESC_SIZE                                                + \
                                           USBD_MSC_ENABLE * USBD_MSC_DESC_LEN + USBD_CDC_ACM_ENABLE * USBD_CDC_ACM_DESC_LEN + CDC_ENDPOINT2 * USBD_CDC_ACM_DESC_LEN)

#define USBD_WTOTALLENGTH                 (USB_CONFIGUARTION_DESC_SIZE +                 \
                                           USBD_CDC_ACM_DESC_LEN * USBD_CDC_ACM_ENABLE + \
                                           USBD_CDC_ACM_DESC_LEN * CDC_ENDPOINT2 + \
                                           USBD_HID_DESC_LEN     * USBD_HID_ENABLE     + \
                                           USBD_MSC_DESC_LEN     * USBD_MSC_ENABLE)

/*------------------------------------------------------------------------------
  Default HID Report Descriptor
 *----------------------------------------------------------------------------*/

/*   Bit    Input       Output
      0     IN0          OUT0
      1     IN1          OUT1
      2     IN2          OUT2
      3     IN3          OUT3
      4     IN4          OUT4
      5     IN5          OUT5
      6     IN6          OUT6
      7     IN7          OUT7
*/

__weak \
const U8 USBD_HID_ReportDescriptor[] = {
    HID_UsagePageVendor(0x00),
    HID_Usage(0x01),
    HID_Collection(HID_Application),
    HID_LogicalMin(0),                              /* value range: 0 - 0xFF */
    HID_LogicalMaxS(0xFF),
    HID_ReportSize(8),                              /* 8 bits */
#if (USBD_HID_INREPORT_MAX_SZ > 255)
    HID_ReportCountS(USBD_HID_INREPORT_MAX_SZ),
#else
    HID_ReportCount(USBD_HID_INREPORT_MAX_SZ),
#endif
    HID_Usage(0x01),
    HID_Input(HID_Data | HID_Variable | HID_Absolute),
#if (USBD_HID_OUTREPORT_MAX_SZ > 255)
    HID_ReportCountS(USBD_HID_OUTREPORT_MAX_SZ),
#else
    HID_ReportCount(USBD_HID_OUTREPORT_MAX_SZ),
#endif
    HID_Usage(0x01),
    HID_Output(HID_Data | HID_Variable | HID_Absolute),
#if (USBD_HID_FEATREPORT_MAX_SZ > 255)
    HID_ReportCountS(USBD_HID_FEATREPORT_MAX_SZ),
#else
    HID_ReportCount(USBD_HID_FEATREPORT_MAX_SZ),
#endif
    HID_Usage(0x01),
    HID_Feature(HID_Data | HID_Variable | HID_Absolute),
    HID_EndCollection,
};

__weak \
const U16 USBD_HID_ReportDescriptorSize = sizeof(USBD_HID_ReportDescriptor);

__weak \
const U16 USBD_HID_DescriptorOffset     = USBD_HID_DESC_OFS;

/* USB Device Standard Descriptor */
__weak \
const U8 USBD_DeviceDescriptor[] = {
    USB_DEVICE_DESC_SIZE,                 /* bLength */
    USB_DEVICE_DESCRIPTOR_TYPE,           /* bDescriptorType */
#if ((USBD_HS_ENABLE) || (USBD_MULTI_IF))
    WBVAL(0x0200), /* 2.00 */             /* bcdUSB */
#else
    WBVAL(0x0110), /* 1.10 */             /* bcdUSB */
#endif
#if (USBD_MULTI_IF)
    USB_DEVICE_CLASS_MISCELLANEOUS,       /* bDeviceClass */
    0x02,                                 /* bDeviceSubClass */
    0x01,                                 /* bDeviceProtocol */
#elif (USBD_CDC_ACM_ENABLE)
    USB_DEVICE_CLASS_COMMUNICATIONS,      /* bDeviceClass CDC*/
    0x00,                                 /* bDeviceSubClass */
    0x00,                                 /* bDeviceProtocol */
#else
    0x00,                                 /* bDeviceClass */
    0x00,                                 /* bDeviceSubClass */
    0x00,                                 /* bDeviceProtocol */
#endif
    USBD_MAX_PACKET0,                     /* bMaxPacketSize0 */
    WBVAL(USBD_DEVDESC_IDVENDOR),         /* idVendor */
    WBVAL(USBD_DEVDESC_IDPRODUCT),        /* idProduct */
    WBVAL(USBD_DEVDESC_BCDDEVICE),        /* bcdDevice */
    0x01,                                 /* iManufacturer */
    0x02,                                 /* iProduct */
    0x03 * USBD_STRDESC_SER_ENABLE,       /* iSerialNumber */
    0x01                                  /* bNumConfigurations: one possible configuration*/
};
const uint16_t USBD_DeviceDescriptorSize = sizeof(USBD_DeviceDescriptor);


#if (USBD_HS_ENABLE)
/* USB Device Qualifier Descriptor (for Full Speed) */
__weak \
const U8 USBD_DeviceQualifier[] = {
    USB_DEVICE_QUALI_SIZE,                /* bLength */
    USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE, /* bDescriptorType */
    WBVAL(0x0200), /* 2.00 */             /* bcdUSB */
    0x00,                                 /* bDeviceClass */
    0x00,                                 /* bDeviceSubClass */
    0x00,                                 /* bDeviceProtocol */
    USBD_MAX_PACKET0,                     /* bMaxPacketSize0 */
    0x01,                                 /* bNumConfigurations */
    0x00                                  /* bReserved */
};

/* USB Device Qualifier Descriptor for High Speed */
__weak \
const U8 USBD_DeviceQualifier_HS[] = {
    USB_DEVICE_QUALI_SIZE,                /* bLength */
    USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE, /* bDescriptorType */
    WBVAL(0x0200), /* 2.00 */             /* bcdUSB */
    0x00,                                 /* bDeviceClass */
    0x00,                                 /* bDeviceSubClass */
    0x00,                                 /* bDeviceProtocol */
    USBD_MAX_PACKET0,                     /* bMaxPacketSize0 */
    0x01,                                 /* bNumConfigurations */
    0x00                                  /* bReserved */
};
#else
/* USB Device Qualifier Descriptor (for Full Speed) */
__weak \
const U8 USBD_DeviceQualifier[]    = { 0 };

/* USB Device Qualifier Descriptor for High Speed */
__weak \
const U8 USBD_DeviceQualifier_HS[] = { 0 };
#endif

#define HID_DESC                                                                                            \
  /* Interface, Alternate Setting 0, HID Class */                                                           \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  USBD_HID_IF_NUM,                      /* bInterfaceNumber */                                              \
  0x00,                                 /* bAlternateSetting */                                             \
  0x01+(USBD_HID_EP_INTOUT != 0),       /* bNumEndpoints */                                                 \
  USB_DEVICE_CLASS_HUMAN_INTERFACE,     /* bInterfaceClass */                                               \
  HID_SUBCLASS_NONE,                    /* bInterfaceSubClass */                                            \
  HID_PROTOCOL_NONE,                    /* bInterfaceProtocol */                                            \
  USBD_HID_IF_STR_NUM,                  /* iInterface */                                                    \
                                                                                                            \
/* HID Class Descriptor */                                                                                  \
  USB_HID_DESC_SIZE,                    /* bLength */                                                       \
  HID_HID_DESCRIPTOR_TYPE,              /* bDescriptorType */                                               \
  WBVAL(0x0100), /* 1.00 */             /* bcdHID */                                                        \
  0x00,                                 /* bCountryCode */                                                  \
  0x01,                                 /* bNumDescriptors */                                               \
  HID_REPORT_DESCRIPTOR_TYPE,           /* bDescriptorType */                                               \
  WBVAL(USB_HID_REPORT_DESC_SIZE),      /* wDescriptorLength */

#define HID_EP                          /* HID Endpoint for Low-speed/Full-speed */                         \
/* Endpoint, HID Interrupt In */                                                                            \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_HID_EP_INTIN),   /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  USBD_HID_BINTERVAL,                   /* bInterval */

#define HID_EP_INOUT                    /* HID Endpoint for Low-speed/Full-speed */                         \
/* Endpoint, HID Interrupt In */                                                                            \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_HID_EP_INTIN),   /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  USBD_HID_BINTERVAL,                   /* bInterval */                                                     \
                                                                                                            \
/* Endpoint, HID Interrupt Out */                                                                           \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_HID_EP_INTOUT), /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  USBD_HID_BINTERVAL,                   /* bInterval */

#define HID_EP_HS                       /* HID Endpoint for High-speed */                                   \
/* Endpoint, HID Interrupt In */                                                                            \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_HID_EP_INTIN),   /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_HID_HS_BINTERVAL,                /* bInterval */

#define HID_EP_INOUT_HS                 /* HID Endpoint for High-speed */                                   \
/* Endpoint, HID Interrupt In */                                                                            \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_HID_EP_INTIN),   /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_HID_HS_BINTERVAL,                /* bInterval */                                                     \
                                                                                                            \
/* Endpoint, HID Interrupt Out */                                                                           \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_HID_EP_INTOUT), /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_HID_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_HID_HS_BINTERVAL,                /* bInterval */

#define MSC_DESC                                                                                            \
/* Interface, Alternate Setting 0, MSC Class */                                                             \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  USBD_MSC_IF_NUM,                      /* bInterfaceNumber */                                              \
  0x00,                                 /* bAlternateSetting */                                             \
  0x02,                                 /* bNumEndpoints */                                                 \
  USB_DEVICE_CLASS_STORAGE,             /* bInterfaceClass */                                               \
  MSC_SUBCLASS_SCSI,                    /* bInterfaceSubClass */                                            \
  MSC_PROTOCOL_BULK_ONLY,               /* bInterfaceProtocol */                                            \
  USBD_MSC_IF_STR_NUM,                  /* iInterface */

#define MSC_EP                          /* MSC Endpoints for Low-speed/Full-speed */                        \
/* Endpoint, EP Bulk IN */                                                                                  \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_MSC_EP_BULKIN),  /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_MSC_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  0x00,                                 /* bInterval: ignore for Bulk transfer */                           \
                                                                                                            \
/* Endpoint, EP Bulk OUT */                                                                                 \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_MSC_EP_BULKOUT),/* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_MSC_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  0x00,                                 /* bInterval: ignore for Bulk transfer */

#define MSC_EP_HS                       /* MSC Endpoints for High-speed */                                  \
/* Endpoint, EP Bulk IN */                                                                                  \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN(USBD_MSC_EP_BULKIN),  /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_MSC_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_MSC_HS_BINTERVAL,                /* bInterval */                                                     \
                                                                                                            \
/* Endpoint, EP Bulk OUT */                                                                                 \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_MSC_EP_BULKOUT),/* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_MSC_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_MSC_HS_BINTERVAL,                /* bInterval */

#define ADC_DESC_IAD(first,num_of_ifs)  /* ADC: Interface Association Descriptor */                         \
  USB_INTERFACE_ASSOC_DESC_SIZE,        /* bLength */                                                       \
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,  /* bDescriptorType */                                         \
 (first),                               /* bFirstInterface */                                               \
 (num_of_ifs),                          /* bInterfaceCount */                                               \
  USB_DEVICE_CLASS_AUDIO,               /* bFunctionClass */                                                \
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bFunctionSubclass */                                             \
  AUDIO_PROTOCOL_UNDEFINED,             /* bFunctionProtocol */                                             \
  USBD_ADC_CIF_STR_NUM,                 /* iFunction */                                                     \

#define ADC_DESC                                                                                            \
/* Interface, Alternate Setting 0, Audio Control */                                                         \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  USBD_ADC_CIF_NUM,                     /* bInterfaceNumber */                                              \
  0x00,                                 /* bAlternateSetting */                                             \
  0x00,                                 /* bNumEndpoints */                                                 \
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                                               \
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */                                            \
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                                            \
  USBD_ADC_CIF_STR_NUM,                 /* iInterface */                                                    \
/* Audio Control Interface */                                                                               \
  AUDIO_CONTROL_INTERFACE_DESC_SZ(1),   /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */                                            \
  WBVAL(0x0100), /* 1.00 */             /* bcdADC */                                                        \
  WBVAL(                                /* wTotalLength */                                                  \
    AUDIO_CONTROL_INTERFACE_DESC_SZ(1) +                                                                    \
    AUDIO_INPUT_TERMINAL_DESC_SIZE     +                                                                    \
    AUDIO_FEATURE_UNIT_DESC_SZ(1,1)    +                                                                    \
    AUDIO_OUTPUT_TERMINAL_DESC_SIZE                                                                         \
  ),                                                                                                        \
  0x01,                                 /* bInCollection */                                                 \
  0x01,                                 /* baInterfaceNr */                                                 \
                                                                                                            \
/* Audio Input Terminal */                                                                                  \
  AUDIO_INPUT_TERMINAL_DESC_SIZE,       /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_CONTROL_INPUT_TERMINAL,         /* bDescriptorSubtype */                                            \
  0x01,                                 /* bTerminalID */                                                   \
  WBVAL(AUDIO_TERMINAL_USB_STREAMING),  /* wTerminalType */                                                 \
  0x00,                                 /* bAssocTerminal */                                                \
  0x01,                                 /* bNrChannels */                                                   \
  WBVAL(AUDIO_CHANNEL_M),               /* wChannelConfig */                                                \
  0x00,                                 /* iChannelNames */                                                 \
  0x00,                                 /* iTerminal */                                                     \
                                                                                                            \
/* Audio Feature Unit */                                                                                    \
  AUDIO_FEATURE_UNIT_DESC_SZ(1,1),      /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_CONTROL_FEATURE_UNIT,           /* bDescriptorSubtype */                                            \
  0x02,                                 /* bUnitID */                                                       \
  0x01,                                 /* bSourceID */                                                     \
  0x01,                                 /* bControlSize */                                                  \
  AUDIO_CONTROL_MUTE |                                                                                      \
  AUDIO_CONTROL_VOLUME,                 /* bmaControls(0) */                                                \
  0x00,                                 /* bmaControls(1) */                                                \
  0x00,                                 /* iTerminal */                                                     \
                                                                                                            \
/* Audio Output Terminal */                                                                                 \
  AUDIO_OUTPUT_TERMINAL_DESC_SIZE,      /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_CONTROL_OUTPUT_TERMINAL,        /* bDescriptorSubtype */                                            \
  0x03,                                 /* bTerminalID */                                                   \
  WBVAL(AUDIO_TERMINAL_SPEAKER),        /* wTerminalType */                                                 \
  0x00,                                 /* bAssocTerminal */                                                \
  0x02,                                 /* bSourceID */                                                     \
  0x00,                                 /* iTerminal */                                                     \
                                                                                                            \
/* Interface, Alternate Setting 0, Audio Streaming - Zero Bandwith */                                       \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  USBD_ADC_SIF1_NUM,                    /* bInterfaceNumber */                                              \
  0x00,                                 /* bAlternateSetting */                                             \
  0x00,                                 /* bNumEndpoints */                                                 \
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                                               \
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                                            \
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                                            \
  USBD_ADC_SIF1_STR_NUM,                /* iInterface */                                                    \
                                                                                                            \
/* Interface, Alternate Setting 1, Audio Streaming - Operational */                                         \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  USBD_ADC_SIF1_NUM,                    /* bInterfaceNumber */                                              \
  0x01,                                 /* bAlternateSetting */                                             \
  0x01,                                 /* bNumEndpoints */                                                 \
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */                                               \
  AUDIO_SUBCLASS_AUDIOSTREAMING,        /* bInterfaceSubClass */                                            \
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */                                            \
  USBD_ADC_SIF2_STR_NUM,                /* iInterface */                                                    \
                                                                                                            \
/* Audio Streaming Interface */                                                                             \
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,  /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_STREAMING_GENERAL,              /* bDescriptorSubtype */                                            \
  0x01,                                 /* bTerminalLink */                                                 \
  0x01,                                 /* bDelay */                                                        \
  WBVAL(AUDIO_FORMAT_PCM),              /* wFormatTag */                                                    \
                                                                                                            \
/* Audio Type I Format */                                                                                   \
  AUDIO_FORMAT_TYPE_I_DESC_SZ(1),       /* bLength */                                                       \
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */                                               \
  AUDIO_STREAMING_FORMAT_TYPE,          /* bDescriptorSubtype */                                            \
  AUDIO_FORMAT_TYPE_I,                  /* bFormatType */                                                   \
  0x01,                                 /* bNrChannels */                                                   \
  USBD_ADC_BSUBFRAMESIZE,               /* bSubFrameSize */                                                 \
  USBD_ADC_BBITRESOLUTION,              /* bBitResolution */                                                \
  0x01,                                 /* bSamFreqType */                                                  \
  B3VAL(USBD_ADC_TSAMFREQ),             /* tSamFreq */

#define ADC_EP                          /* ADC Endpoints for Low-speed/Full-speed */                        \
/* Endpoint, EP ISO OUT - Standard Descriptor */                                                            \
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_ADC_EP_ISOOUT), /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* bmAttributes */                                                  \
  WBVAL(USBD_ADC_WMAXPACKETSIZE),       /* wMaxPacketSize */                                                \
  USBD_ADC_BINTERVAL,                   /* bInterval */                                                     \
  0x00,                                 /* bRefresh */                                                      \
  0x00,                                 /* bSynchAddress */                                                 \
                                                                                                            \
/* Endpoint - Audio Streaming */                                                                            \
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */                                                       \
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */                                               \
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */                                                   \
  0x00,                                 /* bmAttributes */                                                  \
  0x00,                                 /* bLockDelayUnits */                                               \
  WBVAL(0x0000),                        /* wLockDelay */

#define ADC_EP_HS                       /* ADC Endpoints for High-speed */                                  \
/* Endpoint, EP ISO OUT - Standard Descriptor */                                                            \
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT(USBD_ADC_EP_ISOOUT), /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_ISOCHRONOUS,        /* bmAttributes */                                                  \
  WBVAL(USBD_ADC_HS_WMAXPACKETSIZE),    /* wMaxPacketSize */                                                \
  USBD_ADC_BINTERVAL,                   /* bInterval */                                                     \
  0x00,                                 /* bRefresh */                                                      \
  0x00,                                 /* bSynchAddress */                                                 \
                                                                                                            \
/* Endpoint - Audio Streaming */                                                                            \
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,   /* bLength */                                                       \
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */                                               \
  AUDIO_ENDPOINT_GENERAL,               /* bDescriptor */                                                   \
  0x00,                                 /* bmAttributes */                                                  \
  0x00,                                 /* bLockDelayUnits */                                               \
  WBVAL(0x0000),                        /* wLockDelay */

#define CDC_ACM_DESC_IAD(first,num_of_ifs)  /* CDC: Interface Association Descriptor */                     \
  USB_INTERFACE_ASSOC_DESC_SIZE,        /* bLength */                                                       \
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,  /* bDescriptorType */                                         \
 (first),                               /* bFirstInterface */                                               \
 (num_of_ifs),                          /* bInterfaceCount */                                               \
  CDC_COMMUNICATION_INTERFACE_CLASS,    /* bFunctionClass    (Communication Class) */                       \
  CDC_ABSTRACT_CONTROL_MODEL,           /* bFunctionSubclass (Abstract Control Model) */                    \
  0x01,                                 /* bFunctionProtocol (V.25ter, Common AT commands) */               \
  USBD_CDC_ACM_CIF_STR_NUM,             /* iFunction */                                                     \

#define CDC_ACM_DESC_IF0(cif, dif)                                                                          \
/* Interface, Alternate Setting 0, CDC Class */                                                             \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  (cif),                                /* bInterfaceNumber: Number of Interface */                         \
  0x00,                                 /* bAlternateSetting: Alternate setting */                          \
  0x01,                                 /* bNumEndpoints: One endpoint used */                              \
  CDC_COMMUNICATION_INTERFACE_CLASS,    /* bInterfaceClass: Communication Interface Class */                \
  CDC_ABSTRACT_CONTROL_MODEL,           /* bInterfaceSubClass: Abstract Control Model */                    \
  0x01,                                 /* bInterfaceProtocol: no protocol used */                          \
  USBD_CDC_ACM_CIF_STR_NUM,             /* iInterface: */                                                   \
                                                                                                            \
/* Header Functional Descriptor */                                                                          \
  CDC_HEADER_SIZE,                      /* bLength: Endpoint Descriptor size */                             \
  CDC_CS_INTERFACE,                     /* bDescriptorType: CS_INTERFACE */                                 \
  CDC_HEADER,                           /* bDescriptorSubtype: Header Func Desc */                          \
  WBVAL(CDC_V1_10), /* 1.10 */          /* bcdCDC */                                                        \
/* Call Management Functional Descriptor */                                                                 \
  CDC_CALL_MANAGEMENT_SIZE,             /* bFunctionLength */                                               \
  CDC_CS_INTERFACE,                     /* bDescriptorType: CS_INTERFACE */                                 \
  CDC_CALL_MANAGEMENT,                  /* bDescriptorSubtype: Call Management Func Desc */                 \
  0x03,                                 /* bmCapabilities: device handles call management */                \
  0x02,                                 /* bDataInterface: CDC data IF ID */                                \
/* Abstract Control Management Functional Descriptor */                                                     \
  CDC_ABSTRACT_CONTROL_MANAGEMENT_SIZE, /* bFunctionLength */                                               \
  CDC_CS_INTERFACE,                     /* bDescriptorType: CS_INTERFACE */                                 \
  CDC_ABSTRACT_CONTROL_MANAGEMENT,      /* bDescriptorSubtype: Abstract Control Management desc */          \
  0x06,                                 /* bmCapabilities: SET_LINE_CODING, GET_LINE_CODING, SET_CONTROL_LINE_STATE supported */ \
/* Union Functional Descriptor */                                                                           \
  CDC_UNION_SIZE,                       /* bFunctionLength */                                               \
  CDC_CS_INTERFACE,                     /* bDescriptorType: CS_INTERFACE */                                 \
  CDC_UNION,                            /* bDescriptorSubtype: Union func desc */                           \
  (cif),                                /* bMasterInterface: Communication class interface is master */     \
  (dif),                                /* bSlaveInterface0: Data class interface is slave 0 */

#define CDC_ACM_EP_IF0(ep)              /* CDC Endpoints for Interface 0 for Low-speed/Full-speed */        \
/* Endpoint, EP Interrupt IN */         /* event notification (optional) */                                 \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN((ep)),                /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_INTERRUPT,          /* bmAttributes */                                                  \
  WBVAL(USBD_CDC_ACM_WMAXPACKETSIZE),   /* wMaxPacketSize */                                                \
  USBD_CDC_ACM_BINTERVAL,               /* bInterval */

#define CDC_ACM_DESC_IF1(dif)                                                                               \
/* Interface, Alternate Setting 0, Data class interface descriptor*/                                        \
  USB_INTERFACE_DESC_SIZE,              /* bLength */                                                       \
  USB_INTERFACE_DESCRIPTOR_TYPE,        /* bDescriptorType */                                               \
  (dif),                                /* bInterfaceNumber: Number of Interface */                         \
  0x00,                                 /* bAlternateSetting: no alternate setting */                       \
  0x02,                                 /* bNumEndpoints: two endpoints used */                             \
  CDC_DATA_INTERFACE_CLASS,             /* bInterfaceClass: Data Interface Class */                         \
  0x00,                                 /* bInterfaceSubClass: no subclass available */                     \
  0x00,                                 /* bInterfaceProtocol: no protocol used */                          \
  USBD_CDC_ACM_DIF_STR_NUM,             /* iInterface */

#define CDC_ACM_EP_IF1(epout, epin)     /* CDC Endpoints for Interface 1 for Low-speed/Full-speed */        \
/* Endpoint, EP Bulk OUT */                                                                                 \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_OUT((epout)),            /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_CDC_ACM_WMAXPACKETSIZE1),  /* wMaxPacketSize */                                                \
  0x00,                                 /* bInterval: ignore for Bulk transfer */                           \
                                                                                                            \
/* Endpoint, EP Bulk IN */                                                                                  \
  USB_ENDPOINT_DESC_SIZE,               /* bLength */                                                       \
  USB_ENDPOINT_DESCRIPTOR_TYPE,         /* bDescriptorType */                                               \
  USB_ENDPOINT_IN((epin)),              /* bEndpointAddress */                                              \
  USB_ENDPOINT_TYPE_BULK,               /* bmAttributes */                                                  \
  WBVAL(USBD_CDC_ACM_WMAXPACKETSIZE1),  /* wMaxPacketSize */                                                \
  0x00,                                 /* bInterval: ignore for Bulk transfer */

/* USB Device Configuration Descriptor (for Full Speed) */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
__weak \
/*const*/ U8 USBD_ConfigDescriptor[] = {
    /* Configuration 1 */
    USB_CONFIGUARTION_DESC_SIZE,          /* bLength */
    USB_CONFIGURATION_DESCRIPTOR_TYPE,    /* bDescriptorType */
    WBVAL(USBD_WTOTALLENGTH),             /* wTotalLength */
    USBD_IF_NUM,                          /* bNumInterfaces */
    0x01,                                 /* bConfigurationValue: 0x01 is used to select this configuration */
    0x00,                                 /* iConfiguration: no string to describe this configuration */
    USBD_CFGDESC_BMATTRIBUTES |           /* bmAttributes */
    (USBD_POWER << 6),
    USBD_CFGDESC_BMAXPOWER,               /* bMaxPower, device power consumption */

#if (USBD_ADC_ENABLE)
#if (USBD_MULTI_IF)
    ADC_DESC_IAD(USBD_ADC_CIF_NUM, 2)
#endif
    ADC_DESC
    ADC_EP
#endif

#if (USBD_MSC_ENABLE)
    MSC_DESC
    MSC_EP
#endif

#if (USBD_HID_ENABLE)
    HID_DESC
#if (USBD_HID_EP_INTOUT != 0)
    HID_EP_INOUT
#else
    HID_EP
#endif
#endif

#if (USBD_CDC_ACM_ENABLE)
#if (USBD_MULTI_IF)
    CDC_ACM_DESC_IAD(USBD_CDC_ACM_CIF_NUM, 2)
#endif
    CDC_ACM_DESC_IF0(USBD_CDC_ACM_CIF_NUM, USBD_CDC_ACM_DIF_NUM)
    CDC_ACM_EP_IF0(USBD_CDC_ACM_EP_INTIN)
    CDC_ACM_DESC_IF1(USBD_CDC_ACM_DIF_NUM)
    CDC_ACM_EP_IF1(USBD_CDC_ACM_EP_BULKOUT, USBD_CDC_ACM_EP_BULKIN)
#endif

#if (CDC_ENDPOINT2)
#if (USBD_MULTI_IF)
    CDC_ACM_DESC_IAD(USBD_CDC_ACM_CIF_NUM2, 2)
#endif
    CDC_ACM_DESC_IF0(USBD_CDC_ACM_CIF_NUM2, USBD_CDC_ACM_DIF_NUM2)
    CDC_ACM_EP_IF0(USBD_CDC_ACM_EP_INTIN2)
    CDC_ACM_DESC_IF1(USBD_CDC_ACM_DIF_NUM2)
    CDC_ACM_EP_IF1(USBD_CDC_ACM_EP_BULKOUT2, USBD_CDC_ACM_EP_BULKIN2)
#endif

    /* Terminator */                                                                                            \
    0                                     /* bLength */                                                       \
};
const uint16_t USBD_ConfigDescriptorSize = sizeof(USBD_ConfigDescriptor);

#if (USBD_HS_ENABLE == 0)               /* If High-speed not enabled, declare dummy descriptors for High-speed */
__weak \
const U8 USBD_ConfigDescriptor_HS[] = { 0 };
__weak \
const U8 USBD_OtherSpeedConfigDescriptor[] = { 0 };
__weak \
const U8 USBD_OtherSpeedConfigDescriptor_HS[] = { 0 };
#endif

/* USB Device Create String Descriptor */
#define USBD_STR_DEF(n)                 \
  __packed struct {                              \
    U8  len;                            \
    U8  type;                           \
    wchar_t str[sizeof(USBD_##n)/2-1];      \
  } desc##n

#define USBD_STR_VAL(n)                  \
 { sizeof(USBD_##n), USB_STRING_DESCRIPTOR_TYPE, USBD_##n }

__weak \
const __packed struct {
    __packed struct {
        U8  len;
        U8  type;
        U16 langid;
    } desc_langid;
    USBD_STR_DEF(STRDESC_MAN);
    USBD_STR_DEF(STRDESC_PROD);
#if  (USBD_STRDESC_SER_ENABLE)
    USBD_STR_DEF(STRDESC_SER);
#endif
#if (USBD_ADC_ENABLE)
    USBD_STR_DEF(ADC_CIF_STRDESC);
    USBD_STR_DEF(ADC_SIF1_STRDESC);
    USBD_STR_DEF(ADC_SIF2_STRDESC);
#endif
#if (USBD_CDC_ACM_ENABLE)
    USBD_STR_DEF(CDC_ACM_CIF_STRDESC);
    USBD_STR_DEF(CDC_ACM_DIF_STRDESC);
#endif
#if (USBD_HID_ENABLE)
    USBD_STR_DEF(HID_STRDESC);
#endif
#if (USBD_MSC_ENABLE)
    USBD_STR_DEF(MSC_STRDESC);
#endif
} USBD_StringDescriptor
= {
    { 4, USB_STRING_DESCRIPTOR_TYPE, USBD_STRDESC_LANGID },
    USBD_STR_VAL(STRDESC_MAN),
    USBD_STR_VAL(STRDESC_PROD),
#if (USBD_STRDESC_SER_ENABLE)
    USBD_STR_VAL(STRDESC_SER),
#endif
#if (USBD_ADC_ENABLE)
    USBD_STR_VAL(ADC_CIF_STRDESC),
    USBD_STR_VAL(ADC_SIF1_STRDESC),
    USBD_STR_VAL(ADC_SIF2_STRDESC),
#endif
#if (USBD_CDC_ACM_ENABLE)
    USBD_STR_VAL(CDC_ACM_CIF_STRDESC),
    USBD_STR_VAL(CDC_ACM_DIF_STRDESC),
#endif
#if (USBD_HID_ENABLE)
    USBD_STR_VAL(HID_STRDESC),
#endif
#if (USBD_MSC_ENABLE)
    USBD_STR_VAL(MSC_STRDESC),
#endif
};

#endif

#endif  /* __USB_CONFIG__ */
