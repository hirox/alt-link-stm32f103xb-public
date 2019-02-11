/**
  ******************************************************************************
  * @file    usbd.c
  */

#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc.h"
#include "usbd_cdc_io.h"

extern USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops;
extern const USBD_CDC_ItfTypeDef USBD_CDC_fops;

USBD_CUSTOM_HID_HandleTypeDef hidClassData;

extern const uint8_t usbd_hid_ep_intin;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 


uint8_t  USBD_CDC_Init(USBD_HandleTypeDef *pdev,  uint8_t cfgidx);
uint8_t  USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t  USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
uint8_t  USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

void USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev);
uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);

uint8_t I2C_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t I2C_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t USBD_Class_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    USBD_HID_Init(pdev, cfgidx);
    USBD_CDC_Init(pdev, cfgidx);
    return 0;
}
/**
  * @brief  USBD_CUSTOM_HID_Init
  *         DeInitialize the CUSTOM_HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_Class_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    (void)cfgidx;

    // HID
    /* Close CUSTOM_HID EP IN */
    USBD_LL_CloseEP(pdev, CUSTOM_HID_EPIN_ADDR);

    /* Close CUSTOM_HID EP OUT */
    USBD_LL_CloseEP(pdev, CUSTOM_HID_EPOUT_ADDR);

    USBD_CustomHID_fops.DeInit();

    // CDC
    /* Close EP IN/OUT/CMD */
    USBD_LL_CloseEP(pdev, CDC_IN_EP);
    USBD_LL_CloseEP(pdev, CDC_IN_EP2);
    USBD_LL_CloseEP(pdev, CDC_IN_EP3);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP2);
    USBD_LL_CloseEP(pdev, CDC_OUT_EP3);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP2);
    USBD_LL_CloseEP(pdev, CDC_CMD_EP3);

    USBD_CDC_fops.DeInit();

    return USBD_OK;
}

extern const uint8_t usbd_hid_if_num;
static uint8_t USBD_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR)
    {
      return I2C_Setup(pdev, req);
    }
    else
    {
        if (LOBYTE(req->wIndex) == usbd_hid_if_num) {
            return USBD_HID_Setup(pdev, req);
        } else {
            return USBD_CDC_Setup(pdev, req);
        }
    }
}

/**
  * @brief  USBD_CUSTOM_HID_GetCfgDesc 
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
extern const uint8_t USBD_ConfigDescriptor[];
extern const uint16_t USBD_ConfigDescriptorSize;
static const uint8_t *USBD_GetCfgDesc (uint16_t *length)
{
  *length = USBD_ConfigDescriptorSize;
  return USBD_ConfigDescriptor;
}

uint8_t USBD_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (usbd_hid_ep_intin == epnum) {
        return USBD_HID_DataIn(pdev, epnum);
    } else {
        return USBD_CDC_DataIn(pdev, epnum);
    }
}

uint8_t USBD_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (usbd_hid_ep_intin == epnum) {
        return USBD_HID_DataOut(pdev, epnum);
    } else {
        return USBD_CDC_DataOut(pdev, epnum);
    }
}

uint8_t USBD_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    if ((pdev->request.bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR)
    {
      return I2C_EP0_RxReady(pdev);
    }
    else
    {
        if (LOBYTE(pdev->request.wIndex) == usbd_hid_if_num) {
            return USBD_HID_EP0_RxReady(pdev);
        } else {
            return USBD_CDC_EP0_RxReady(pdev);
        }
    }
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
extern const uint8_t USBD_DeviceQualifier[];

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static const uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

static const uint8_t *USBD_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = sizeof(USBD_DeviceQualifierDesc);
  return USBD_DeviceQualifierDesc;
}

const USBD_ClassTypeDef USBD_HANDLER = 
{
  USBD_Class_Init,
  USBD_Class_DeInit,
  USBD_Setup,
  NULL, /*EP0_TxSent*/  
  USBD_EP0_RxReady, /*EP0_RxReady*/ /* STATUS STAGE IN */
  USBD_DataIn, /*DataIn*/
  USBD_DataOut,
  NULL, /*SOF */
  NULL,
  NULL,      
  USBD_GetCfgDesc,
  USBD_GetCfgDesc, 
  USBD_GetCfgDesc,
  USBD_GetDeviceQualifierDesc,
};
