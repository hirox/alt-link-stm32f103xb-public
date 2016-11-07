/**
  ******************************************************************************
  * @file    usbd_customhid.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the CUSTOM_HID core functions.
  *
  * @verbatim
  *      
  *          ===================================================================      
  *                                CUSTOM_HID Class  Description
  *          =================================================================== 
  *           This module manages the CUSTOM_HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (CUSTOM_HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - Usage Page : Generic Desktop
  *             - Usage : Vendor
  *             - Collection : Application 
  *      
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *           
  *      
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc.h"
#include "usbd_cdc_io.h"

extern const uint16_t USBD_HID_DescriptorOffset;
extern const uint16_t USBD_HID_ReportDescriptorSize;

extern USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops;
extern USBD_CDC_ItfTypeDef USBD_CDC_fops;

USBD_CUSTOM_HID_HandleTypeDef hidClassData;

extern const uint8_t usbd_hid_ep_intin;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 


static uint8_t USBD_Class_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_Class_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t  USBD_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  USBD_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_EP0_RxReady (USBD_HandleTypeDef  *pdev);

static uint8_t  *USBD_GetCfgDesc (uint16_t *length);
static uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc (uint16_t *length);

uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t  USBD_CDC_Init (USBD_HandleTypeDef *pdev,  uint8_t cfgidx);
uint8_t  USBD_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);
uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Variables
  * @{
  */ 

USBD_ClassTypeDef  USBD_CUSTOM_HID = 
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
  USBD_CUSTOM_HID_GetDeviceQualifierDesc,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CUSTOM_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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


static void USBD_HID_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                 CUSTOM_HID_EPIN_ADDR,
                 USBD_EP_TYPE_INTR,
                 CUSTOM_HID_EPIN_SIZE);  

    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                 CUSTOM_HID_EPOUT_ADDR,
                 USBD_EP_TYPE_INTR,
                 CUSTOM_HID_EPOUT_SIZE);

    USBD_CUSTOM_HID_HandleTypeDef *hhid = &hidClassData;
      
    hhid->state = CUSTOM_HID_IDLE;
    USBD_CustomHID_fops.Init();
          /* Prepare Out endpoint to receive 1st packet */ 
    USBD_LL_PrepareReceive(pdev, CUSTOM_HID_EPOUT_ADDR, hhid->Report_buf, 
                           USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);
}




static uint8_t  USBD_Class_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
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
static uint8_t  USBD_Class_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
    // HID
    /* Close CUSTOM_HID EP IN */
    USBD_LL_CloseEP(pdev,
                  CUSTOM_HID_EPIN_ADDR);

    /* Close CUSTOM_HID EP OUT */
    USBD_LL_CloseEP(pdev,
                  CUSTOM_HID_EPOUT_ADDR);

    USBD_CustomHID_fops.DeInit();

    // CDC
    /* Close EP IN */
    USBD_LL_CloseEP(pdev,
              CDC_IN_EP);

    /* Close EP OUT */
    USBD_LL_CloseEP(pdev,
              CDC_OUT_EP);

    /* Close Command IN EP */
    USBD_LL_CloseEP(pdev,
              CDC_CMD_EP);

    USBD_CDC_fops.DeInit();

    return USBD_OK;
}



/**
  * @brief  USBD_CUSTOM_HID_Setup
  *         Handle the CUSTOM_HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
  uint16_t len = 0;
  uint8_t  *pbuf = NULL;
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = &hidClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :  
    switch (req->bRequest)
    {
    case CUSTOM_HID_REQ_SET_PROTOCOL:
      hhid->Protocol = (uint8_t)(req->wValue);
      break;
      
    case CUSTOM_HID_REQ_GET_PROTOCOL:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->Protocol,
                        1);    
      break;
      
    case CUSTOM_HID_REQ_SET_IDLE:
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;
      
    case CUSTOM_HID_REQ_GET_IDLE:
      USBD_CtlSendData (pdev, 
                        (uint8_t *)&hhid->IdleState,
                        1);        
      break;      
    
    case CUSTOM_HID_REQ_SET_REPORT:
      hhid->IsReportAvailable = 1;
      USBD_CtlPrepareRx (pdev, hhid->Report_buf, (uint8_t)(req->wLength));
      
      break;
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL; 
    }
    break;
    
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( req->wValue >> 8 == CUSTOM_HID_REPORT_DESC)
      {
        len = MIN(USBD_HID_ReportDescriptorSize , req->wLength);
        pbuf =  USBD_CustomHID_fops.pReport;
      }
      else if( req->wValue >> 8 == CUSTOM_HID_DESCRIPTOR_TYPE)
      {
//        pbuf = USBD_CUSTOM_HID_Desc;   
//        len = MIN(USB_CUSTOM_HID_DESC_SIZ , req->wLength);

        pbuf = USBD_CustomHID_fops.pReport + USBD_HID_DescriptorOffset;
        len = MIN(USB_CUSTOM_HID_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&hhid->AltSetting,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      hhid->AltSetting = (uint8_t)(req->wValue);
      break;
    }
  }
  return USBD_OK;
}



extern const uint8_t usbd_hid_if_num;
static uint8_t  USBD_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
    if (LOBYTE(req->wIndex) == usbd_hid_if_num) {
        return USBD_HID_Setup(pdev, req);
    } else {
        return USBD_CDC_Setup(pdev, req);
    }
}


/**
  * @brief  USBD_CUSTOM_HID_SendReport 
  *         Send CUSTOM_HID Report
  * @param  pdev: device instance
  * @param  buff: pointer to report
  * @retval status
  */
uint8_t USBD_CUSTOM_HID_SendReport     (USBD_HandleTypeDef  *pdev, 
                                 uint8_t *report,
                                 uint16_t len)
{
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = &hidClassData;
  
  if (pdev->dev_state == USBD_STATE_CONFIGURED )
  {
    if(hhid->state == CUSTOM_HID_IDLE)
    {
      hhid->state = CUSTOM_HID_BUSY;
      USBD_LL_Transmit (pdev, 
                        CUSTOM_HID_EPIN_ADDR,
                        report,
                        len);
    }
  }
  return USBD_OK;
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
static uint8_t  *USBD_GetCfgDesc (uint16_t *length)
{
  *length = USBD_ConfigDescriptorSize;
  return USBD_ConfigDescriptor;
}

/**
  * @brief  USBD_CUSTOM_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  
  /* Ensure that the FIFO is empty before a new transfer, this condition could 
  be caused by  a new transfer before the end of the previous transfer */
  hidClassData.state = CUSTOM_HID_IDLE;

  return USBD_OK;
}


uint8_t USBD_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (usbd_hid_ep_intin == epnum) {
        return USBD_HID_DataIn(pdev, epnum);
    } else {
        return USBD_CDC_DataIn(pdev, epnum);
    }
}


/**
  * @brief  USBD_CUSTOM_HID_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_DataOut (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum)
{
  
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = &hidClassData;
  
  USBD_CustomHID_fops.OutEvent(hhid->Report_buf);

  return USBD_OK;
}

uint8_t USBD_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    if (usbd_hid_ep_intin == epnum) {
        return USBD_HID_DataOut(pdev, epnum);
    } else {
        return USBD_CDC_DataOut(pdev, epnum);
    }
}
/**
  * @brief  USBD_HID_EP0_RxReady
  *         Handles control request data.
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = &hidClassData;

  if (hhid->IsReportAvailable == 1)
  {
    USBD_CustomHID_fops.OutEvent(hhid->Report_buf);
    hhid->IsReportAvailable = 0;      
  }

  return USBD_OK;
}

uint8_t USBD_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_HID_EP0_RxReady(pdev);
    USBD_CDC_EP0_RxReady(pdev);
    return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
extern const uint8_t USBD_DeviceQualifier[];
static uint8_t  *USBD_CUSTOM_HID_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_CUSTOM_HID_DeviceQualifierDesc);
  return USBD_CUSTOM_HID_DeviceQualifierDesc;
}




/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
