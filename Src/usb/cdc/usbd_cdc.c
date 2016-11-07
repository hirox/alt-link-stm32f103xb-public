/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
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
#include "usbd_cdc.h"
#include "usbd_cdc_io.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

USBD_CDC_HandleTypeDef cdcClassData[2];
extern USBD_CDC_ItfTypeDef USBD_CDC_fops;



void USBD_CDC_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
    /* Open EP IN */
    USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_IN_EP2, USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_OUT_EP2, USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE);

    /* Open Command IN EP */
    USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
    USBD_LL_OpenEP(pdev, CDC_CMD_EP2, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  
    /* Init  physical Interface components */
    USBD_CDC_fops.Init();
    
    /* Init Xfer states */
    USBD_CDC_HandleTypeDef* hcdc = &cdcClassData[0];
    hcdc->TxState =0;
    hcdc->RxState =0;
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, hcdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);

    hcdc = &cdcClassData[1];
    hcdc->TxState =0;
    hcdc->RxState =0;
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP2, hcdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);
}

extern const uint8_t usbd_cdc_acm_cif_num;
extern const uint8_t usbd_cdc_acm_dif_num;
static volatile uint32_t lastIndex = 0;

uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_CDC_HandleTypeDef   *hcdc;

  if (LOBYTE(req->wIndex) == usbd_cdc_acm_cif_num || LOBYTE(req->wIndex) == usbd_cdc_acm_dif_num) {
    hcdc = &cdcClassData[0];
    lastIndex = 0;
  } else {
    hcdc = &cdcClassData[1];
    lastIndex = 1;
  }
  static uint8_t ifalt = 0;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        USBD_CDC_fops.Control(lastIndex, req->bRequest, (uint8_t *)hcdc->data, req->wLength);
        USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
    }
    else
    {
        USBD_CDC_fops.Control(lastIndex, req->bRequest, (uint8_t*)req, 0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}


/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_CDC_HandleTypeDef *hcdc;
    
    if (epnum == (CDC_IN_EP & 0x7F)) {
        hcdc = &cdcClassData[0];
    } else {
        hcdc = &cdcClassData[1];
    }

    hcdc->TxState = 0;

    return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
    USBD_CDC_HandleTypeDef   *hcdc;

    if (epnum == (CDC_OUT_EP & 0x7F)) {
        hcdc = &cdcClassData[0];

        /* Get the received data length */
        uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);

        /* USB data will be immediately processed, this allow next USB traffic being 
        NAKed till the end of the application Xfer */
        USBD_CDC_fops.Receive(0, hcdc->RxBuffer, len);
    } else {
        hcdc = &cdcClassData[1];
        uint32_t len = USBD_LL_GetRxDataSize(pdev, epnum);
        USBD_CDC_fops.Receive(1, hcdc->RxBuffer, len);
    }

    return USBD_OK;
}



/**
  * @brief  USBD_CDC_EP0_RxReady
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{ 
    USBD_CDC_HandleTypeDef   *hcdc = &cdcClassData[lastIndex];

    if((hcdc->CmdOpCode != 0xFF))
    {
        USBD_CDC_fops.Control(lastIndex, hcdc->CmdOpCode, (uint8_t *)hcdc->data, hcdc->CmdLength);
        hcdc->CmdOpCode = 0xFF; 
    }
  return USBD_OK;
}


/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = sizeof(USBD_CDC_DeviceQualifierDesc);
  return USBD_CDC_DeviceQualifierDesc;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer(uint32_t index, uint8_t *pbuff, uint16_t length)
{
  USBD_CDC_HandleTypeDef   *hcdc = &cdcClassData[index];
  
  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;
  
  return USBD_OK;
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer(uint32_t index, uint8_t  *pbuff)
{
  USBD_CDC_HandleTypeDef   *hcdc = &cdcClassData[index];
  hcdc->RxBuffer = pbuff;
  
  return USBD_OK;
}

extern PCD_HandleTypeDef hpcd;

/**
  * @brief  USBD_CDC_TransmitPacket
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(uint32_t index, USBD_HandleTypeDef *pdev)
{      
    USBD_CDC_HandleTypeDef   *hcdc = &cdcClassData[index];
    uint32_t epnum = (index == 0) ? CDC_IN_EP : CDC_IN_EP2;

    if(hcdc->TxState == 0 || ((PCD_GET_ENDPOINT(hpcd.Instance, (epnum & 0x7F))) & USB_EP_CTR_TX) == 0) {
      /* Tx Transfer in progress */
      hcdc->TxState = 1;
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       epnum,
                       hcdc->TxBuffer,
                       hcdc->TxLength);
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(uint32_t index, USBD_HandleTypeDef *pdev)
{      
    USBD_CDC_HandleTypeDef   *hcdc = &cdcClassData[index];
    uint32_t epnum = (index == 0) ? CDC_OUT_EP : CDC_OUT_EP2;

    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(pdev, epnum, hcdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);
    return USBD_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
