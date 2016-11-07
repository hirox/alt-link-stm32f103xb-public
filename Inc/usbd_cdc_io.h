/**
  ******************************************************************************
  * @file    usbd_cdc_io.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_cdc.c file.
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
 
/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

typedef struct {
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

typedef struct _USBD_CDC_Itf {
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint32_t, uint8_t, uint8_t * , uint16_t);
  int8_t (* Receive)       (uint32_t, uint8_t *, uint32_t);  
} USBD_CDC_ItfTypeDef;


typedef struct {
  uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t TxLength;
  
  __IO uint32_t TxState;
  __IO uint32_t RxState;
} USBD_CDC_HandleTypeDef; 



/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_CDC_SetTxBuffer        (uint32_t index, uint8_t  *pbuff, uint16_t length);
uint8_t  USBD_CDC_SetRxBuffer        (uint32_t index, uint8_t  *pbuff);
uint8_t  USBD_CDC_ReceivePacket      (uint32_t index, USBD_HandleTypeDef *pdev);
uint8_t  USBD_CDC_TransmitPacket     (uint32_t index, USBD_HandleTypeDef *pdev);

uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);

void CDC_Run_In_Thread_Mode();

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
