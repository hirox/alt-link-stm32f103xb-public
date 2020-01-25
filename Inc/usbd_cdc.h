/**
  ******************************************************************************
  * @file    usbd_cdc.h
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

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */ 
#define CDC_ENDPOINT
#define CDC_ENDPOINT2
#define CDC_ENDPOINT3

#define CDC_IN_EP                                   0x82  /* EP2 for data IN */
#define CDC_OUT_EP                                  0x02  /* EP2 for data OUT */
#define CDC_IN_EP2                                  0x83  /* EP3 for data IN */
#define CDC_OUT_EP2                                 0x03  /* EP3 for data OUT */
#define CDC_IN_EP3                                  0x84  /* EP4 for data IN */
#define CDC_OUT_EP3                                 0x04  /* EP4 for data OUT */
#define CDC_CMD_EP                                  0x85  /* EP5 for CDC commands (INTR IN) */
#define CDC_CMD_EP2                                 0x86  /* EP6 for CDC commands (INTR IN) */
#define CDC_CMD_EP3                                 0x87  /* EP7 for CDC commands (INTR IN) */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64   /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8    /* Control Endpoint Packet size */

#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_DATA_CAN_FS_PACKET_SIZE                 32

/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23

#define APP_RX_DATA_SIZE  512
#define APP_TX_DATA_SIZE  2048

#define NUM_CDC (3)
#define SLCAN_INDEX (2)

typedef struct {
    struct {
        /* Received Data over USB are stored in this buffer (Host -> buffer -> UART) */
        uint8_t UsbOutBuffer[APP_RX_DATA_SIZE + CDC_DATA_FS_OUT_PACKET_SIZE];
        /* Received Data over UART (CDC interface) are stored in this buffer (UART -> buffer -> Host) */
        uint8_t UserTxBuffer[APP_TX_DATA_SIZE];
        uint32_t UserTxBufPtrOut; /* Increment this pointer or roll it back to
                                         start address when data are sent over USB */
        uint32_t UsbOutOverWriteSize; // [NOTICE] Will be updeted in USB OUT callback
        uint32_t UsbOutBufWritePos;   // [NOTICE] Will be updeted in USB OUT callback
        uint32_t UsbOutBufReadPos;
        uint32_t UsbOutBufReadPosInOverwrite;
        uint32_t Run_Receive_From_HOST;
        uint32_t Run_PortConfig;
        uint32_t Run_DMA_Transfer;
        uint32_t Run_TIM;
    } d[NUM_CDC];
} CDC_WorkMemory;

#ifdef __cplusplus
}
#endif
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
