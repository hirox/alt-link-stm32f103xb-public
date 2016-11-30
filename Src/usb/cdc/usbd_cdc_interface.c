/**
  ******************************************************************************
  * @file    USB_Device/CustomHID_Standalone/Src/usbd_customhid_if.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define APP_RX_DATA_SIZE  192
#define APP_TX_DATA_SIZE  512

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };
USBD_CDC_LineCodingTypeDef LineCoding2 =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

struct {
    struct {
        uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
        uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
        uint32_t UserTxBufPtrIn;/* Increment this pointer or roll it back to
                                       start address when data are received over USART */
        uint32_t UserTxBufPtrOut; /* Increment this pointer or roll it back to
                                         start address when data are sent over USB */
        uint32_t UserRxBufPtr;
        uint32_t Run_Receive_From_HOST;
        uint32_t Run_PortConfig;
        uint8_t *TransmitBuf;
        uint32_t TransmitLen;
        uint32_t DMA_Transferring;
        uint32_t Run_DMA_Transfer_Finished;
    } d[2];
    uint32_t Run_TIM;
} CDC = {0};

/* UART handler declaration */
UART_HandleTypeDef UartHandle[2];
/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;
/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint32_t index, uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint32_t index, uint8_t*, uint32_t);

static void Error_Handler(void);
static void ComPort_Config(uint32_t index);
static void TIM_Config(void);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

static void Clear_UART_Status(uint32_t index) {
    CDC.d[index].DMA_Transferring = 0;
    CDC.d[index].Run_DMA_Transfer_Finished = 0;
}

/**
  * @brief  CDC_Itf_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
    for (uint32_t i = 0; i < 2; i++) {
        /*##-1- Configure the UART peripheral ######################################*/
        /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
        /* USART configured as follow:
            - Word Length = 8 Bits
            - Stop Bit    = One Stop bit
            - Parity      = No parity
            - BaudRate    = 115200 baud
            - Hardware flow control disabled (RTS and CTS signals) */
        UartHandle[i].Instance        = i == 0 ? USARTx : USARTx2;
        UartHandle[i].Init.BaudRate   = 115200;
        UartHandle[i].Init.WordLength = UART_WORDLENGTH_8B;
        UartHandle[i].Init.StopBits   = UART_STOPBITS_1;
        UartHandle[i].Init.Parity     = UART_PARITY_NONE;
        UartHandle[i].Init.HwFlowCtl  = UART_HWCONTROL_NONE;
        UartHandle[i].Init.Mode       = UART_MODE_TX_RX;

        if(HAL_UART_Init(&UartHandle[i]) != HAL_OK) {
            /* Initialization Error */
            Error_Handler();
        }

        /*##-2- Put UART peripheral in IT reception process ########################*/
        /* Any data received will be stored in "UserTxBuffer" buffer  */
        if(HAL_UART_Receive_IT(&UartHandle[i], (uint8_t *)CDC.d[i].UserTxBuffer) != HAL_OK) {
            /* Transfer error in reception process */
            Error_Handler();
        }
    }

    /*##-3- Configure the TIM Base generation  #################################*/
    TIM_Config();
  
    /*##-4- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
        /* Starting Error */
        Error_Handler();
    }
  
    /*##-5- Set Application Buffers ############################################*/
    for (uint32_t i = 0; i < 2; i++) {
        USBD_CDC_SetTxBuffer(i, CDC.d[i].UserTxBuffer, 0);
        USBD_CDC_SetRxBuffer(i, CDC.d[i].UserRxBuffer);
    }

    return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
    /* DeInitialize the UART peripheral */
    for (uint32_t i = 0; i < 2; i++) {
        UartHandle[i].Instance = i == 0 ? USARTx : USARTx2;
        if(HAL_UART_DeInit(&UartHandle[i]) != HAL_OK) {
            /* Initialization Error */
            Error_Handler();
        }
        Clear_UART_Status(i);
    }

    return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control(uint32_t index, uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
    // inside IRQ
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    {
        uint32_t i = index == 0 ? 0 : 1;
        USBD_CDC_LineCodingTypeDef lc;
        lc.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                                (pbuf[2] << 16) | (pbuf[3] << 24));
        lc.format     = pbuf[4];
        lc.paritytype = pbuf[5];
        lc.datatype   = pbuf[6];
        
        
        if (i == 0) { LineCoding = lc; }
        else { LineCoding2 = lc; }
        
        CDC.d[i].Run_PortConfig = 1;
    }
    break;

  case CDC_GET_LINE_CODING:
    {
        USBD_CDC_LineCodingTypeDef lc;
        if (index == 0) { lc = LineCoding; }
        else { lc = LineCoding2; }

        pbuf[0] = (uint8_t)(lc.bitrate);
        pbuf[1] = (uint8_t)(lc.bitrate >> 8);
        pbuf[2] = (uint8_t)(lc.bitrate >> 16);
        pbuf[3] = (uint8_t)(lc.bitrate >> 24);
        pbuf[4] = lc.format;
        pbuf[5] = lc.paritytype;
        pbuf[6] = lc.datatype;
    }
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }
  
  return (USBD_OK);
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
    CDC.Run_TIM = 1;
}

/**
  * @brief  UART Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
uint8_t* HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // inside IRQ
    uint32_t i = huart->Instance == USARTx ? 0 : 1;

    /* Increment Index for buffer writing */
    CDC.d[i].UserTxBufPtrIn++;

    /* To avoid buffer overflow */
    CDC.d[i].UserTxBufPtrIn &= 0x1FF;

    /* Start another reception: provide the buffer pointer with offset and the buffer size */
    return (uint8_t *)(CDC.d[i].UserTxBuffer + CDC.d[i].UserTxBufPtrIn);
}

/**
  * @brief  CDC_Itf_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  * @param  Buf: Buffer of data to be transmitted
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint32_t index, uint8_t* buf, uint32_t len)
{
    // inside IRQ
    uint32_t i = index == 0 ? 0 : 1;
    CDC.d[i].TransmitBuf = buf;
    CDC.d[i].TransmitLen = len;

    CDC.d[i].UserRxBufPtr += len;
    if (CDC.d[i].UserRxBufPtr + 64 > APP_RX_DATA_SIZE) {
        CDC.d[i].UserRxBufPtr = 0;
    }

    CDC.d[i].Run_Receive_From_HOST = 1;
    return (USBD_OK);
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // inside IRQ
    uint32_t i = huart->Instance == USARTx ? 0 : 1;
    /* Initiate next USB packet transfer once UART completes transfer (transmitting data over Tx line) */
    CDC.d[i].Run_DMA_Transfer_Finished = 1;
}

// noinline is necessary to output correct code
__NOINLINE void CDC_Run_In_Thread_Mode()
{
#if 1
    for (uint32_t i = 0; i < 2; i++) {
        if (CDC.d[i].Run_PortConfig) {
            // [TODO] Run_PortConfigを毎回動かすと HAL_UART_STATE_BUSY_TX で HAL_UART_Transmit_DMA がエラーになる謎がある
            CDC.d[i].Run_PortConfig = 0;
            ComPort_Config(i);
            HAL_UART_Receive_IT(&UartHandle[i], (uint8_t *)(CDC.d[i].UserTxBuffer + CDC.d[i].UserTxBufPtrIn));
        }

        if (CDC.d[i].Run_DMA_Transfer_Finished) {
            CDC.d[i].Run_DMA_Transfer_Finished = 0;
            CDC.d[i].DMA_Transferring = 0;
        }
        if (CDC.d[i].Run_Receive_From_HOST && CDC.d[i].DMA_Transferring == 0) {
            CDC.d[i].Run_Receive_From_HOST = 0;
            HAL_UART_Transmit_DMA(&UartHandle[i], CDC.d[i].TransmitBuf, CDC.d[i].TransmitLen);
            CDC.d[i].DMA_Transferring = 1;

            USBD_CDC_SetRxBuffer(i, &CDC.d[i].UserRxBuffer[CDC.d[i].UserRxBufPtr]);

            // USBのIRQの中でUSBが叩かれることがあるので競合してしまう可能性がある
            // IRQを止めることで競合の可能性を排除する
            HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
            USBD_CDC_ReceivePacket(i, &USBD_Device);
            HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
        }
    }
#endif
#if 1
    if (CDC.Run_TIM) {
        CDC.Run_TIM = 0;

        for (uint32_t i = 0; i < 2; i++) {
            if(CDC.d[i].UserTxBufPtrOut != CDC.d[i].UserTxBufPtrIn) {
                uint32_t buffsize;
                uint8_t ret;

                if(CDC.d[i].UserTxBufPtrOut > CDC.d[i].UserTxBufPtrIn) { /* Rollback */
                    buffsize = APP_TX_DATA_SIZE - CDC.d[i].UserTxBufPtrOut;
                } else {
                    buffsize = CDC.d[i].UserTxBufPtrIn - CDC.d[i].UserTxBufPtrOut;
                }

                USBD_CDC_SetTxBuffer(i, &CDC.d[i].UserTxBuffer[CDC.d[i].UserTxBufPtrOut], buffsize);

                HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
                ret = USBD_CDC_TransmitPacket(i, &USBD_Device);
                HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
                if(ret == USBD_OK) {
                    CDC.d[i].UserTxBufPtrOut += buffsize;
                    if (CDC.d[i].UserTxBufPtrOut == APP_TX_DATA_SIZE) {
                        CDC.d[i].UserTxBufPtrOut = 0;
                    }
                }
            }
        }
    }
#endif
}

/**
  * @brief  ComPort_Config
  *         Configure the COM Port with the parameters received from host.
  * @param  None.
  * @retval None.
  * @note   When a configuration is not supported, a default value is used.
  */
static void ComPort_Config(uint32_t index)
{
    UART_HandleTypeDef* handle = &UartHandle[index];
    USBD_CDC_LineCodingTypeDef* lc = index == 0 ? &LineCoding : &LineCoding2;

    if(HAL_UART_DeInit(handle) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }
    Clear_UART_Status(index);
  
    /* set the Stop bit */
    switch (lc->format) {
    case 0:
        handle->Init.StopBits = UART_STOPBITS_1;
        break;
    case 2:
        handle->Init.StopBits = UART_STOPBITS_2;
        break;
    default :
        handle->Init.StopBits = UART_STOPBITS_1;
        break;
    }
  
    /* set the parity bit*/
    switch (lc->paritytype) {
    case 0:
        handle->Init.Parity = UART_PARITY_NONE;
        break;
    case 1:
        handle->Init.Parity = UART_PARITY_ODD;
        break;
    case 2:
        handle->Init.Parity = UART_PARITY_EVEN;
        break;
    default :
        handle->Init.Parity = UART_PARITY_NONE;
        break;
    }
  
    /*set the data type : only 8bits and 9bits is supported */
    switch (lc->datatype) {
    case 0x07:
        /* With this configuration a parity (Even or Odd) must be set */
        handle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case 0x08:
        if(handle->Init.Parity == UART_PARITY_NONE)
            handle->Init.WordLength = UART_WORDLENGTH_8B;
        else
            handle->Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default :
        handle->Init.WordLength = UART_WORDLENGTH_8B;
        break;
    }
  
    handle->Init.BaudRate = lc->bitrate;
    handle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    handle->Init.Mode       = UART_MODE_TX_RX;

    if(HAL_UART_Init(handle) != HAL_OK) {
        /* Initialization Error */
        Error_Handler();
    }
}

/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None.
  */
static void TIM_Config(void)
{  
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
  
  /* Initialize TIM3 peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*1000) - 1;
  TimHandle.Init.Prescaler = 84-1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    (void)UartHandle;
  /* Transfer error occured in reception and/or transmission process */
  Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Add your own code here */
    while(1) {};
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
