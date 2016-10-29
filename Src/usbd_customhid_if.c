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
#include "usbd_customhid_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t CustomHID_Init     (void);
static int8_t CustomHID_DeInit   (void);
static int8_t CustomHID_OutEvent (uint8_t* buf);
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef  AdcHandle;
uint32_t ADCConvertedValue = 0;
uint32_t ADC_Prev_ConvertedValue = 0;
extern USBD_HandleTypeDef USBD_Device;
extern const uint8_t USBD_HID_ReportDescriptor[];

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops = 
{
  USBD_HID_ReportDescriptor,
  CustomHID_Init,
  CustomHID_DeInit,
  CustomHID_OutEvent,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CustomHID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_Init(void)
{
//  GPIO_InitTypeDef   GPIO_InitStructure;
//  ADC_ChannelConfTypeDef sConfig;
  
  /* Configure the ADC peripheral */
//  AdcHandle.Instance = ADCx;
  
//  __HAL_RCC_ADC1_CLK_ENABLE();
  /*
  AdcHandle.Init.ScanConvMode = DISABLE;
  AdcHandle.Init.ContinuousConvMode = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 0;
  AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&AdcHandle);*/
   
  /* Configure ADC regular channel */  
/*  sConfig.Channel = ADCx_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);*/

  /* Start the conversion process and enable interrupt */  
//  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&ADCConvertedValue, 1);
    
  /* Configure LED1, LED2, LED3 and LED4 */
  /*BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);*/
  
  /* Enable GPIOG clock */
//  __HAL_RCC_GPIOG_CLK_ENABLE();
  
  /* Configure PG8 pin as input floating for key Button */
//  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStructure.Pull = GPIO_NOPULL;
//  GPIO_InitStructure.Pin = GPIO_PIN_8;
//  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* Enable and set EXTI2_TSC Interrupt to the lowest priority */
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  return (0);
}

/**
  * @brief  CustomHID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_DeInit(void)
{
  /*
  Add your deinitialization code here 
  */  
  return (0);
}

extern uint32_t DAP_ProcessCommand (uint8_t *request, uint8_t *response);
static uint8_t temp[64];

/**
  * @brief  CustomHID_OutEvent
  *         Manage the CUSTOM HID class Out Event    
  * @param  event_idx: LED Report Number
  * @param  state: LED states (ON/OFF)
  */
static int8_t CustomHID_OutEvent  (uint8_t* buf)
{ 
  DAP_ProcessCommand(buf, temp);
  USBD_CUSTOM_HID_SendReport(&USBD_Device, temp, sizeof(temp));

  switch(1)
  {
  case 1: /* LED1 */
//    (state == 1) ? BSP_LED_On(LED1) : BSP_LED_Off(LED1); 
    break;
    
  case 2: /* LED2 */
//    (state == 1) ? BSP_LED_On(LED2) : BSP_LED_Off(LED2); 
    break;
  case 3: /* LED3 */
//    (state == 1) ? BSP_LED_On(LED3) : BSP_LED_Off(LED3); 
    break;
  case 4: /* LED4 */
//    (state == 1) ? BSP_LED_On(LED4) : BSP_LED_Off(LED4); 
    break;
    
  default:
    /*BSP_LED_Off(LED1);
    BSP_LED_Off(LED2);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4); */
    break;
  }
  return (0);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
/*
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    SendBuffer[0] = KEY_REPORT_ID; 
    
    if(BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
    {
      SendBuffer[1] = 0x01;
    }
    else
    {
      SendBuffer[1] = 0x00;
    }
    USBD_CUSTOM_HID_SendReport(&USBD_Device, SendBuffer, 2);
  }
  */
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
