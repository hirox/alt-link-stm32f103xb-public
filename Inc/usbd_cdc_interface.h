/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/usbd_cdc_interface.h
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   Header for usbd_cdc_interface.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright � 2016 STMicroelectronics International N.V. 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usbd_cdc.h"
#include "usbd_cdc_io.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USART1_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_GPIO_CLK_ENABLE()  do { \
                                    __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                    __HAL_RCC_GPIOB_CLK_ENABLE(); \
                                  } while(0)

#define USART1_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()
#define USART2_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USART2_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()
#define USART3_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
#define USART3_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USART1_TX_PIN                    GPIO_PIN_9
#define USART1_RX_PIN                    GPIO_PIN_10

/* Definition for USARTx's DMA: used for transmitting data over Tx pin */
#define USART1_TX_DMA_STREAM             DMA1_Channel4
#define USART1_RX_DMA_STREAM             DMA1_Channel5
#define USART1_DMA_TX_IRQHandler         DMA1_Channel4_IRQHandler
#define USART1_DMA_TX_IRQn               DMA1_Channel4_IRQn

/* Definition for USARTx clock resources */
#define USART2_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();

/* Definition for USARTx Pins */
#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_RX_PIN                    GPIO_PIN_3

/* Definition for USARTx's DMA: used for transmitting data over Tx pin */
#define USART2_TX_DMA_STREAM             DMA1_Channel7
#define USART2_RX_DMA_STREAM             DMA1_Channel6
#define USART2_DMA_TX_IRQHandler         DMA1_Channel7_IRQHandler
#define USART2_DMA_TX_IRQn               DMA1_Channel7_IRQn

/* Definition for USARTx clock resources */
#define USART3_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE();

/* Definition for USARTx Pins */
#define USART3_TX_PIN                    GPIO_PIN_10
#define USART3_RX_PIN                    GPIO_PIN_11

/* Definition for USARTx's DMA: used for transmitting data over Tx pin */
#define USART3_TX_DMA_STREAM             DMA1_Channel2
#define USART3_RX_DMA_STREAM             DMA1_Channel3
#define USART3_DMA_TX_IRQHandler         DMA1_Channel2_IRQHandler
#define USART3_DMA_TX_IRQn               DMA1_Channel2_IRQn

/* Definition for TIMx clock resources */
#define TIM2_CLK_ENABLE                  __HAL_RCC_TIM2_CLK_ENABLE
#define TIM3_CLK_ENABLE                  __HAL_RCC_TIM3_CLK_ENABLE
#define TIM4_CLK_ENABLE                  __HAL_RCC_TIM4_CLK_ENABLE

/* Periodically, the state of the buffer "UserTxBuffer" is checked.
   The period depends on CDC_POLLING_INTERVAL */
#define CDC_POLLING_INTERVAL             4 /* in ms. The max is 65 and the min is 1 */

/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __USBD_CDC_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
