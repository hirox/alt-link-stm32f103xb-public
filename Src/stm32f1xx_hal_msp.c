/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Src/stm32f1xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usbd_cdc_interface.h"
//#include "stm3210C_eval.h"
//#include "main.h"

/** @addtogroup USBD_USER
* @{
*/

/** @defgroup USBD_USR_MAIN
  * @brief This file is the CDC application main file
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_tx2;

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  
  /* Enable USARTx clock */
  if (huart->Instance == USARTx) {
      USARTx_CLK_ENABLE();
  } else {
      USARTx_CLK_ENABLE2();
  }
   
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = USARTx_TX_PIN2;
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
  
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USARTx_RX_PIN2;
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
  
  /*##-3- Configure the NVIC for UART ########################################*/
  //[J] UARTの割り込み優先度が低いとOverrunしてしまうので高い優先度にする必要がある
  HAL_NVIC_SetPriority(USARTx_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
  HAL_NVIC_SetPriority(USARTx_IRQn2, 2, 0);
  HAL_NVIC_EnableIRQ(USARTx_IRQn2);

  /* Enable DMAx clock */
  DMAx_CLK_ENABLE();
  
  /*##-4- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  DMA_HandleTypeDef* tx = NULL;
  if (huart->Instance == USARTx) {
    tx = &hdma_tx;
    tx->Instance                 = USARTx_TX_DMA_STREAM;
  } else {
    tx = &hdma_tx2;
    tx->Instance                 = USARTx_TX_DMA_STREAM2;
  }
  tx->Init.Direction           = DMA_MEMORY_TO_PERIPH;
  tx->Init.PeriphInc           = DMA_PINC_DISABLE;
  tx->Init.MemInc              = DMA_MINC_ENABLE;
  tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  tx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  tx->Init.Mode                = DMA_NORMAL;
  tx->Init.Priority            = DMA_PRIORITY_LOW;
  
  HAL_DMA_Init(tx);
  
  /* Associate the initialized DMA handle to the UART handle */
  if (huart->Instance == USARTx) {
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);
  } else {
    __HAL_LINKDMA(huart, hdmatx, hdma_tx2);
  }
  
  /*##-5- Configure the NVIC for DMA #########################################*/   
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
  if (huart->Instance == USARTx) {
    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);
  } else {
    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn2, 6, 0);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn2);
  }
  
  /*##-6- Enable TIM peripherals Clock #######################################*/
  TIMx_CLK_ENABLE();
  
  /*##-7- Configure the NVIC for TIMx ########################################*/
  /* Set Interrupt Group Priority */ 
  HAL_NVIC_SetPriority(TIMx_IRQn, 6, 0);
  
  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USARTx) {
        /*##-0- Disable the NVIC for UART/DMA ######################################*/
        HAL_NVIC_DisableIRQ(USARTx_IRQn);
        HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn);

        /*##-1- Reset peripherals ##################################################*/
        USARTx_FORCE_RESET();
        USARTx_RELEASE_RESET();

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

        hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
        HAL_DMA_DeInit(&hdma_tx);
    } else {
        HAL_NVIC_DisableIRQ(USARTx_IRQn2);
        HAL_NVIC_DisableIRQ(USARTx_DMA_TX_IRQn2);

        USARTx_FORCE_RESET2();
        USARTx_RELEASE_RESET2();

        HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN2);
        HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN2);

        hdma_tx2.Instance                 = USARTx_TX_DMA_STREAM2;
        HAL_DMA_DeInit(&hdma_tx2);
    }
  
    /*##-4- Reset TIM peripheral ###############################################*/
    TIMx_FORCE_RESET();
    TIMx_RELEASE_RESET();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
