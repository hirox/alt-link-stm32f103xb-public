/**
  ******************************************************************************
  * @file    Src/stm32f1xx_hal_msp.c
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

#include "stm32f1xx_hal.h"
#include "usbd_cdc_interface.h"

static DMA_HandleTypeDef hdma_tx[3];
static DMA_HandleTypeDef hdma_rx[3];

static const uint32_t const USART_IRQn[] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};
static const uint32_t const USART_DMA_IRQn[] = {USART1_DMA_TX_IRQn, USART2_DMA_TX_IRQn, USART3_DMA_TX_IRQn};
static DMA_Channel_TypeDef* const DMA_TX_Stream[] = {USART1_TX_DMA_STREAM, USART2_TX_DMA_STREAM, USART3_TX_DMA_STREAM};
static DMA_Channel_TypeDef* const DMA_RX_Stream[] = {USART1_RX_DMA_STREAM, USART2_RX_DMA_STREAM, USART3_RX_DMA_STREAM};
static GPIO_TypeDef* const GPIO_Port[] = {GPIOA, GPIOA, GPIOB};
static const uint32_t const GPIO_TX_Pin[] = {USART1_TX_PIN, USART2_TX_PIN, USART3_TX_PIN};
static const uint32_t const GPIO_RX_Pin[] = {USART1_RX_PIN, USART2_RX_PIN, USART3_RX_PIN};

static void UART_Init_Internal(UART_HandleTypeDef *huart, uint32_t index) {
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  USARTx_GPIO_CLK_ENABLE();
  
  /* Enable USARTx clock */
  if (huart->Instance == USART1) {
      USART1_CLK_ENABLE();
  } else if (huart->Instance == USART2) {
      USART2_CLK_ENABLE();
  } else {
      USART3_CLK_ENABLE();
  }
   
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  {
    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Pin       = GPIO_TX_Pin[index];
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIO_Port[index], &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_RX_Pin[index];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    HAL_GPIO_Init(GPIO_Port[index], &GPIO_InitStruct);
  }

  /*##-3- Configure the NVIC for UART ########################################*/
  {
    HAL_NVIC_SetPriority(USART_IRQn[index], 2, index);
    HAL_NVIC_EnableIRQ(USART_IRQn[index]);

    /* Enable DMAx clock */
    DMAx_CLK_ENABLE();
  }

  /*##-4- Configure the DMA streams ##########################################*/
  /* Configure the DMA handler for Transmission process */
  {
    DMA_HandleTypeDef* tx = &hdma_tx[index];

    tx->Instance = DMA_TX_Stream[index];
    tx->Init.Direction           = DMA_MEMORY_TO_PERIPH;
    tx->Init.PeriphInc           = DMA_PINC_DISABLE;
    tx->Init.MemInc              = DMA_MINC_ENABLE;
    tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    tx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    tx->Init.Mode                = DMA_NORMAL;
    tx->Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(tx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx[index]);
  }

  {
    DMA_HandleTypeDef* rx = &hdma_rx[index];

    rx->Instance = DMA_RX_Stream[index];
    rx->Init.Direction           = DMA_PERIPH_TO_MEMORY;
    rx->Init.PeriphInc           = DMA_PINC_DISABLE;
    rx->Init.MemInc              = DMA_MINC_ENABLE;
    rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rx->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    rx->Init.Mode                = DMA_CIRCULAR;
    rx->Init.Priority            = DMA_PRIORITY_LOW;

    HAL_DMA_Init(rx);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(huart, hdmarx, hdma_rx[index]);
  }
  
  /*##-5- Configure the NVIC for DMA #########################################*/   
  /* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
  {
    HAL_NVIC_SetPriority(USART_DMA_IRQn[index], 6, index);
    HAL_NVIC_EnableIRQ(USART_DMA_IRQn[index]);
  }
}

static void UART_DeInit_Internal(UART_HandleTypeDef *huart, uint32_t index) {
    /* Disable the NVIC for UART/DMA */
    HAL_NVIC_DisableIRQ(USART_IRQn[index]);
    HAL_NVIC_DisableIRQ(USART_DMA_IRQn[index]);

    /* Reset peripherals */
    if (huart->Instance == USART1) {
        USART1_FORCE_RESET();
        USART1_RELEASE_RESET();
    } else if (huart->Instance == USART2) {
        USART2_FORCE_RESET();
        USART2_RELEASE_RESET();
    } else {
        USART3_FORCE_RESET();
        USART3_RELEASE_RESET();
    }

    /* Disable peripherals and GPIO Clocks */
    HAL_GPIO_DeInit(GPIO_Port[index], GPIO_TX_Pin[index]);
    HAL_GPIO_DeInit(GPIO_Port[index], GPIO_RX_Pin[index]);

    hdma_tx[index].Instance = DMA_TX_Stream[index];
    hdma_rx[index].Instance = DMA_RX_Stream[index];
    HAL_DMA_DeInit(&hdma_tx[index]);
    HAL_DMA_DeInit(&hdma_rx[index]);
}

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
    if (huart->Instance == USART1) {
        UART_Init_Internal(huart, 0);
    } else if (huart->Instance == USART2) {
        UART_Init_Internal(huart, 1);
    } else {
        UART_Init_Internal(huart, 2);
    }
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
    if (huart->Instance == USART1) {
        UART_DeInit_Internal(huart, 0);
    } else if (huart->Instance == USART2) {
        UART_DeInit_Internal(huart, 1);
    } else {
        UART_DeInit_Internal(huart, 2);
    }
}

/**
  * @brief I2C MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for I2C ########################################*/
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 7, 1);
  HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
  HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 7, 2);
  HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
}

/**
  * @brief I2C MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
