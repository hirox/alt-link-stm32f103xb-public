/**
 * @file    IO_Config.h
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

//#include "stm32f10x.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
//#include "compiler.h"
//#include "daplink.h"

//COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_STM32F103XB);

#if 0
//USB control pin
#define USB_CONNECT_PORT_ENABLE()    (RCC->APB2ENR |= RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT_DISABLE()   (RCC->APB2ENR &= ~RCC_APB2Periph_GPIOA)
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_Pin_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BSRR = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BRR  = USB_CONNECT_PIN)

//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            GPIOB
#define POWER_EN_PIN                 GPIO_PIN_15
#define POWER_EN_Bit                 15
#endif

// SWD
#define SWCLK_TCK_PIN_PORT           GPIOB
#define SWCLK_TCK_PIN                GPIO_PIN_12
#define SWCLK_TCK_PIN_Bit            12

#define SWDIO_OUT_PIN_PORT           GPIOB
#define SWDIO_OUT_PIN                GPIO_PIN_13
#define SWDIO_OUT_PIN_Bit            13

#define SWDIO_IN_PIN_PORT            GPIOB
#define SWDIO_IN_PIN                 GPIO_PIN_13
#define SWDIO_IN_PIN_Bit             13

// JTAG
#define JTAG_TDI_PIN_PORT            GPIOB
#define JTAG_TDI_PIN                 GPIO_PIN_14
#define JTAG_TDI_PIN_Bit             14

#define JTAG_TDO_PIN_PORT            GPIOB
#define JTAG_TDO_PIN                 GPIO_PIN_15
#define JTAG_TDO_PIN_Bit             15

#define JTAG_nTRST_PIN_PORT          GPIOB
#define JTAG_nTRST_PIN               GPIO_PIN_8
#define JTAG_nTRST_PIN_Bit           8

//Press and power, enter bootloader 7
//When daplnk_if, reset target board
#define nRESET_PIN_PORT              GPIOB
#define nRESET_PIN                   GPIO_PIN_9
#define nRESET_PIN_Bit               9

//LEDs
// Connected LED
#define CONNECTED_LED_PORT           GPIOC
#define CONNECTED_LED_PIN            GPIO_PIN_13
#define CONNECTED_LED_PIN_Bit        13

#if 0
//USB status LED
#define RUNNING_LED_PORT             GPIOB
#define RUNNING_LED_PIN              GPIO_PIN_5
#define RUNNING_LED_Bit              5

#define PIN_HID_LED_PORT             GPIOB
#define PIN_HID_LED                  GPIO_PIN_6
#define PIN_HID_LED_Bit              6

#define PIN_CDC_LED_PORT             GPIOB
#define PIN_CDC_LED                  GPIO_PIN_6
#define PIN_CDC_LED_Bit              6

#define PIN_MSC_LED_PORT             GPIOB
#define PIN_MSC_LED                  GPIO_PIN_6
#define PIN_MSC_LED_Bit              6
#endif

#endif
