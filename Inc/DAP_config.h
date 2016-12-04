/**
 * @file    DAP_config.h
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

#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

//#include "stm32f10x.h"
#include "stm32f103xb.h"
#include "stdint.h"
//#include "RTL.h"
#include "IO_Config.h"
//#include "uart.h"
//#include "debug_cm.h"
//#include "swd_host.h"

#define __forceinline __inline
#define RCC_APB2Periph_GPIOA             ((uint32_t)0x00000004)
#define RCC_APB2Periph_GPIOB             ((uint32_t)0x00000008)
#define RCC_APB2Periph_GPIOC             ((uint32_t)0x00000010)

//**************************************************************************************************
/**
\defgroup DAP_Config_Debug_gr CMSIS-DAP Debug Unit Information
\ingroup DAP_ConfigIO_gr
@{
Provides definitions about:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit communication packet size.
 - Debug Access Port communication mode (JTAG or SWD).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

/// Processor Clock of the Cortex-M MCU used in the Debug Unit.
/// This value is used to calculate the SWD/JTAG clock speed.
#define CPU_CLOCK               SystemCoreClock        ///< Specifies the CPU Clock in Hz

/// Number of processor cycles for I/O Port write operations.
/// This value is used to calculate the SWD/JTAG clock speed that is generated with I/O
/// Port write operations in the Debug Unit by a Cortex-M MCU. Most Cortex-M processors
/// requrie 2 processor cycles for a I/O Port Write operation.  If the Debug Unit uses
/// a Cortex-M0+ processor with high-speed peripheral I/O only 1 processor cycle might be
/// requrired.
#define IO_PORT_WRITE_CYCLES    2               ///< I/O Cycles: 2=default, 1=Cortex-M0+ fast I/0

/// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define DAP_SWD                 1               ///< SWD Mode:  1 = available, 0 = not available

/// Indicate that JTAG communication mode is available at the Debug Port.
/// This information is returned by the command \ref DAP_Info as part of <b>Capabilities</b>.
#define DAP_JTAG                1               ///< JTAG Mode: 1 = available, 0 = not available.

/// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
/// This setting impacts the RAM requirements of the Debug Unit. Valid range is 1 .. 255.
#define DAP_JTAG_DEV_CNT        255               ///< Maximum number of JTAG devices on scan chain

/// Default communication mode on the Debug Access Port.
/// Used for the command \ref DAP_Connect when Port Default mode is selected.
#define DAP_DEFAULT_PORT        1               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.

/// Default communication speed on the Debug Access Port for SWD and JTAG mode.
/// Used to initialize the default SWD/JTAG clock frequency.
/// The command \ref DAP_SWJ_Clock can be used to overwrite this default setting.
#define DAP_DEFAULT_SWJ_CLOCK   5000000         ///< Default SWD/JTAG clock frequency in Hz.

/// Maximum Package Size for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. Change setting to 1024 for High-Speed USB.
#define DAP_PACKET_SIZE        64              ///< USB: 64 = Full-Speed, 1024 = High-Speed.

/// Maximum Package Buffers for Command and Response data.
/// This configuration settings is used to optimized the communication performance with the
/// debugger and depends on the USB peripheral. For devices with limited RAM or USB buffer the
/// setting can be reduced (valid range is 1 .. 255). Change setting to 4 for High-Speed USB.
#define DAP_PACKET_COUNT       4              ///< Buffers: 64 = Full-Speed, 4 = High-Speed.


/// Debug Unit is connected to fixed Target Device.
/// The Debug Unit may be part of an evaluation board and always connected to a fixed
/// known device.  In this case a Device Vendor and Device Name string is stored which
/// may be used by the debugger or IDE to configure device parameters.
#define TARGET_DEVICE_FIXED     0               ///< Target Device: 1 = known, 0 = unknown;

#if TARGET_DEVICE_FIXED
#define TARGET_DEVICE_VENDOR    ""              ///< String indicating the Silicon Vendor
#define TARGET_DEVICE_NAME      ""              ///< String indicating the Target Device
#endif


#define DAP_VENDOR "Alt-Link";
#define DAP_PRODUCT "Alt-Link CMSIS-DAP";
///@}


static __inline void pin_out_init(GPIO_TypeDef* GPIOx, uint8_t pin_bit)
{
    if(pin_bit >= 8)
    {
        GPIOx->CRH = (GPIOx->CRH & ~(0x0000000F << ((pin_bit-8) << 2))) | ( ((uint32_t)(0x00|0x03) & 0x0F) << ((pin_bit-8) << 2) );
    }
    else
    {
        GPIOx->CRL = (GPIOx->CRL & ~(0x0000000F << ((pin_bit) << 2))) | ( ((uint32_t)(0x00|0x03) & 0x0F) << ((pin_bit) << 2) );
    }
}

static __inline void pin_in_init(GPIO_TypeDef* GPIOx, uint8_t pin_bit, uint8_t mode)
{
    uint8_t config;
    if(mode == 1)
        config = 0x08; //Up
    else if(mode == 2)
        config = 0x08; //down
    else
        config = 0x00; //GPIO_Mode_AIN

    if(pin_bit >= 8)
    {
        GPIOx->CRH = (GPIOx->CRH & ~(0x0000000F << ((pin_bit-8) << 2))) | ( ((uint32_t)(config) & 0x0F) << ((pin_bit-8) << 2) );
        if(mode == 1)
            GPIOx->BSRR = (((uint32_t)0x01) << pin_bit);
        else if(mode == 2)
            GPIOx->BRR = (((uint32_t)0x01) << pin_bit);
    }
    else
    {
        GPIOx->CRL = (GPIOx->CRL & ~(0x0000000F << ((pin_bit) << 2))) | ( ((uint32_t)(config) & 0x0F) << ((pin_bit) << 2) );
        if(mode == 1)
            GPIOx->BSRR = (((uint32_t)0x01) << pin_bit);
        else if(mode == 2)
            GPIOx->BRR = (((uint32_t)0x01) << pin_bit);
    }
}
//**************************************************************************************************
/**
\defgroup DAP_Config_PortIO_gr CMSIS-DAP Hardware I/O Pin Access
\ingroup DAP_ConfigIO_gr
@{

Standard I/O Pins of the CMSIS-DAP Hardware Debug Port support standard JTAG mode
and Serial Wire Debug (SWD) mode. In SWD mode only 2 pins are required to implement the debug
interface of a device. The following I/O Pins are provided:

JTAG I/O Pin                 | SWD I/O Pin          | CMSIS-DAP Hardware pin mode
---------------------------- | -------------------- | ---------------------------------------------
TCK: Test Clock              | SWCLK: Clock         | Output Push/Pull
TMS: Test Mode Select        | SWDIO: Data I/O      | Output Push/Pull; Input (for receiving data)
TDI: Test Data Input         |                      | Output Push/Pull
TDO: Test Data Output        |                      | Input
nTRST: Test Reset (optional) |                      | Output Open Drain with pull-up resistor
nRESET: Device Reset         | nRESET: Device Reset | Output Open Drain with pull-up resistor


DAP Hardware I/O Pin Access Functions
-------------------------------------
The various I/O Pins are accessed by functions that implement the Read, Write, Set, or Clear to
these I/O Pins.

For the SWDIO I/O Pin there are additional functions that are called in SWD I/O mode only.
This functions are provided to achieve faster I/O that is possible with some advanced GPIO
peripherals that can independently write/read a single I/O pin without affecting any other pins
of the same I/O port. The following SWDIO I/O Pin functions are provided:
 - \ref PIN_SWDIO_OUT_ENABLE to enable the output mode from the DAP hardware.
 - \ref PIN_SWDIO_OUT_DISABLE to enable the input mode to the DAP hardware.
 - \ref PIN_SWDIO_IN to read from the SWDIO I/O pin with utmost possible speed.
 - \ref PIN_SWDIO_OUT to write to the SWDIO I/O pin with utmost possible speed.
*/


// Configure DAP I/O pins ------------------------------

static int isSWDIO_IN_OUT_SAME() {
    if ((SWDIO_OUT_PIN_PORT == SWDIO_IN_PIN_PORT) && (SWDIO_OUT_PIN_Bit == SWDIO_IN_PIN_Bit)) {
        return 1;
    }
    return 0;
}

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
Configures the DAP Hardware I/O pins for JTAG mode:
 - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
 - TDO to input mode.
*/
static __inline void PORT_JTAG_SETUP(void)
{
    // Set SWCLK(TCK) HIGH
    pin_out_init(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN_Bit);
    SWCLK_TCK_PIN_PORT->BSRR = SWCLK_TCK_PIN;
    // Set SWDIO(TMS) HIGH
    pin_out_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit);
    SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;

    // Set TDI HIGH
    pin_out_init(JTAG_TDI_PIN_PORT, JTAG_TDI_PIN_Bit);
    JTAG_TDI_PIN_PORT->BSRR = JTAG_TDI_PIN;

    // TDO
    pin_in_init(JTAG_TDO_PIN_PORT, JTAG_TDO_PIN_Bit, 1);

    // Set nTRST HIGH
    pin_out_init(JTAG_nTRST_PIN_PORT, JTAG_nTRST_PIN_Bit);
    JTAG_nTRST_PIN_PORT->BSRR = JTAG_nTRST_PIN;

    // Set nRESET HIGH
    pin_out_init(nRESET_PIN_PORT, nRESET_PIN_Bit);
    nRESET_PIN_PORT->BSRR = nRESET_PIN;
}

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
 - SWCLK, SWDIO, nRESET to output mode and set to default high level.
 - TDI, TMS, nTRST to HighZ mode (pins are unused in SWD mode).
*/
static __inline void PORT_SWD_SETUP(void)
{
    // Set SWCLK HIGH
    pin_out_init(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN_Bit);
    SWCLK_TCK_PIN_PORT->BSRR = SWCLK_TCK_PIN;
    // Set SWDIO HIGH
    pin_out_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit);
    SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;

    if (!isSWDIO_IN_OUT_SAME()) {
        pin_in_init(SWDIO_IN_PIN_PORT, SWDIO_IN_PIN_Bit, 1);
    }
    // Set nRESET HIGH
    pin_out_init(nRESET_PIN_PORT, nRESET_PIN_Bit);
    nRESET_PIN_PORT->BSRR = nRESET_PIN;
}

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
 - TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
static __inline void PORT_OFF(void)
{
    pin_in_init(SWCLK_TCK_PIN_PORT, SWCLK_TCK_PIN_Bit, 0);
    pin_in_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit, 0);
    if (!isSWDIO_IN_OUT_SAME()) {
        pin_in_init(SWDIO_IN_PIN_PORT, SWDIO_IN_PIN_Bit, 0);
    }
    pin_in_init(JTAG_TDI_PIN_PORT, JTAG_TDI_PIN_Bit, 0);
	pin_in_init(JTAG_TDO_PIN_PORT, JTAG_TDO_PIN_Bit, 0);
	pin_in_init(JTAG_nTRST_PIN_PORT, JTAG_nTRST_PIN_Bit, 0);
	pin_in_init(nRESET_PIN_PORT, nRESET_PIN_Bit, 0);
}

// SWCLK/TCK I/O pin -------------------------------------

/** SWCLK/TCK I/O pin: Get Input.
\return Current status of the SWCLK/TCK DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWCLK_TCK_IN(void)
{
    return ((SWCLK_TCK_PIN_PORT->ODR & SWCLK_TCK_PIN) ? 1 : 0);
}

/** SWCLK/TCK I/O pin: Set Output to High.
Set the SWCLK/TCK DAP hardware I/O pin to high level.
*/
static __forceinline void PIN_SWCLK_TCK_SET(void)
{
    SWCLK_TCK_PIN_PORT->BSRR = SWCLK_TCK_PIN;
    __asm__ __volatile__("" : : : "memory");
}

/** SWCLK/TCK I/O pin: Set Output to Low.
Set the SWCLK/TCK DAP hardware I/O pin to low level.
*/
static __forceinline void PIN_SWCLK_TCK_CLR(void)
{
    SWCLK_TCK_PIN_PORT->BRR = SWCLK_TCK_PIN;
    __asm__ __volatile__("" : : : "memory");
}

// SWDIO/TMS Pin I/O --------------------------------------

/** SWDIO/TMS I/O pin: Get Input.
\return Current status of the SWDIO/TMS DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWDIO_TMS_IN(void)
{
    return ((SWDIO_IN_PIN_PORT->IDR & SWDIO_IN_PIN) ? 1 : 0);
}

/** SWDIO/TMS I/O pin: Set Output to High.
Set the SWDIO/TMS DAP hardware I/O pin to high level.
*/
static __forceinline void PIN_SWDIO_TMS_SET(void)
{
    SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
    __asm__ __volatile__("" : : : "memory");
}

/** SWDIO/TMS I/O pin: Set Output to Low.
Set the SWDIO/TMS DAP hardware I/O pin to low level.
*/
static __forceinline void PIN_SWDIO_TMS_CLR(void)
{
    SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
    __asm__ __volatile__("" : : : "memory");
}

/** SWDIO I/O pin: Get Input (used in SWD mode only).
\return Current status of the SWDIO DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_SWDIO_IN(void)
{
    return ((SWDIO_IN_PIN_PORT->IDR & SWDIO_IN_PIN) ? 1 : 0);
}

/** SWDIO I/O pin: Set Output (used in SWD mode only).
\param bit Output value for the SWDIO DAP hardware I/O pin.
*/
static __forceinline void PIN_SWDIO_OUT(uint32_t bit)
{
    if (bit & 1)
        SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
    else
        SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
    __asm__ __volatile__("" : : : "memory");
}

/** SWDIO I/O pin: Switch to Output mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to output mode. This function is
called prior \ref PIN_SWDIO_OUT function calls.
*/
static __forceinline void PIN_SWDIO_OUT_ENABLE(void)
{
    pin_out_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit);
    SWDIO_OUT_PIN_PORT->BRR = SWDIO_OUT_PIN;
}

/** SWDIO I/O pin: Switch to Input mode (used in SWD mode only).
Configure the SWDIO DAP hardware I/O pin to input mode. This function is
called prior \ref PIN_SWDIO_IN function calls.
*/
static __forceinline void PIN_SWDIO_OUT_DISABLE(void)
{
    if (isSWDIO_IN_OUT_SAME()) {
        pin_in_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit, 1);
    } else {
        pin_in_init(SWDIO_OUT_PIN_PORT, SWDIO_OUT_PIN_Bit, 0);
        SWDIO_OUT_PIN_PORT->BSRR = SWDIO_OUT_PIN;
    }
}


// TDI Pin I/O ---------------------------------------------

/** TDI I/O pin: Get Input.
\return Current status of the TDI DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_TDI_IN(void)
{
    return ((JTAG_TDI_PIN_PORT->IDR & JTAG_TDI_PIN) ? 1 : 0);
}

/** TDI I/O pin: Set Output.
\param bit Output value for the TDI DAP hardware I/O pin.
*/
static __forceinline void PIN_TDI_OUT(uint32_t bit)
{
    if (bit & 1)
        JTAG_TDI_PIN_PORT->BSRR = JTAG_TDI_PIN;
    else
        JTAG_TDI_PIN_PORT->BRR = JTAG_TDI_PIN;
    __asm__ __volatile__("" : : : "memory");
}


// TDO Pin I/O ---------------------------------------------

/** TDO I/O pin: Get Input.
\return Current status of the TDO DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_TDO_IN(void)
{
    return ((JTAG_TDO_PIN_PORT->IDR & JTAG_TDO_PIN) ? 1 : 0);
}


// nTRST Pin I/O -------------------------------------------

/** nTRST I/O pin: Get Input.
\return Current status of the nTRST DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_nTRST_IN(void)
{
    return ((JTAG_nTRST_PIN_PORT->IDR & JTAG_nTRST_PIN) ? 1 : 0);
}

/** nTRST I/O pin: Set Output.
\param bit JTAG TRST Test Reset pin status:
           - 0: issue a JTAG TRST Test Reset.
           - 1: release JTAG TRST Test Reset.
*/
static __forceinline void PIN_nTRST_OUT(uint32_t bit)
{
    if (bit & 1)
        JTAG_nTRST_PIN_PORT->BSRR = JTAG_nTRST_PIN;
    else
        JTAG_nTRST_PIN_PORT->BRR = JTAG_nTRST_PIN;
    __asm__ __volatile__("" : : : "memory");
}

// nRESET Pin I/O------------------------------------------

/** nRESET I/O pin: Get Input.
\return Current status of the nRESET DAP hardware I/O pin.
*/
static __forceinline uint32_t PIN_nRESET_IN(void)
{
    return ((nRESET_PIN_PORT->IDR >> nRESET_PIN_Bit) & 1);
}

/** nRESET I/O pin: Set Output.
\param bit target device hardware reset pin status:
           - 0: issue a device hardware reset.
           - 1: release device hardware reset.
*/
// TODO - sw specific implementation should be created

static __forceinline void     PIN_nRESET_OUT(uint32_t bit)
{
    if (bit & 1)
        nRESET_PIN_PORT->BSRR = nRESET_PIN;
    else
        nRESET_PIN_PORT->BRR = nRESET_PIN;
    __asm__ __volatile__("" : : : "memory");
}

//**************************************************************************************************
/**
\defgroup DAP_Config_LEDs_gr CMSIS-DAP Hardware Status LEDs
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware may provide LEDs that indicate the status of the CMSIS-DAP Debug Unit.

It is recommended to provide the following LEDs for status indication:
 - Connect LED: is active when the DAP hardware is connected to a debugger.
 - Running LED: is active when the debugger has put the target device into running state.
*/

/** Debug Unit: Set status of Connected LED.
\param bit status of the Connect LED.
           - 1: Connect LED ON: debugger is connected to CMSIS-DAP Debug Unit.
           - 0: Connect LED OFF: debugger is not connected to CMSIS-DAP Debug Unit.
*/
static __inline void LED_CONNECTED_OUT(uint32_t bit)
{
    if (bit & 1)
        CONNECTED_LED_PORT->BRR = CONNECTED_LED_PIN; // LED on
    else
        CONNECTED_LED_PORT->BSRR = CONNECTED_LED_PIN;// LED off
    __asm__ __volatile__("" : : : "memory");
}

/** Debug Unit: Set status Target Running LED.
\param bit status of the Target Running LED.
           - 1: Target Running LED ON: program execution in target started.
           - 0: Target Running LED OFF: program execution in target stopped.
*/
static __inline void LED_RUNNING_OUT(uint32_t bit)
{
    ;             // Not available
}

///@}


//**************************************************************************************************
/**
\defgroup DAP_Config_Initialization_gr CMSIS-DAP Initialization
\ingroup DAP_ConfigIO_gr
@{

CMSIS-DAP Hardware I/O and LED Pins are initialized with the function \ref DAP_SETUP.
*/

/** Setup of the Debug Unit I/O pins and LEDs (called when Debug Unit is initialized).
This function performs the initialization of the CMSIS-DAP Hardware I/O Pins and the
Status LEDs. In detail the operation of Hardware I/O and LED pins are enabled and set:
 - I/O clock system enabled.
 - all I/O pins: input buffer enabled, output pins are set to HighZ mode.
 - for nTRST, nRESET a weak pull-up (if available) is enabled.
 - LED output pins are enabled and LEDs are turned off.
*/
static __inline void DAP_SETUP(void)
{
    /* Enable port clock */
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC;

    /* Configure I/O pin SWCLK */
    PORT_SWD_SETUP();
    PORT_JTAG_SETUP();

    pin_out_init(CONNECTED_LED_PORT, CONNECTED_LED_PIN_Bit);
    CONNECTED_LED_PORT->BSRR = CONNECTED_LED_PIN;
}

/** Reset Target Device with custom specific I/O pin or command sequence.
This function allows the optional implementation of a device specific reset sequence.
It is called when the command \ref DAP_ResetTarget and is for example required
when a device needs a time-critical unlock sequence that enables the debug port.
\return 0 = no device specific reset sequence is implemented.\n
        1 = a device specific reset sequence is implemented.
*/
static __inline uint32_t RESET_TARGET(void)
{
    return (0);              // change to '1' when a device reset sequence is implemented
}

///@}


// Configurable delay for clock generation
#ifndef DELAY_SLOW_CYCLES
#define DELAY_SLOW_CYCLES       10       // Number of cycles for one iteration
#endif
static __forceinline void PIN_DELAY_SLOW (uint32_t delay) {
    int32_t count;

    count = delay;
    while (--count) {
        __NOP();
    }
}

// Fixed delay for fast clock generation
#ifndef DELAY_FAST_CYCLES
#define DELAY_FAST_CYCLES       0       // Number of cycles: 0..3
#endif
static __forceinline void PIN_DELAY_FAST (void) {
#if (DELAY_FAST_CYCLES >= 1)
  __nop();
#endif
#if (DELAY_FAST_CYCLES >= 2)
  __nop();
#endif
#if (DELAY_FAST_CYCLES >= 3)
  __nop();
#endif
}


#endif /* __DAP_CONFIG_H__ */
