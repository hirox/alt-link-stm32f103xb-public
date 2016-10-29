/**
 * @file    SW_DP.c
 * @brief   SWD driver
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

#include "DAP_config.h"
#include "DAP.h"


// SW Macros

#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#if 1
#undef PIN_SWCLK_SET
#undef PIN_SWDIO_OUT
#undef PIN_SWCLK_CLR
#undef PIN_SWDIO_IN

#define PIN_SWCLK_SET() *bsrr = SWCLK_TCK_PIN; __asm__ __volatile__("" : : : "memory");
#define PIN_SWCLK_CLR() *brr = SWCLK_TCK_PIN; __asm__ __volatile__("" : : : "memory");
//#define PIN_SWDIO_OUT(bit) *(brr - (bit & 1)) = SWDIO_OUT_PIN; __asm__ __volatile__("" : : : "memory");
#define PIN_SWDIO_OUT(bit) if (bit & 1) { *bsrr = SWDIO_OUT_PIN; } else { *brr = SWDIO_OUT_PIN; }; __asm__ __volatile__("" : : : "memory");
#define PIN_SWDIO_IN() ((*idr & SWDIO_IN_PIN) ? 1 : 0);
#endif

#define SW_CLOCK_CYCLE()                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_WRITE_BIT(bit)               \
  PIN_SWDIO_OUT(bit);                   \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define SW_READ_BIT(bit)                \
  PIN_SWCLK_CLR();                      \
  PIN_DELAY();                          \
  bit = PIN_SWDIO_IN();                 \
  PIN_SWCLK_SET();                      \
  PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)


// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
#if ((DAP_SWD != 0) || (DAP_JTAG != 0))
void SWJ_Sequence (uint32_t count, uint8_t *data) {
  uint32_t val;
  uint32_t n;
  volatile uint32_t *bsrr = &SWCLK_TCK_PIN_PORT->BSRR;
  volatile uint32_t *brr = &SWCLK_TCK_PIN_PORT->BRR;

  val = 0;
  n = 0;
  while (count--) {
    if (n == 0) {
      val = *data++;
      n = 8;
    }
    if (val & 1) {
      PIN_SWDIO_TMS_SET();
    } else {
      PIN_SWDIO_TMS_CLR();
    }
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
}
#endif


#if (DAP_SWD != 0)

// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
#define SWD_TransferFunction(speed)     /**/                                    \
uint8_t SWD_Transfer##speed (uint32_t request, uint32_t *data) {                \
  volatile uint32_t *bsrr = &SWCLK_TCK_PIN_PORT->BSRR;                          \
  volatile uint32_t *brr = &SWCLK_TCK_PIN_PORT->BRR;                            \
  /* SWCLK_TCK_PIN_PORT と SWDIO_OUT_PIN_PORT は同じ前提 */                     \
  /* volatile uint32_t *iobrr = &SWDIO_OUT_PIN_PORT->BRR; */                    \
  volatile uint32_t *idr = &SWDIO_IN_PIN_PORT->IDR;                             \
  uint32_t ack;                                                                 \
  uint32_t parity;                                                              \
                                                                                \
  uint32_t n;                                                                   \
                                                                                \
  /* Packet Request */                                                          \
  parity = 0;                                                                   \
  SW_WRITE_BIT(1);                      /* Start Bit */                         \
  n = request >> 0;                                                           \
  SW_WRITE_BIT(n);                    /* APnDP Bit */                         \
  parity += n;                                                                \
  n = request >> 1;                                                           \
  SW_WRITE_BIT(n);                    /* RnW Bit */                           \
  parity += n;                                                                \
  n = request >> 2;                                                           \
  SW_WRITE_BIT(n);                    /* A2 Bit */                            \
  parity += n;                                                                \
  n = request >> 3;                                                           \
  SW_WRITE_BIT(n);                    /* A3 Bit */                            \
  parity += n;                                                                \
  SW_WRITE_BIT(parity);                 /* Parity Bit */                        \
  SW_WRITE_BIT(0);                      /* Stop Bit */                          \
  SW_WRITE_BIT(1);                      /* Park Bit */                          \
                                                                                \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    SW_CLOCK_CYCLE();                                                           \
  }                                                                             \
                                                                                \
  /* Acknowledge response */                                                    \
  SW_READ_BIT(n);                                                             \
  ack  = n << 0;                                                              \
  SW_READ_BIT(n);                                                             \
  ack |= n << 1;                                                              \
  SW_READ_BIT(n);                                                             \
  ack |= n << 2;                                                              \
                                                                                \
  if (ack == DAP_TRANSFER_OK) {         /* OK response */                       \
    /* Data transfer */                                                         \
    if (request & DAP_TRANSFER_RnW) {                                           \
      /* Read data */                                                           \
      uint32_t val = 0;                                                         \
      uint32_t bit;                                                         \
      parity = 0;                                                               \
      n = 32;                                                                   \
      while (n) {                                                             \
        SW_READ_BIT(bit);               /* Read RDATA[0:31] */                  \
        parity += bit;                                                          \
        val >>= 1;                                                              \
        val  |= bit << 31;                                                      \
        n--;                                                                    \
      }                                                                         \
      SW_READ_BIT(bit);                 /* Read Parity */                       \
      if ((parity ^ bit) & 1) {                                                 \
        ack = DAP_TRANSFER_ERROR;                                               \
      }                                                                         \
      if (data) *data = val;                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
    } else {                                                                    \
      /* Turnaround */                                                          \
      for (n = DAP_Data.swd_conf.turnaround; n; n--) {                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
      PIN_SWDIO_OUT_ENABLE();                                                   \
      /* Write data */                                                          \
      uint32_t val = *data;                                                     \
      parity = 0;                                                               \
      n = 32;                                                               \
      /*volatile uint32_t *_bsrr = &SWCLK_TCK_PIN_PORT->BSRR;*/                          \
      while (n) {                                                    \
        /*SW_WRITE_BIT(val);*/              /* Write WDATA[0:31] */                 \
        PIN_SWDIO_OUT(val);                   \
        PIN_SWCLK_CLR();                      \
        PIN_DELAY();                          \
        __asm__ __volatile__ (" ");           \
        parity += val;                                                          \
        val >>= 1;                                                              \
        n--;                                  \
        __asm__ __volatile__ (" ");           \
        PIN_SWCLK_SET();                      \
        PIN_DELAY();                          \
      }                                                                         \
      SW_WRITE_BIT(parity);             /* Write Parity Bit */                  \
    }                                                                           \
    /* Idle cycles */                                                           \
    n = DAP_Data.transfer.idle_cycles;                                          \
    if (n) {                                                                    \
      PIN_SWDIO_OUT(0);                                                         \
      for (; n; n--) {                                                          \
        SW_CLOCK_CYCLE();                                                       \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1);                                                           \
    return (ack);                                                               \
  }                                                                             \
                                                                                \
  if ((ack == DAP_TRANSFER_WAIT) || (ack == DAP_TRANSFER_FAULT)) {              \
    /* WAIT or FAULT response */                                                \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) != 0)) {  \
      for (n = 32+1; n; n--) {                                                  \
        SW_CLOCK_CYCLE();               /* Dummy Read RDATA[0:31] + Parity */   \
      }                                                                         \
    }                                                                           \
    /* Turnaround */                                                            \
    for (n = DAP_Data.swd_conf.turnaround; n; n--) {                            \
      SW_CLOCK_CYCLE();                                                         \
    }                                                                           \
    PIN_SWDIO_OUT_ENABLE();                                                     \
    if (DAP_Data.swd_conf.data_phase && ((request & DAP_TRANSFER_RnW) == 0)) {  \
      PIN_SWDIO_OUT(0);                                                         \
      for (n = 32+1; n; n--) {                                                  \
        SW_CLOCK_CYCLE();               /* Dummy Write WDATA[0:31] + Parity */  \
      }                                                                         \
    }                                                                           \
    PIN_SWDIO_OUT(1);                                                           \
    return (ack);                                                               \
  }                                                                             \
                                                                                \
  /* Protocol error */                                                          \
  for (n = DAP_Data.swd_conf.turnaround + 32 + 1; n; n--) {                     \
    SW_CLOCK_CYCLE();                   /* Back off data phase */               \
  }                                                                             \
  PIN_SWDIO_OUT(1);                                                             \
  return (ack);                                                                 \
}


#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_FAST()
SWD_TransferFunction(Fast);

#undef  PIN_DELAY
#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)
SWD_TransferFunction(Slow);


// SWD Transfer I/O
//   request: A[3:2] RnW APnDP
//   data:    DATA[31:0]
//   return:  ACK[2:0]
uint8_t  SWD_Transfer(uint32_t request, uint32_t *data) {
  if (DAP_Data.fast_clock) {
    return SWD_TransferFast(request, data);
  } else {
    return SWD_TransferSlow(request, data);
  }
}


#endif  /* (DAP_SWD != 0) */
