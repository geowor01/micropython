/*
 * Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MICROPY_NRF__
#define __MICROPY_NRF__

#ifdef __cplusplus
extern "C" {
#endif

#define MICROBIT_PIN_BUTTON_A                   P0_17
#define MICROBIT_PIN_BUTTON_B                   P0_26
#define MICROBIT_PIN_BUTTON_RESET               P0_19

#include "us_ticker_api.h"

typedef struct {                                    /*!< TEMP Structure                                                        */
  uint32_t  TASKS_START;                       /*!< Start temperature measurement.                                        */
  uint32_t  TASKS_STOP;                        /*!< Stop temperature measurement.                                         */
  uint32_t  EVENTS_DATARDY;                    /*!< Temperature measurement complete, data ready event.                   */
  int32_t   TEMP;                              /*!< Die temperature in degC, 2's complement format, 0.25 degC pecision.   */
} NRF_TEMP_Type;

extern NRF_TEMP_Type nrf_temp;

#define NRF_TEMP (&nrf_temp)

void __NOP();

#define SWI3_IRQn TIMER2_IRQn
#define SWI4_IRQn TIMER3_IRQn

#define TIMER_INTENSET_COMPARE3_Pos (19UL)
#define TIMER_INTENSET_COMPARE3_Msk (0x1UL << TIMER_INTENSET_COMPARE3_Pos)
#define TIMER_INTENCLR_COMPARE2_Pos (18UL)
#define TIMER_INTENCLR_COMPARE2_Msk (0x1UL << TIMER_INTENCLR_COMPARE2_Pos)
#define TIMER_INTENCLR_COMPARE1_Pos (17UL)
#define TIMER_INTENCLR_COMPARE1_Msk (0x1UL << TIMER_INTENCLR_COMPARE1_Pos)
#define TIMER_INTENCLR_COMPARE0_Pos (16UL)
#define TIMER_INTENCLR_COMPARE0_Msk (0x1UL << TIMER_INTENCLR_COMPARE0_Pos)

#define TIMER_MODE_MODE_Timer (0UL)
#define TIMER_BITMODE_BITMODE_Pos (0UL)
#define TIMER_BITMODE_BITMODE_24Bit (0x02UL)

typedef struct {
    uint32_t  TASKS_START;
    uint32_t  TASKS_STOP;
    uint32_t  TASKS_COUNT;
    uint32_t  TASKS_CLEAR;
    uint32_t  TASKS_SHUTDOWN;
    uint32_t  RESERVED0[11];
    uint32_t  TASKS_CAPTURE[4];
    uint32_t  RESERVED1[60];
    uint32_t  EVENTS_COMPARE[4];
    uint32_t  RESERVED2[44];
    uint32_t  SHORTS;
    uint32_t  RESERVED3[64];
    uint32_t  INTENSET;
    uint32_t  INTENCLR;
    uint32_t  RESERVED4[126];
    uint32_t  MODE;
    uint32_t  BITMODE;
    uint32_t  RESERVED5;
    uint32_t  PRESCALER;

    uint32_t  RESERVED6[11];
    uint32_t  CC[4];
    uint32_t  RESERVED7[683];
    uint32_t  POWER;
} NRF_TIMER_Type;

extern NRF_TIMER_Type nrf_timer0;

#define NRF_TIMER0 (&nrf_timer0)

void TIMER0_IRQHandler(void);
void SWI3_IRQHandler(void);
void SWI4_IRQHandler(void);

void ticker_handler(const ticker_data_t *data);

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);

void NVIC_SetPendingIRQ(IRQn_Type IRQn);

void NVIC_EnableIRQ(IRQn_Type IRQn);

void NVIC_DisableIRQ(IRQn_Type IRQn);

#ifdef __cplusplus
}
#endif

#endif // __MICROPY_NRF__
