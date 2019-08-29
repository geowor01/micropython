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

typedef struct {                               /*!< RADIO Structure                                                       */
  uint32_t  TASKS_TXEN;                        /*!< Enable radio in TX mode.                                              */
  uint32_t  TASKS_RXEN;                        /*!< Enable radio in RX mode.                                              */
  uint32_t  TASKS_START;                       /*!< Start radio.                                                          */
  uint32_t  TASKS_DISABLE;                     /*!< Disable radio.                                                        */
  uint32_t  EVENTS_READY;                      /*!< Ready event.                                                          */
  uint32_t  EVENTS_END;                        /*!< End event.                                                            */
  uint32_t  EVENTS_DISABLED;                   /*!< Disable event.                                                        */
  uint32_t  SHORTS;                            /*!< Shortcuts for the radio.                                              */
  uint32_t  INTENSET;                          /*!< Interrupt enable set register.                                        */
  uint32_t  CRCSTATUS;                         /*!< CRC status of received packet.                                        */
  uint32_t  PACKETPTR;                         /*!< Packet pointer. Decision point: START task.                           */
  uint32_t  FREQUENCY;                         /*!< Frequency.                                                            */
  uint32_t  TXPOWER;                           /*!< Output power.                                                         */
  uint32_t  MODE;                              /*!< Data rate and modulation.                                             */
  uint32_t  PCNF0;                             /*!< Packet configuration 0.                                               */
  uint32_t  PCNF1;                             /*!< Packet configuration 1.                                               */
  uint32_t  BASE0;                             /*!< Radio base address 0. Decision point: START task.                     */
  uint32_t  PREFIX0;                           /*!< Prefixes bytes for logical addresses 0 to 3.                          */
  uint32_t  TXADDRESS;                         /*!< Transmit address select.                                              */
  uint32_t  RXADDRESSES;                       /*!< Receive address select.                                               */
  uint32_t  CRCCNF;                            /*!< CRC configuration.                                                    */
  uint32_t  CRCPOLY;                           /*!< CRC polynomial.                                                       */
  uint32_t  CRCINIT;                           /*!< CRC initial value.                                                    */
  uint32_t  RSSISAMPLE;                        /*!< RSSI sample.                                                          */
  uint32_t  DATAWHITEIV;                       /*!< Data whitening initial value.                                         */
} NRF_RADIO_Type;

typedef struct {                               /*!< CLOCK Structure                                                       */
  uint32_t  TASKS_HFCLKSTART;                  /*!< Start HFCLK clock source.                                             */
  uint32_t  EVENTS_HFCLKSTARTED;               /*!< HFCLK oscillator started.                                             */
} NRF_CLOCK_Type;

typedef struct {                               /*!< TEMP Structure                                                        */
  uint32_t  TASKS_START;                       /*!< Start temperature measurement.                                        */
  uint32_t  TASKS_STOP;                        /*!< Stop temperature measurement.                                         */
  uint32_t  EVENTS_DATARDY;                    /*!< Temperature measurement complete, data ready event.                   */
  int32_t   TEMP;                              /*!< Die temperature in degC, 2's complement format, 0.25 degC pecision.   */
} NRF_TEMP_Type;

extern NRF_RADIO_Type nrf_radio;
extern NRF_CLOCK_Type nrf_clock;
extern NRF_TEMP_Type nrf_temp;

#define NRF_RADIO (&nrf_radio)
#define NRF_CLOCK (&nrf_clock)
#define NRF_TEMP (&nrf_temp)

void __NOP();

#define SWI3_IRQn TIMER1_IRQn
#define SWI4_IRQn TIMER2_IRQn
#define RADIO_IRQn TIMER3_IRQn

#define RADIO_CRCCNF_LEN_Two (2UL) /*!< Two bytes long CRC. */
#define RADIO_SHORTS_ADDRESS_RSSISTART_Pos (4UL) /*!< Position of ADDRESS_RSSISTART field. */
#define RADIO_SHORTS_ADDRESS_RSSISTART_Msk (0x1UL << RADIO_SHORTS_ADDRESS_RSSISTART_Pos) /*!< Bit mask of ADDRESS_RSSISTART field. */
#define RADIO_MODE_MODE_Nrf_1Mbit (0x00UL) /*!< 1Mbit/s Nordic propietary radio mode. */
#define RADIO_MODE_MODE_Nrf_2Mbit (0x01UL) /*!< 2Mbit/s Nordic propietary radio mode. */
#define RADIO_MODE_MODE_Nrf_250Kbit (0x02UL) /*!< 250kbit/s Nordic propietary radio mode. */

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
extern void RADIO_IRQHandler(void);

extern volatile uint32_t us_tick_delta;
extern volatile bool perform_reset;

void ticker_handler(const ticker_data_t *data);

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority);

void NVIC_SetPendingIRQ(IRQn_Type IRQn);

void NVIC_ClearPendingIRQ(IRQn_Type IRQn);

void NVIC_EnableIRQ(IRQn_Type IRQn);

void NVIC_DisableIRQ(IRQn_Type IRQn);

#ifdef __cplusplus
}
#endif

#endif // __MICROPY_NRF__
