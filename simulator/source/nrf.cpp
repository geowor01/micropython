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

#include <map>

#include "nrf.h"
#include "emscripten.h"
#include "MicroBitAccelerometer.h"
#include "py/mpstate.h"

extern "C" {

NRF_RADIO_Type nrf_radio;
NRF_CLOCK_Type nrf_clock;
NRF_TEMP_Type nrf_temp;
NRF_TIMER_Type nrf_timer0;

bool radio_enabled = false;

typedef enum {
    RADIO_DISABLED = 0,
    RADIO_TRANSMIT_RDY = 1,
    RADIO_RECEIVE_RDY = 2,
    RADIO_RECEIVE_ACTIVE = 3
} mp_radio_state_t;

mp_radio_state_t radio_state = RADIO_DISABLED;

void __NOP() {}

static uint16_t compute_crc16(const uint8_t *buf, uint32_t buf_len)
{
    uint16_t crc = NRF_RADIO->CRCINIT;
    const uint32_t poly = NRF_RADIO->CRCPOLY;

    for(int i = 0; i < buf_len; ++i) {
        crc ^= (uint16_t)(buf[i] << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = ((uint16_t)(crc << 1)) ^ poly;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// The number of milliseconds that have passed since the ticker was initialised
volatile uint32_t us_tick_delta = 6000;
volatile bool perform_reset = false;

EMSCRIPTEN_KEEPALIVE
extern void reset_device()
{
    perform_reset = true;
}

EMSCRIPTEN_KEEPALIVE
extern void radio_receive(const uint8_t *buf)
{
    if (radio_enabled && radio_state == RADIO_RECEIVE_ACTIVE) {
        uint32_t max_len = (NRF_RADIO->PCNF1 & 0xff) + 1;
        uint32_t len = buf[5] + 1;
        if (len > max_len) {
            len = max_len;
        }

        if (buf[4] != NRF_RADIO->PREFIX0
         || buf[len + 7] != NRF_RADIO->FREQUENCY
         || (uint32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)) != NRF_RADIO->BASE0) {
            return;
        }

        memcpy((uint8_t *)NRF_RADIO->PACKETPTR, buf + 5, len);
        NRF_RADIO->EVENTS_END = 1;
        NRF_RADIO->CRCSTATUS = (uint16_t)((buf[len + 5] << 8) | buf[len + 6]) == compute_crc16(buf, len + 5);
        NRF_RADIO->RSSISAMPLE = buf[len + 8];
        RADIO_IRQHandler();
        radio_state = RADIO_RECEIVE_RDY;
    }
}

#define RADIO_BUF_SIZE 1024
static uint32_t current_buf = 0;
static uint8_t radio_packet_buffer[RADIO_BUF_SIZE] __attribute__((aligned(MBED_CONF_APP_MICROBIT_PAGE_SIZE)));

extern void radio_broadcast()
{
    if (radio_state == RADIO_TRANSMIT_RDY) {
        size_t max_len = (NRF_RADIO->PCNF1 & 0xff) + 1;
        size_t len = MP_STATE_PORT(radio_buf)[0] + 1;
        if (len > max_len) {
            len = max_len;
        }

        if (current_buf + len + 8 > RADIO_BUF_SIZE) {
            current_buf = 0;
        }

        // Based on packet configuration here:
        // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fradio.html&cp=2_1_0_22_13_3
        radio_packet_buffer[current_buf] = NRF_RADIO->BASE0 & 0xff;
        radio_packet_buffer[current_buf + 1] = (NRF_RADIO->BASE0 >> 8) & 0xff;
        radio_packet_buffer[current_buf + 2] = (NRF_RADIO->BASE0 >> 16) & 0xff;
        radio_packet_buffer[current_buf + 3] = (NRF_RADIO->BASE0 >> 24) & 0xff;
        radio_packet_buffer[current_buf + 4] = NRF_RADIO->PREFIX0;

        memcpy(radio_packet_buffer + current_buf + 5, MP_STATE_PORT(radio_buf), len);

        uint16_t crc = compute_crc16(radio_packet_buffer + current_buf, len + 5);
        radio_packet_buffer[current_buf + 5 + len] = (uint8_t)(crc >> 8);
        radio_packet_buffer[current_buf + 5 + len + 1] = (uint8_t)(crc & 0xff);

        // Frequency and power tagged on the end as meta-data for the simulator.
        static std::map<uint32_t, int8_t> power_map = {{-30, -50}, {-20, -35}, {-16, -25}, {-12, -20}, {-8, -16}, {-4, -12}, {0, -8}, {4, -4}};
        radio_packet_buffer[current_buf + 5 + len + 2] = NRF_RADIO->FREQUENCY;
        radio_packet_buffer[current_buf + 5 + len + 3] = -power_map[NRF_RADIO->TXPOWER];

        EM_ASM_({ window.MbedJSUI.RadioPacketManager.prototype.broadcast(new Uint8Array(Module.HEAPU8.subarray($0, $0 + $1))); }, radio_packet_buffer + current_buf, len + 9);
        emscripten_sleep(1);

        current_buf += len + 8;
        if (current_buf > RADIO_BUF_SIZE) {
            current_buf = 0;
        }

        NRF_RADIO->EVENTS_END = 1;
    }
}

void update_tick_delta() {
    static uint32_t last_tick = 0;
    uint32_t time = us_ticker_read();
    us_tick_delta = last_tick == 0 ? 6000 : time - last_tick;
    last_tick = time;
}

void ticker_handler(const ticker_data_t *data) {
    if (perform_reset) {
        return;
    }
    update_tick_delta();
    TIMER0_IRQHandler();
    SWI3_IRQHandler();
    SWI4_IRQHandler();
    if (NRF_RADIO->TASKS_START) {
        // Start radio
        if (radio_state == RADIO_RECEIVE_RDY) {
            radio_state = RADIO_RECEIVE_ACTIVE;
        }
        else if (radio_state == RADIO_TRANSMIT_RDY) {
            radio_broadcast();
        }
        NRF_RADIO->TASKS_START = 0;
    }
    if (NRF_RADIO->TASKS_RXEN) {
        // Enable radio in receiving mode
        radio_state = RADIO_RECEIVE_RDY;
        NRF_RADIO->EVENTS_READY = 1;
        NRF_RADIO->TASKS_RXEN = 0;
    }
    if (NRF_RADIO->TASKS_TXEN) {
        // Enable radio in transmission mode
        radio_state = RADIO_TRANSMIT_RDY;
        NRF_RADIO->EVENTS_READY = 1;
        NRF_RADIO->TASKS_TXEN = 0;
    }
    if (NRF_RADIO->TASKS_DISABLE) {
        // Disable radio
        radio_state = RADIO_DISABLED;
        NRF_RADIO->EVENTS_DISABLED = 1;
        NRF_RADIO->TASKS_DISABLE = 0;
    }
    if (NRF_CLOCK->TASKS_HFCLKSTART) {
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 1;
        NRF_CLOCK->TASKS_HFCLKSTART = 0;
    }
    if (NRF_TEMP->TASKS_START && NRF_TEMP->EVENTS_DATARDY == 0) {
        NRF_TEMP->TEMP = EM_ASM_INT({ return MbedJSUI.TemperatureSensor.read(); }) * 4;
        NRF_TEMP->EVENTS_DATARDY = 1;
        NRF_TEMP->TASKS_START = 0;
    }
    static uint32_t last_accelerometer_call = 0;
    static uint32_t us_accelerometer_delta = 6000;

    uint32_t time = us_ticker_read();
    us_accelerometer_delta = last_accelerometer_call == 0 ? 100000 : time - last_accelerometer_call;
    if (us_accelerometer_delta > 20000) {
        last_accelerometer_call = time;
        MicroBitAccelerometer::autoDetect().update();
    }
    us_ticker_set_interrupt(0);
}

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {}

void NVIC_SetPendingIRQ(IRQn_Type IRQn) {}

void NVIC_ClearPendingIRQ(IRQn_Type IRQn) {}

void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    if (IRQn == RADIO_IRQn) {
        radio_enabled = true;
    }
    else if (IRQn == TIMER0_IRQn) {
        set_us_ticker_irq_handler(ticker_handler);
        us_ticker_set_interrupt(0);
    }
}

void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    if (IRQn == RADIO_IRQn) {
        radio_enabled = false;
    }
    else {
        us_ticker_disable_interrupt();
    }
}

}