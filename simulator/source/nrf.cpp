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


#include "nrf.h"
#include "emscripten.h"
#include "MicroBitAccelerometer.h"


extern "C" {

NRF_TEMP_Type nrf_temp;
NRF_TIMER_Type nrf_timer0;

void __NOP() {}

void ticker_handler(const ticker_data_t *data) {
    static uint32_t last_slow_tick = 0;
    uint32_t time = us_ticker_read();
    uint32_t delta = time - last_slow_tick;
    TIMER0_IRQHandler();
    if (delta > 4000) {
        last_slow_tick = time;
        SWI3_IRQHandler();
        SWI4_IRQHandler();
        MicroBitAccelerometer::autoDetect().update();
        if (NRF_TEMP->TASKS_START && NRF_TEMP->EVENTS_DATARDY == 0) {
            NRF_TEMP->TEMP = EM_ASM_INT({ return MbedJSUI.TemperatureSensor.read(); }) * 4;
            NRF_TEMP->EVENTS_DATARDY = 1;
            NRF_TEMP->TASKS_START = 0;
        }
    }
    us_ticker_set_interrupt(time + 1000);
}

void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {}

void NVIC_SetPendingIRQ(IRQn_Type IRQn) {}

void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    if (IRQn == TIMER0_IRQn) {
        set_us_ticker_irq_handler(ticker_handler);
        us_ticker_set_interrupt(us_ticker_read() + 1000);
    }
}

void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    us_ticker_disable_interrupt();
}

}