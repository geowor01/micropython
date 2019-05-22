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

#include "nrf_gpio.h"

NRF_ADC_Type nrf_adc_obj;

NRF_GPIO_Type nrf_gpio_obj;

PinMode nrf_pin_map[] = { PullNone, PullDown, PullDefault, PullUp };

PinMode get_simulator_pin_mode(nrf_gpio_pin_pull_t nrf_mode) {
    return nrf_pin_map[(int)nrf_mode];
}

int nrf_gpio_pin_read(uint32_t pin_number)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    return gpio_read(&pin_obj);
}

void nrf_gpio_pin_write(uint32_t pin_number, int value)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    gpio_write(&pin_obj, value);
}

void nrf_gpio_pins_write(uint32_t pin_mask, int value) {
    for (int i = 0; i<31; i++) {
        if (pin_mask & (1 << i)) {
            nrf_gpio_pin_write(i, value);
        }
    }
}

void nrf_gpio_pin_set(uint32_t pin_number)
{
    nrf_gpio_pin_write(pin_number, 1);
}

void nrf_gpio_pins_set(uint32_t pin_mask)
{
    nrf_gpio_pins_write(pin_mask, 1);
}

void nrf_gpio_pin_clear(uint32_t pin_number)
{
    nrf_gpio_pin_write(pin_number, 0);
}

void nrf_gpio_pins_clear(uint32_t pin_mask)
{
    nrf_gpio_pins_write(pin_mask, 0);
}

void nrf_gpio_pin_dir(uint32_t pin_number, PinDirection direction)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    gpio_dir(&pin_obj, direction);
    NRF_GPIO->PIN_CNF[pin_number] = (NRF_GPIO->PIN_CNF[pin_number] & ~(3UL << GPIO_PIN_CNF_DIR_Pos))
                                  | ((uint32_t)direction  << GPIO_PIN_CNF_DIR_Pos);
}

void nrf_gpio_set_pull(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    NRF_GPIO->PIN_CNF[pin_number] = (NRF_GPIO->PIN_CNF[pin_number] & ~(3UL << GPIO_PIN_CNF_PULL_Pos))
                                  | ((uint32_t)pull_config  << GPIO_PIN_CNF_PULL_Pos);
}

void nrf_gpio_pin_mode(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    gpio_mode(&pin_obj, get_simulator_pin_mode(pull_config));
    nrf_gpio_set_pull(pin_number, pull_config);
}

void nrf_gpio_cfg_output(uint32_t pin_number)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    gpio_init_out(&pin_obj, pin_obj.pin);
}

void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
    gpio_t pin_obj = { .pin = (PinName)pin_number };
    gpio_init_in_ex(&pin_obj, pin_obj.pin, get_simulator_pin_mode(pull_config));
    nrf_gpio_set_pull(pin_number, pull_config);
}

void nrf_gpio_pins_dir(uint32_t pin_range_start, uint32_t pin_range_end, PinDirection direction) {
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_pin_dir(pin_range_start, direction);
    }
}

void nrf_gpio_pins_mode(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config) {
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_pin_mode(pin_range_start, pull_config);
    }
}

void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_cfg_output(pin_range_start);
    }
}

void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        nrf_gpio_cfg_input(pin_range_start, pull_config);
    }
}

