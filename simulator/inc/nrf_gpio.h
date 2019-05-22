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

#ifndef __MICROPY_NRF_GPIO__
#define __MICROPY_NRF_GPIO__

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio_api.h"
#include "gpio_object.h"

#define ADC_ENABLE_ENABLE_Disabled (0x00UL)
#define ADC_ENABLE_ENABLE_Enabled (0x01UL)

typedef struct {
  uint32_t  ENABLE;                            /*!< ADC enable.                                                           */
} NRF_ADC_Type;

#define ADC_ENABLE_ENABLE_Disabled (0x00UL)

extern NRF_ADC_Type nrf_adc_obj;

#define NRF_ADC (&nrf_adc_obj)

typedef struct {                                    /*!< GPIO Structure                                                        */
  uint32_t  RESERVED0[321];
  uint32_t  OUT;                               /*!< Write GPIO port.                                                      */
  uint32_t  OUTSET;                            /*!< Set individual bits in GPIO port.                                     */
  uint32_t  OUTCLR;                            /*!< Clear individual bits in GPIO port.                                   */
  uint32_t  IN;                                /*!< Read GPIO port.                                                       */
  uint32_t  DIR;                               /*!< Direction of GPIO pins.                                               */
  uint32_t  DIRSET;                            /*!< DIR set register.                                                     */
  uint32_t  DIRCLR;                            /*!< DIR clear register.                                                   */
  uint32_t  RESERVED1[120];
  uint32_t  PIN_CNF[32];                       /*!< Configuration of GPIO pins.                                           */
} NRF_GPIO_Type;

#define GPIO_PIN_CNF_PULL_Pos (2UL)
#define GPIO_PIN_CNF_DIR_Pos (0UL)

extern NRF_GPIO_Type nrf_gpio_obj;

#define NRF_GPIO (&nrf_gpio_obj)

typedef enum
{
    NRF_GPIO_PIN_NOPULL   = 0x00UL,                 ///<  Pin pullup resistor disabled
    NRF_GPIO_PIN_PULLDOWN = 0x01UL,                 ///<  Pin pulldown resistor enabled
    NRF_GPIO_PIN_PULLUP   = 0x03UL,                   ///<  Pin pullup resistor enabled
} nrf_gpio_pin_pull_t;

int nrf_gpio_pin_read(uint32_t pin_number);

void nrf_gpio_pin_write(uint32_t pin_number, int value);

void nrf_gpio_pin_set(uint32_t pin_number);

void nrf_gpio_pins_set(uint32_t pin_mask);

void nrf_gpio_pin_clear(uint32_t pin_number);

void nrf_gpio_pins_clear(uint32_t pin_mask);

void nrf_gpio_cfg_output(uint32_t pin_number);

void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config);

void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end);

void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config);

#ifdef __cplusplus
}
#endif

#endif // __MICROPY_NRF_GPIO__
