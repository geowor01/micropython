/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "us_ticker_api.h"
#include "wait_api.h"
#include "MicroBitSystemTimer.h"
#include "MicroBitSerial.h"

extern "C" {

#include "py/runtime.h"
#include "py/mphal.h"
#include "lib/utils/interrupt_char.h"
#include "microbit/modmicrobit.h"

#define UART_RX_BUF_SIZE (64) // it's large so we can paste example code

static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_buf_head, uart_rx_buf_tail;

MicroBitSerial ubit_serial(USBTX, USBRX);

void uart_rx_irq(void) {
    if (!ubit_serial.readable()) {
        return;
    }
    int c = ubit_serial.getc();
    if (c == mp_interrupt_char) {
        mp_keyboard_interrupt();
    } else {
        uint16_t next_head = (uart_rx_buf_head + 1) % UART_RX_BUF_SIZE;
        if (next_head != uart_rx_buf_tail) {
            // only store data if room in buf
            uart_rx_buf[uart_rx_buf_head] = c;
            uart_rx_buf_head = next_head;
        }
    }
}

void mp_hal_init(void) {
    uart_rx_buf_head = 0;
    uart_rx_buf_tail = 0;
    ubit_serial.attach(uart_rx_irq);
}

int mp_hal_stdin_rx_any(void) {
    return uart_rx_buf_tail != uart_rx_buf_head;
}

int mp_hal_stdin_rx_chr(void) {
    while (uart_rx_buf_tail == uart_rx_buf_head) {
        wait_ms(10);
    }
    int c = uart_rx_buf[uart_rx_buf_tail];
    uart_rx_buf_tail = (uart_rx_buf_tail + 1) % UART_RX_BUF_SIZE;
    return c;
}

void mp_hal_stdout_tx_str(const char *str) {
    mp_hal_stdout_tx_strn(str, strlen(str));
}

void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    for (; len > 0; --len) {
        ubit_serial.putc(*str++);
    }
}

void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    for (; len > 0; --len) {
        if (*str == '\n') {
            ubit_serial.putc('\r');
        }
        ubit_serial.putc(*str++);
    }
}

STATIC void mp_hal_print_many(const char chrs[8], unsigned int total) {
    while (total > 0) {
        unsigned int n = total;
        if (n > 8) {
            n = 8;
        }
        total -= n;
        mp_hal_stdout_tx_strn(chrs, n);
    }
}

void mp_hal_move_cursor_back(unsigned int pos) {
    mp_hal_print_many("\b\b\b\b\b\b\b\b", pos);
}

void mp_hal_erase_line_from_cursor(unsigned int n_chars) {
    mp_hal_print_many("        ", n_chars);
    mp_hal_move_cursor_back(n_chars);
}

int mp_hal_display_string(const char *str) {
    return microbit_display_scroll(&microbit_display_obj, str);
}

void mp_hal_delay_us(mp_uint_t us) {
    mp_uint_t start = mp_hal_ticks_us();
    while (mp_hal_ticks_us() - start < us) {
        wait_us(1);
    }
}

void mp_hal_delay_ms(mp_uint_t ms) {
    mp_uint_t start = mp_hal_ticks_ms();
    while (mp_hal_ticks_ms() - start < ms) {
        wait_ms(1);
    }
}

mp_uint_t mp_hal_ticks_us(void) {
    return us_ticker_read();
}

mp_uint_t mp_hal_ticks_ms(void) {
    return mp_hal_ticks_us() / 1000;
}

void crash_micropython(const char *str) {
    MP_STATE_THREAD(cur_exc) = NULL;
    mp_hal_stdout_tx_str("\r\nMicroPython crashes due to ");
    mp_hal_stdout_tx_str(str);
    mp_hal_stdout_tx_str(". \r\nSimulator will reset after scrolling text.\r\nPress Ctrl + C to reset now.\r\n");
    mp_hal_display_string("MicroPython crashes due to ");
    mp_hal_display_string(str);
    mp_hal_display_string(". \r\nSimulator will now reset.");
    microbit_reset();
    wait_ms(1000);
}

}