/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Josef Gajdusek
 * Copyright (c) 2015 Paul Sokolovsky
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

#include "py/runtime.h"
#include "py/obj.h"

void mp_call_function_1_protected(mp_obj_t fun, mp_obj_t arg) {
    m_rs_push_barrier();
    mp_obj_t ret = mp_call_function_1(fun, arg);
    m_rs_clear_to_barrier();
    if (MP_STATE_THREAD(cur_exc) != NULL) {
        m_rs_push_barrier();
        MP_STATE_THREAD(cur_exc) = NULL;
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(MP_STATE_THREAD(cur_exc)));
        m_rs_clear_to_barrier();
    }
}

void mp_call_function_2_protected(mp_obj_t fun, mp_obj_t arg1, mp_obj_t arg2) {
    m_rs_push_barrier();
    mp_obj_t ret = mp_call_function_2(fun, arg1, arg2);
    m_rs_clear_to_barrier();
    if (MP_STATE_THREAD(cur_exc) != NULL) {
        m_rs_push_barrier();
        MP_STATE_THREAD(cur_exc) = NULL;
        mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(MP_STATE_THREAD(cur_exc)));
        m_rs_clear_to_barrier();
    }
}
