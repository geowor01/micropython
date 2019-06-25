/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
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

#include <stdlib.h>
#include <assert.h>

#include "py/objtuple.h"
#include "py/runtime.h"

typedef struct _mp_obj_zip_t {
    mp_obj_base_t base;
    size_t n_iters;
    mp_obj_t iters[];
} mp_obj_zip_t;

STATIC mp_obj_t zip_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, false);

    mp_obj_zip_t *o = m_new_obj_var(mp_obj_zip_t, mp_obj_t, n_args);
    if (!o) {
        return MP_OBJ_NULL;
    }
    o->base.type = type;
    o->n_iters = n_args;
    m_rs_push_ptr(o);
    for (size_t i = 0; i < n_args; i++) {
        o->iters[i] = mp_getiter(args[i], NULL);
    }
    m_rs_pop_ptr(o);
    return MP_OBJ_FROM_PTR(o);
}

STATIC mp_obj_t zip_iternext(mp_obj_t self_in) {
    mp_check_self(MP_OBJ_IS_TYPE(self_in, &mp_type_zip));
    mp_obj_zip_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->n_iters == 0) {
        return MP_OBJ_STOP_ITERATION;
    }
    mp_obj_tuple_t *tuple = MP_OBJ_TO_PTR(mp_obj_new_tuple(self->n_iters, NULL));
    m_rs_push_ptr(tuple);

    for (size_t i = 0; i < self->n_iters; i++) {
        mp_obj_t next = mp_iternext(self->iters[i]);
        if (next == MP_OBJ_NULL) {
            // exception
            return MP_OBJ_NULL;
        }
        if (next == MP_OBJ_STOP_ITERATION) {
            mp_obj_tuple_del(MP_OBJ_FROM_PTR(tuple));
            m_rs_pop_ptr(tuple);
            return MP_OBJ_STOP_ITERATION;
        }
        tuple->items[i] = next;
    }
    m_rs_pop_ptr(tuple);
    return MP_OBJ_FROM_PTR(tuple);
}

const mp_obj_type_t mp_type_zip = {
    { &mp_type_type },
    .name = MP_QSTR_zip,
    .make_new = zip_make_new,
    .getiter = mp_identity_getiter,
    .iternext = zip_iternext,
};
