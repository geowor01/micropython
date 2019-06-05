/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2017 Damien P. George
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

#include "microbit/microbitdal.h"

extern "C" {

#include "py/runtime.h"
#include "microbit/modmicrobit.h"

typedef struct _microbit_i2c_obj_t {
    mp_obj_base_t base;
    MicroPythonI2C *i2c;
} microbit_i2c_obj_t;

volatile bool i2c_in_use = false;

STATIC volatile const microbit_pin_obj_t *pin_sda = &microbit_p20_obj;
STATIC volatile const microbit_pin_obj_t *pin_scl = &microbit_p19_obj;
STATIC int frequency = 100000;

bool i2c_matches_accel(void) {
    return !(pin_sda != &microbit_p20_obj || pin_scl != &microbit_p19_obj || frequency != ubit_i2c.get_i2c_obj()->freq);
}

STATIC void microbit_i2c_pin_free(const microbit_pin_obj_t *pin) {
    if (microbit_pin_get_mode(pin) == microbit_pin_mode_i2c) {
        microbit_obj_pin_free(pin);
    }
}

void microbit_i2c_free_current(void) {
    microbit_i2c_pin_free((const microbit_pin_obj_t *)pin_sda);
    microbit_i2c_pin_free((const microbit_pin_obj_t *)pin_scl);
}

STATIC void microbit_i2c_free_default(void) {
    microbit_i2c_pin_free(&microbit_p20_obj);
    microbit_i2c_pin_free(&microbit_p19_obj);
}

STATIC void microbit_i2c_reset(microbit_i2c_obj_t *self) {
    self->i2c->set_pins((PinName)pin_sda->name, (PinName)pin_scl->name);

    self->i2c->frequency(frequency);

    // Call underlying mbed i2c_reset to reconfigure the pins and reset the peripheral
    i2c_reset(self->i2c->get_i2c_obj());
}

STATIC void microbit_i2c_acquire(microbit_i2c_obj_t *self) {
    // i2c requires resetting if either the accelerometer has changed the i2c configuration,
    // or if the pins being used have been used for something else (are in a different mode).
    bool i2c_changed = accelerometer_in_use && !i2c_matches_accel();
    accelerometer_in_use = false;
    i2c_in_use = true;
    if (i2c_changed) {
        // accelerometer no longer using pins 19 and 20, so free them.
        microbit_i2c_free_default();
    }
    bool acquire_sda = microbit_obj_pin_acquire((const microbit_pin_obj_t *)pin_sda, microbit_pin_mode_i2c);
    bool acquire_scl = microbit_obj_pin_acquire((const microbit_pin_obj_t *)pin_scl, microbit_pin_mode_i2c);
    if (i2c_changed || acquire_sda || acquire_scl) {
        microbit_i2c_reset(self);
    }
}

STATIC mp_obj_t microbit_i2c_init(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_freq, MP_ARG_INT, {.u_int = 100000} },
        { MP_QSTR_sda, MP_ARG_OBJ, {.u_obj = mp_const_none } },
        { MP_QSTR_scl, MP_ARG_OBJ, {.u_obj = mp_const_none } },
    };

    // parse args
    microbit_i2c_obj_t *self = (microbit_i2c_obj_t*)pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    microbit_i2c_free_current();
    microbit_i2c_free_default();

    pin_sda = &microbit_p20_obj;
    pin_scl = &microbit_p19_obj;

    if (args[1].u_obj != mp_const_none) {
        pin_sda = microbit_obj_get_pin(args[1].u_obj);
    }
    if (args[2].u_obj != mp_const_none) {
        pin_scl = microbit_obj_get_pin(args[2].u_obj);
    }

    frequency = args[0].u_int;

    microbit_i2c_acquire(self);

    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_i2c_init_obj, 1, microbit_i2c_init);

// Probe the given I2C address with an empty write to see if a device responds with an ACK
STATIC bool i2c_probe(i2c_t *obj, uint8_t address) {
    obj->i2c->ADDRESS = address;
    obj->i2c->SHORTS = 0;
    obj->i2c->TASKS_STARTTX = 1;
    obj->i2c->EVENTS_STOPPED = 0;
    obj->i2c->TASKS_STOP = 1;
    uint32_t timeout = 10000;
    while (!obj->i2c->EVENTS_STOPPED && timeout > 0) {
        --timeout;
    }
    bool ack = timeout > 0 && obj->i2c->ERRORSRC == 0;
    i2c_reset(obj);
    return ack;
}

STATIC mp_obj_t microbit_i2c_scan(mp_obj_t self_in) {
    microbit_i2c_obj_t *self = (microbit_i2c_obj_t*)MP_OBJ_TO_PTR(self_in);
    microbit_i2c_acquire(self);
    i2c_t *obj = self->i2c->get_i2c_obj();
    mp_obj_t list = mp_obj_new_list(0, NULL);
    // 7-bit addresses 0b0000xxx and 0b1111xxx are reserved
    for (int addr = 0x08; addr < 0x78; ++addr) {
        if (i2c_probe(obj, addr)) {
            mp_obj_list_append(list, MP_OBJ_NEW_SMALL_INT(addr));
        }
    }
    return list;
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_i2c_scan_obj, microbit_i2c_scan);

STATIC mp_obj_t microbit_i2c_read(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_n,        MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_repeat,   MP_ARG_BOOL, {.u_bool = false} },
    };

    // parse args
    microbit_i2c_obj_t *self = (microbit_i2c_obj_t*)pos_args[0];
    microbit_i2c_acquire(self);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // do the I2C read
    vstr_t vstr;
    vstr_init_len(&vstr, args[1].u_int);
    int err = self->i2c->read(args[0].u_int << 1, vstr.buf, vstr.len, args[2].u_bool);
    if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C read error %d", err));
    }
    // return bytes object with read data
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_i2c_read_obj, 1, microbit_i2c_read);

STATIC mp_obj_t microbit_i2c_write(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_addr,     MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buf,      MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_repeat,   MP_ARG_BOOL, {.u_bool = false} },
    };

    // parse args
    microbit_i2c_obj_t *self = (microbit_i2c_obj_t*)pos_args[0];
    microbit_i2c_acquire(self);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // do the I2C write
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1].u_obj, &bufinfo, MP_BUFFER_READ);
    int err = self->i2c->write(args[0].u_int << 1, (char*)bufinfo.buf, bufinfo.len, args[2].u_bool);
    if (err != MICROBIT_OK) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "I2C write error %d", err));
    }
    return mp_const_none;

}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_i2c_write_obj, 1, microbit_i2c_write);

STATIC const mp_map_elem_t microbit_i2c_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_init), (mp_obj_t)&microbit_i2c_init_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_scan), (mp_obj_t)&microbit_i2c_scan_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read), (mp_obj_t)&microbit_i2c_read_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write), (mp_obj_t)&microbit_i2c_write_obj },
};

STATIC MP_DEFINE_CONST_DICT(microbit_i2c_locals_dict, microbit_i2c_locals_dict_table);

const mp_obj_type_t microbit_i2c_type = {
    { &mp_type_type },
    .name = MP_QSTR_MicroBitI2C,
    .print = NULL,
    .make_new = NULL,
    .call = NULL,
    .unary_op = NULL,
    .binary_op = NULL,
    .attr = NULL,
    .subscr = NULL,
    .getiter = NULL,
    .iternext = NULL,
    .buffer_p = {NULL},
    .protocol = NULL,
    .parent = NULL,
    .locals_dict = (mp_obj_dict_t*)&microbit_i2c_locals_dict,
};

const microbit_i2c_obj_t microbit_i2c_obj = {
    {&microbit_i2c_type},
    .i2c = &ubit_i2c,
};

}
