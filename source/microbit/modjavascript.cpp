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

#include <stdio.h>
#include <string.h>

extern "C" {

#include "py/obj.h"
#include "py/objmodule.h"
#include "py/runtime.h"
#ifdef TARGET_SIMULATOR
#include "emscripten.h"
#endif

STATIC mp_obj_t new_module(mp_obj_t module_name) {
    mp_obj_new_module(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(module_name)));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(new_module_obj, new_module);

STATIC mp_obj_t new_var(mp_obj_t module_name, mp_obj_t variable_name, mp_obj_t variable_value) {
    mp_obj_t module_obj = mp_module_get(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(module_name)));
    mp_obj_dict_store(mp_obj_module_get_globals(module_obj), MP_OBJ_NEW_QSTR(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(variable_name))), MP_OBJ_NEW_QSTR(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(variable_value))));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(new_var_obj, new_var);

STATIC mp_obj_t call_js_fun(const char *fun_call_name) {
    EM_ASM_({ executeFunctionByName(UTF8ToString($0), window); }, fun_call_name);
    return mp_const_none;
}

STATIC mp_obj_t new_fun(mp_obj_t module_name, mp_obj_t fun_name, mp_obj_t fun_call_name) {
    mp_obj_t module_obj = mp_module_get(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(module_name)));
    MP_DEFINE_CONST_FUN_OBJ_JS(fun_call_obj, call_js_fun, mp_obj_str_get_str(fun_call_name));
    mp_obj_dict_store(mp_obj_module_get_globals(module_obj), MP_OBJ_NEW_QSTR(QSTR_FROM_STR_STATIC(mp_obj_str_get_str(fun_name))), (mp_obj_t)&fun_call_obj);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(new_fun_obj, new_fun);

typedef struct _js_function_obj_t {
    mp_obj_base_t base;
    const char *fun_call_name;
} js_function_obj_t;

extern const mp_obj_type_t js_function_type;

mp_obj_t js_function_make_new( const mp_obj_type_t *type,
                                  size_t n_args,
                                  size_t n_kw,
                                  const mp_obj_t *args ) {
    mp_arg_check_num(n_args, n_kw, 1, 1, true);
    js_function_obj_t *self = m_new_obj(js_function_obj_t);
    self->base.type = &js_function_type;
    self->fun_call_name = mp_obj_str_get_str(args[0]);
    return MP_OBJ_FROM_PTR(self);
}

STATIC void js_function_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    js_function_obj_t *self = (js_function_obj_t*)MP_OBJ_TO_PTR(self_in);
    mp_printf(&mp_plat_print, "Javascript function <%s>", self->fun_call_name);
}

STATIC mp_obj_t call_js_method(mp_obj_t self_in) {
    js_function_obj_t *self = (js_function_obj_t*)MP_OBJ_TO_PTR(self_in);
    EM_ASM_({ executeFunctionByName(UTF8ToString($0), window); }, self->fun_call_name);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(call_js_obj, call_js_method);

STATIC const mp_rom_map_elem_t js_function_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_call), MP_ROM_PTR(&call_js_obj) },
};

STATIC MP_DEFINE_CONST_DICT(js_function_locals, js_function_locals_table);

const mp_obj_type_t js_function_type = {
    { &mp_type_type },
    .name = MP_QSTR_js_function_type,
    .print = js_function_print,
    .make_new = js_function_make_new,
    .locals_dict = (mp_obj_dict_t*)&js_function_locals,
};

STATIC const mp_map_elem_t javascript_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_javascript) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_new_var), (mp_obj_t)&new_var_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_new_module), (mp_obj_t)&new_module_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_new_function), (mp_obj_t)&new_fun_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_function), (mp_obj_t)&js_function_type },
};

STATIC MP_DEFINE_CONST_DICT(javascript_module_globals, javascript_module_globals_table);

const mp_obj_module_t javascript_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&javascript_module_globals,
};

}
