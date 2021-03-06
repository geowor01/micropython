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

#include <string.h>
#include "nrf_gpio.h"
#include "MicroBitDisplay.h"
#include "MicroBitLightSensor.h"

extern "C" {

#include "py/mphal.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/objstr.h"
#include "lib/iters.h"
#include "lib/ticker.h"
#include "microbit/modmicrobit.h"
#include "microbit/microbit_image.h"

#define ASYNC_MODE_STOPPED 0
#define ASYNC_MODE_ANIMATION 1
#define ASYNC_MODE_CLEAR 2

typedef struct _microbit_display_obj_t {
    mp_obj_base_t base;
    uint8_t image_buffer[5][5];
    uint8_t previous_brightness;
    bool    active;
    /* Current row for strobing */
    uint8_t strobe_row;
    /* boolean histogram of brightness in buffer */
    uint16_t brightnesses;
    uint16_t pins_for_brightness[MAX_BRIGHTNESS+1];

    void advanceRow();
    inline void setPinsForRow(uint8_t brightness);
} microbit_display_obj_t;

void microbit_display_show(microbit_display_obj_t *display, microbit_image_obj_t *image) {
    mp_int_t w = MIN(image->width(), 5);
    mp_int_t h = MIN(image->height(), 5);
    mp_int_t x = 0;
    mp_int_t brightnesses = 0;
    for (; x < w; ++x) {
        mp_int_t y = 0;
        for (; y < h; ++y) {
            uint8_t pix = image->getPixelValue(x, y);
            display->image_buffer[x][y] = pix;
            brightnesses |= (1 << pix);
        }
        for (; y < 5; ++y) {
            display->image_buffer[x][y] = 0;
        }
    }
    for (; x < 5; ++x) {
        for (mp_int_t y = 0; y < 5; ++y) {
            display->image_buffer[x][y] = 0;
        }
    }
    display->brightnesses = brightnesses;
}

#define DEFAULT_PRINT_SPEED 400


mp_obj_t microbit_display_show_func(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {

    // Cancel any animations.
    MP_STATE_PORT(async_data)[0] = NULL;
    MP_STATE_PORT(async_data)[1] = NULL;

    static const mp_arg_t show_allowed_args[] = {
        { MP_QSTR_image,    MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_delay,    MP_ARG_INT, {.u_int = DEFAULT_PRINT_SPEED} },
        { MP_QSTR_clear,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_wait,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_loop,     MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    // Parse the args.
    microbit_display_obj_t *self = (microbit_display_obj_t*)pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(show_allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(show_allowed_args), show_allowed_args, args);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)

    mp_obj_t image = args[0].u_obj;
    mp_int_t delay = args[1].u_int;
    bool clear = args[2].u_bool;
    bool wait = args[3].u_bool;
    bool loop = args[4].u_bool;

    // Convert to string from an integer or float if applicable
    if (mp_obj_is_integer(image) || mp_obj_is_float(image)) {
        image = mp_obj_str_make_new(&mp_type_str, 1, 0, &image);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    }

    if (MP_OBJ_IS_STR(image)) {
        // arg is a string object
        mp_uint_t len;
        const char *str = mp_obj_str_get_data(image, &len);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
        if (len == 0) {
            // There are no chars; do nothing.
            return mp_const_none;
        } else if (len == 1) {
            if (!clear && !loop) {
                // A single char; convert to an image and print that.
                image = microbit_image_for_char(str[0]);
                goto single_image_immediate;
            }
        }
        image = microbit_string_facade(image);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    } else if (mp_obj_get_type(image) == &microbit_image_type) {
        if (!clear && !loop) {
            goto single_image_immediate;
        }
        image = mp_obj_new_tuple(1, &image);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    }

    // iterable:
    if (args[4].u_bool) { /*loop*/
        image = microbit_repeat_iterator(image);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    }
    microbit_display_animate(self, image, delay, clear, wait);
    return mp_const_none;

single_image_immediate:
    microbit_display_show(self, (microbit_image_obj_t *)image);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_display_show_obj, 1, microbit_display_show_func);

static uint8_t async_mode;
static mp_obj_t async_iterator = NULL;
// Record if an error occurs in async animation. Unfortunately there is no way to report this.
static volatile bool wakeup_event = false;
static mp_uint_t async_delay = 1000;
static mp_uint_t async_tick = 0;
static bool async_clear = false;

STATIC void async_stop(void) {
    async_iterator = NULL;
    async_mode = ASYNC_MODE_STOPPED;
    async_tick = 0;
    async_delay = 1000;
    async_clear = false;
    MP_STATE_PORT(async_data)[0] = NULL;
    MP_STATE_PORT(async_data)[1] = NULL;
    wakeup_event = true;
}

STATIC void wait_for_event() {
    while (!wakeup_event) {
        // allow CTRL-C to stop the animation
        if (MP_STATE_VM(mp_pending_exception) != MP_OBJ_NULL) {
            async_stop();
            return;
        }
        if (perform_reset) {
            return;
        }
        emscripten_sleep(10);
    }
    wakeup_event = false;
}

struct DisplayPoint {
    uint8_t x;
    uint8_t y;
};

#define NO_CONN 0

#define ROW_COUNT 3
#define COLUMN_COUNT 9

static const DisplayPoint display_map[COLUMN_COUNT][ROW_COUNT] = {
    {{0,0}, {4,2}, {2,4}},
    {{2,0}, {0,2}, {4,4}},
    {{4,0}, {2,2}, {0,4}},
    {{4,3}, {1,0}, {0,1}},
    {{3,3}, {3,0}, {1,1}},
    {{2,3}, {3,4}, {2,1}},
    {{1,3}, {1,4}, {3,1}},
    {{0,3}, {NO_CONN,NO_CONN}, {4,1}},
    {{1,2}, {NO_CONN,NO_CONN}, {3,2}}
};

#define MIN_COLUMN_PIN 4
#define COLUMN_PINS_MASK 0x1ff0
#define MIN_ROW_PIN 13
#define MAX_ROW_PIN 15
#define ROW_PINS_MASK 0xe000

inline void microbit_display_obj_t::setPinsForRow(uint8_t brightness) {
    if (brightness == 0) {
        nrf_gpio_pins_clear(COLUMN_PINS_MASK & ~this->pins_for_brightness[brightness]);
    } else {
        nrf_gpio_pins_set(this->pins_for_brightness[brightness]);
    }
}

/* This is the primary PWM driver/display driver.  It will operate on one row
 * (9 pins) per invocation.  It will turn on LEDs with maximum brightness,
 * then let the "callback" callback turn off the LEDs as appropriate for the
 * required brightness level.
 *
 * For each row
 *   Turn off all the LEDs in the previous row
 *     Set the column bits high (off)
 *     Set the row strobe low (off)
 *   Turn on all the LEDs in the current row that have maximum brightness
 *     Set the row strobe high (on)
 *     Set some/all column bits low (on)
 *   Register the PWM callback
 *   For each callback start with brightness 0
 *     If brightness 0
 *       Turn off the LEDs specified at this level
 *     Else
 *       Turn on the LEDs specified at this level
 *     If brightness max
 *       Disable the PWM callback
 *     Else
 *       Re-queue the PWM callback after the appropriate delay
 */
void microbit_display_obj_t::advanceRow() {
    /* Clear all of the column bits */
    nrf_gpio_pins_set(COLUMN_PINS_MASK);
    /* Clear the strobe bit for this row */
    nrf_gpio_pin_clear(strobe_row+MIN_ROW_PIN);

    /* Move to the next row.  Before this, "this row" refers to the row
     * manipulated by the previous invocation of this function.  After this,
     * "this row" refers to the row manipulated by the current invocation of
     * this function. */
    strobe_row++;

    // Reset the row counts and bit mask when we have hit the max.
    if (strobe_row == ROW_COUNT) {
        strobe_row = 0;
        EM_ASM_({ window.MbedJSUI.MicrobitDisplay.prototype.set_image(new Uint8Array(Module.HEAPU8.subarray($0, $0 + 25))); }, microbit_display_obj.image_buffer);
        emscripten_sleep(1);
    }

    // Set pin for this row.
    // Prepare row for rendering.
    for (int i = 0; i <= MAX_BRIGHTNESS; i++) {
        pins_for_brightness[i] = 0;
    }
    for (int i = 0; i < COLUMN_COUNT; i++) {
        int x = display_map[i][strobe_row].x;
        int y = display_map[i][strobe_row].y;
        uint8_t brightness = microbit_display_obj.image_buffer[x][y];
        if (brightness) {
            pins_for_brightness[MAX_BRIGHTNESS] |= (1<<(i+MIN_COLUMN_PIN));
        }
    }
    /* Enable the strobe bit for this row */
    nrf_gpio_pin_set(strobe_row+MIN_ROW_PIN);
    /* Enable the column bits for all pins that need to be on. */
    nrf_gpio_pins_clear(pins_for_brightness[MAX_BRIGHTNESS]);
}

static const uint16_t render_timings[] =
// The scale is (approximately) exponential,
// each step is approx x1.9 greater than the previous.
{   0, // Bright, Ticks Duration, Relative power
    2,   //   1,   2,     32µs,     inf
    2,   //   2,   4,     64µs,     200%
    4,   //   3,   8,     128µs,    200%
    7,   //   4,   15,    240µs,    187%
    13,  //   5,   28,    448µs,    187%
    25,  //   6,   53,    848µs,    189%
    49,  //   7,   102,   1632µs,   192%
    97,  //   8,   199,   3184µs,   195%
// Always on  9,   375,   6000µs,   188%
};

#define DISPLAY_TICKER_SLOT 1

enum {
    LIGHT_SENSOR_IDLE,
    LIGHT_SENSOR_REQUEST_SAMPLE,
    LIGHT_SENSOR_TAKING_SAMPLE,
    LIGHT_SENSOR_HAVE_SAMPLE,
};

static MicroBitLightSensor *light_sensor_obj = NULL;
static volatile uint8_t light_sensor_state = LIGHT_SENSOR_IDLE;
static uint32_t light_sensor_last_reading_time = 0;

static int light_sensor_read(void) {
    // Create the light-sensor object if it doesn't yet exist
    if (light_sensor_obj == NULL) {
        light_sensor_obj = new MicroBitLightSensor(microbitMatrixMap);
    }

    // Depending on time since last call, take 1, 2 or 3 readings
    int n;
    uint32_t time = ticker_ticks_ms;
    if (time - light_sensor_last_reading_time < 50) {
        n = 1;
    } else if (time - light_sensor_last_reading_time < 100) {
        n = 2;
    } else {
        n = 3;
    }

    // Take readings so the object can average them out
    for (int i = 0; i < n; ++i) {
        light_sensor_state = LIGHT_SENSOR_REQUEST_SAMPLE;
        while (light_sensor_state != LIGHT_SENSOR_HAVE_SAMPLE) {
            if (perform_reset) {
                return 0;
            }
            emscripten_sleep(10);
        }
    }

    // Record time of last reading
    light_sensor_last_reading_time = ticker_ticks_ms;

    // Get and return the light reading
    return light_sensor_obj->read();
}

static bool light_sensor_busy(void) {
    if (light_sensor_state == LIGHT_SENSOR_TAKING_SAMPLE) {
        if (NRF_ADC->ENABLE == ADC_ENABLE_ENABLE_Enabled) {
            return true;
        }
        light_sensor_state = LIGHT_SENSOR_HAVE_SAMPLE;
    }
    return false;
}

static void light_sensor_update(void) {
    if (light_sensor_state == LIGHT_SENSOR_REQUEST_SAMPLE) {
        light_sensor_obj->startSensing(MicroBitEvent(MICROBIT_ID_DISPLAY, MICROBIT_DISPLAY_EVT_LIGHT_SENSE, CREATE_ONLY));
        light_sensor_state = LIGHT_SENSOR_TAKING_SAMPLE;
    }
}

static void draw_object(mp_obj_t obj) {
    microbit_display_obj_t *display = (microbit_display_obj_t*)MP_STATE_PORT(async_data)[0];
    if (obj == MP_OBJ_STOP_ITERATION) {
        if (async_clear) {
            microbit_display_show(&microbit_display_obj, BLANK_IMAGE);
            async_clear = false;
        } else {
            async_stop();
        }
    } else if (mp_obj_get_type(obj) == &microbit_image_type) {
        microbit_display_show(display, (microbit_image_obj_t *)obj);
    } else if (MP_OBJ_IS_STR(obj)) {
        mp_uint_t len;
        const char *str = mp_obj_str_get_data(obj, &len);
        RETURN_ON_EXCEPTION()
        if (len == 1) {
            microbit_display_show(display, microbit_image_for_char(str[0]));
        } else {
            async_stop();
        }
    } else {
        MP_STATE_VM(mp_pending_exception) = mp_obj_new_exception_msg(&mp_type_TypeError, "not an image");
        async_stop();
    }
}

static void microbit_display_update(void) {
    async_tick += MILLISECONDS_PER_MACRO_TICK;
    if (async_tick < async_delay) {
        return;
    }
    async_tick = 0;
    switch (async_mode) {
        case ASYNC_MODE_ANIMATION:
        {
            if (MP_STATE_PORT(async_data)[0] == NULL || MP_STATE_PORT(async_data)[1] == NULL) {
                async_stop();
                break;
            }
            /* WARNING: We are executing in an interrupt handler.
             * If an exception is raised here then we must hand it to the VM. */
            mp_obj_t obj;
            gc_lock();
            {
                m_rs_push_barrier();
                obj = mp_iternext_allow_raise(async_iterator);
                gc_unlock();
                m_rs_clear_to_barrier();
                if (MP_STATE_THREAD(cur_exc) != NULL) {
                    m_rs_push_barrier();
                    mp_obj_base_t *the_exc = MP_STATE_THREAD(cur_exc);
                    MP_STATE_THREAD(cur_exc) = NULL;
                    // uncaught exception
                    if (!mp_obj_is_subclass_fast(MP_OBJ_FROM_PTR(the_exc->type),
                        MP_OBJ_FROM_PTR(&mp_type_StopIteration))) {
                        // An exception other than StopIteration, so set it for the VM to raise later
                        // If memory error, write an appropriate message.
                        if (mp_obj_get_type(the_exc) == &mp_type_MemoryError) {
                            mp_printf(&mp_plat_print, "Allocation in interrupt handler");
                        }
                        MP_STATE_VM(mp_pending_exception) = MP_OBJ_FROM_PTR(the_exc);
                    }
                    obj = MP_OBJ_STOP_ITERATION;
                    m_rs_clear_to_barrier();
                }
            }
            m_rs_push_barrier();
            draw_object(obj);
            m_rs_clear_to_barrier();
            break;
        }
        case ASYNC_MODE_CLEAR:
            microbit_display_show(&microbit_display_obj, BLANK_IMAGE);
            async_stop();
            break;
    }
}

#define GREYSCALE_MASK ((1<<MAX_BRIGHTNESS)-2)

/* This is the top-level animation/display callback.  It is not a registered
 * callback. */
void microbit_display_tick(void) {
    // We can't update the display if the light sensor is sampling
    if (light_sensor_busy()) {
        return;
    }

    /* Do nothing if the display is not active. */
    if (!microbit_display_obj.active) {
        light_sensor_update();
        return;
    }

    microbit_display_obj.advanceRow();

    microbit_display_update();
    light_sensor_update();
}


void microbit_display_animate(microbit_display_obj_t *self, mp_obj_t iterable, mp_int_t delay, bool clear, bool wait) {
    // Reset the repeat state.
    MP_STATE_PORT(async_data)[0] = NULL;
    MP_STATE_PORT(async_data)[1] = NULL;
    async_iterator = mp_getiter(iterable, NULL);
    RETURN_ON_EXCEPTION()
    async_delay = delay;
    async_clear = clear;
    MP_STATE_PORT(async_data)[0] = self; // so it doesn't get GC'd
    MP_STATE_PORT(async_data)[1] = async_iterator;
    wakeup_event = false;
    mp_obj_t obj = mp_iternext_allow_raise(async_iterator);
    RETURN_ON_EXCEPTION()
    draw_object(obj);
    RETURN_ON_EXCEPTION()
    async_tick = 0;
    async_mode = ASYNC_MODE_ANIMATION;
    if (wait) {
        wait_for_event();
    }
}


// Delay in ms in between moving display one column to the left.
#define DEFAULT_SCROLL_SPEED       150

void microbit_display_scroll(microbit_display_obj_t *self, const char* str) {
    mp_obj_t iterable = scrolling_string_image_iterable(str, strlen(str), NULL, false, false);
    RETURN_ON_EXCEPTION()
    microbit_display_animate(self, iterable, DEFAULT_SCROLL_SPEED, false, true);
}


mp_obj_t microbit_display_scroll_func(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t scroll_allowed_args[] = {
        { MP_QSTR_text, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_delay, MP_ARG_INT, {.u_int = DEFAULT_SCROLL_SPEED} },
        { MP_QSTR_wait, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_monospace, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_loop, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };
    // Parse the args.
    microbit_display_obj_t *self = (microbit_display_obj_t*)pos_args[0];
    mp_arg_val_t args[MP_ARRAY_SIZE(scroll_allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(scroll_allowed_args), scroll_allowed_args, args);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    mp_uint_t len;
    mp_obj_t object_string = args[0].u_obj;
    if (mp_obj_is_integer(object_string) || mp_obj_is_float(object_string)) {
        object_string = mp_obj_str_make_new(&mp_type_str, 1, 0, &object_string);
        RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    }
    const char* str = mp_obj_str_get_data(object_string, &len);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    mp_obj_t iterable = scrolling_string_image_iterable(str, len, args[0].u_obj, args[3].u_bool /*monospace?*/, args[4].u_bool /*loop*/);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_display_animate(self, iterable, args[1].u_int /*delay*/, false/*clear*/, args[2].u_bool/*wait?*/);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(microbit_display_scroll_obj, 1, microbit_display_scroll_func);

mp_obj_t microbit_display_on_func(mp_obj_t obj) {
    microbit_display_obj_t *self = (microbit_display_obj_t*)obj;
    /* Try to reclaim the pins we need */
    microbit_obj_pin_acquire(&microbit_p3_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_acquire(&microbit_p4_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_acquire(&microbit_p6_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_acquire(&microbit_p7_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_acquire(&microbit_p9_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_acquire(&microbit_p10_obj, microbit_pin_mode_display);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    /* Make sure all pins are in the correct state */
    microbit_display_init();
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    /* Re-enable the display loop.  This will resume any animations in
     * progress and display any static image. */
    self->active = true;
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_display_on_obj, microbit_display_on_func);

mp_obj_t microbit_display_off_func(mp_obj_t obj) {
    microbit_display_obj_t *self = (microbit_display_obj_t*)obj;
    /* Disable the display loop.  This will pause any animations in progress.
     * It will not prevent a user from attempting to modify the state, but
     * modifications will not appear to have any effect until the display loop
     * is re-enabled. */
    self->active = false;
    /* Disable the row strobes, allowing the columns to be used freely for
     * GPIO. */
    nrf_gpio_pins_clear(ROW_PINS_MASK);
    /* Free pins for other uses */
    microbit_obj_pin_free(&microbit_p3_obj);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_free(&microbit_p4_obj);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_free(&microbit_p6_obj);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_free(&microbit_p7_obj);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_free(&microbit_p9_obj);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_obj_pin_free(&microbit_p10_obj);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_display_off_obj, microbit_display_off_func);

mp_obj_t microbit_display_is_on_func(mp_obj_t obj) {
    microbit_display_obj_t *self = (microbit_display_obj_t*)obj;
    if (self->active) {
        return mp_const_true;
    }
    else {
        return mp_const_false;
    }
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_display_is_on_obj, microbit_display_is_on_func);

mp_obj_t microbit_display_read_light_level(mp_obj_t obj) {
    (void)obj;
    return MP_OBJ_NEW_SMALL_INT(light_sensor_read());
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_display_read_light_level_obj, microbit_display_read_light_level);

void microbit_display_clear(void) {
    // Reset repeat state, cancel animation and clear screen.
    wakeup_event = false;
    async_mode = ASYNC_MODE_CLEAR;
    async_tick = async_delay - MILLISECONDS_PER_MACRO_TICK;
    wait_for_event();
}

mp_obj_t microbit_display_clear_func(mp_obj_t self) {
    (void)self;
    microbit_display_clear();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(microbit_display_clear_obj, microbit_display_clear_func);

void microbit_display_set_pixel(microbit_display_obj_t *display, mp_int_t x, mp_int_t y, mp_int_t bright) {
    if (x < 0 || y < 0 || x > 4 || y > 4) {
        mp_raise_ValueError_o("index out of bounds");
        return;
    }
    if (bright < 0 || bright > MAX_BRIGHTNESS) {
        mp_raise_ValueError_o("brightness out of bounds");
        return;
    }
    display->image_buffer[x][y] = bright;
    display->brightnesses |= (1 << bright);
}

STATIC mp_obj_t microbit_display_set_pixel_func(mp_uint_t n_args, const mp_obj_t *args) {
    (void)n_args;
    microbit_display_obj_t *self = (microbit_display_obj_t*)args[0];
    mp_int_t args1 = mp_obj_get_int(args[1]);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    mp_int_t args2 = mp_obj_get_int(args[2]);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    mp_int_t args3 = mp_obj_get_int(args[3]);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    microbit_display_set_pixel(self, args1, args2, args3);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(microbit_display_set_pixel_obj, 4, 4, microbit_display_set_pixel_func);

mp_int_t microbit_display_get_pixel(microbit_display_obj_t *display, mp_int_t x, mp_int_t y) {
    if (x < 0 || y < 0 || x > 4 || y > 4) {
        mp_raise_ValueError_o("index out of bounds");
        return 0;
    }
    return display->image_buffer[x][y];
}

STATIC mp_obj_t microbit_display_get_pixel_func(mp_obj_t self_in, mp_obj_t x_in, mp_obj_t y_in) {
    microbit_display_obj_t *self = (microbit_display_obj_t*)self_in;
    mp_int_t args_x = mp_obj_get_int(x_in);
    mp_int_t args_y = mp_obj_get_int(y_in);
    RETURN_ON_EXCEPTION(MP_OBJ_NULL)
    return MP_OBJ_NEW_SMALL_INT(microbit_display_get_pixel(self, args_x, args_y));
}
MP_DEFINE_CONST_FUN_OBJ_3(microbit_display_get_pixel_obj, microbit_display_get_pixel_func);

STATIC const mp_map_elem_t microbit_display_locals_dict_table[] = {

    { MP_OBJ_NEW_QSTR(MP_QSTR_get_pixel),  (mp_obj_t)&microbit_display_get_pixel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_pixel),  (mp_obj_t)&microbit_display_set_pixel_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_show), (mp_obj_t)&microbit_display_show_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_scroll), (mp_obj_t)&microbit_display_scroll_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear), (mp_obj_t)&microbit_display_clear_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_on),  (mp_obj_t)&microbit_display_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_off),  (mp_obj_t)&microbit_display_off_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_is_on),  (mp_obj_t)&microbit_display_is_on_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_read_light_level), (mp_obj_t)&microbit_display_read_light_level_obj },
};

STATIC MP_DEFINE_CONST_DICT(microbit_display_locals_dict, microbit_display_locals_dict_table);

STATIC const mp_obj_type_t microbit_display_type = {
    { &mp_type_type },
    .name = MP_QSTR_MicroBitDisplay,
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
    .locals_dict = (mp_obj_dict_t*)&microbit_display_locals_dict,
};

microbit_display_obj_t microbit_display_obj = {
    {&microbit_display_type},
    { 0 },
    .previous_brightness = 0,
    .active = 1,
    .strobe_row = 0,
    .brightnesses = 0,
    .pins_for_brightness = { 0 },
};

void microbit_display_init(void) {
    //  Set pins as output.
    nrf_gpio_range_cfg_output(MIN_COLUMN_PIN, MIN_COLUMN_PIN + COLUMN_COUNT + ROW_COUNT - 1);
}

}
