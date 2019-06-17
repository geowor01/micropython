#include "lib/ticker.h"
#include "lib/pwm.h"
#include "microbit/memory.h"
#include "microbit/filesystem.h"
#include "microbit/microbitdal.h"
// #include "MicroBitButton.h"
#include "ManagedString.h"

// Global instances of the mbed/DAL components that we use
gpio_t reset_button_gpio;
gpio_irq_t reset_button_gpio_irq;
// MicroBitDisplay ubit_display;
// MicroPythonI2C ubit_i2c(I2C_SDA0, I2C_SCL0);

// // Global pointers to instances of DAL components that are created dynamically
MicroBitAccelerometer *ubit_accelerometer;
// MicroBitCompass *ubit_compass;
// MicroBitCompassCalibrator *ubit_compass_calibrator;

extern "C" {

#include "py/stackctrl.h"
#include "py/gc.h"
#include "py/compile.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/rootstack.h"
#include "lib/mp-readline/readline.h"
#include "lib/utils/pyexec.h"
#include "microbit/modmicrobit.h"
// #include "microbit/modmusic.h"

void reset_button_handler(uint32_t data, gpio_irq_event event) {
    (void)data;
    if (event == IRQ_FALL) {
        microbit_reset();
    }
}

void microbit_ticker(void) {
    // // Update compass if it is calibrating, but not if it is still
    // // updating as compass.idleTick() is not reentrant.
    // if (ubit_compass->isCalibrating() && !compass_updating) {
    //     ubit_compass->idleTick();
    // }

    // compass_up_to_date = false;
    accelerometer_up_to_date = false;

    // Update buttons and pins with touch.
    microbit_button_tick();

    // Update the display.
    microbit_display_tick();

    // // Update the music
    // microbit_music_tick();
}

static void microbit_display_exception(mp_obj_t exc_in) {
    mp_uint_t n, *values;
    mp_obj_exception_get_traceback(exc_in, &n, &values);
    if (1) {
        vstr_t vstr;
        mp_print_t print;
        if (vstr_init_print(&vstr, 50, &print)) {
            return;
        }
        #if MICROPY_ENABLE_SOURCE_LINE
        if (n >= 3) {
            mp_printf(&print, "line %u ", values[1]);
        }
        #endif
        if (mp_obj_is_native_exception_instance(exc_in)) {
            mp_obj_exception_t *exc = (mp_obj_exception_t*)MP_OBJ_TO_PTR(exc_in);
            mp_printf(&print, "%q ", exc->base.type->name);
            if (exc->args != NULL && exc->args->len != 0) {
                mp_obj_print_helper(&print, exc->args->items[0], PRINT_STR);
            }
        }
        // Allow ctrl-C to stop the scrolling message
        mp_hal_set_interrupt_char(CHAR_CTRL_C);
        mp_hal_display_string(vstr_null_terminated_str(&vstr));
        vstr_clear(&vstr);
        mp_hal_set_interrupt_char(-1);
        // This is a variant of mp_handle_pending that swallows exceptions
        #if MICROPY_ENABLE_SCHEDULER
        #error Scheduler currently unsupported
        #endif
        if (MP_STATE_VM(mp_pending_exception) != MP_OBJ_NULL) {
            MP_STATE_VM(mp_pending_exception) = MP_OBJ_NULL;
        }
    }
}

static void do_lexer(mp_lexer_t *lex) {
    if (lex == NULL) {
        printf("MemoryError: lexer could not allocate memory\n");
        return;
    }

    {
        m_rs_push_barrier();
        m_rs_push_ptr(lex);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, MP_PARSE_FILE_INPUT);
        m_rs_assert(parse_tree.chunk);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, false);
        if (module_fun != NULL) {
            mp_hal_set_interrupt_char(3); // allow ctrl-C to interrupt us
            mp_call_function_0(module_fun);
            mp_hal_set_interrupt_char(-1);
        }
        m_rs_clear_to_barrier();
        if (MP_STATE_THREAD(cur_exc) != NULL) {
            m_rs_push_barrier();
            // uncaught exception
            mp_hal_set_interrupt_char(-1); // disable interrupt

            // print exception to stdout
            mp_obj_print_exception(&mp_plat_print, MP_STATE_THREAD(cur_exc));

            // print exception to the display, but not if it's SystemExit or KeyboardInterrupt
            mp_obj_type_t *exc_type = mp_obj_get_type(MP_STATE_THREAD(cur_exc));
            if (!mp_obj_is_subclass_fast(exc_type, &mp_type_SystemExit)
                && !mp_obj_is_subclass_fast(exc_type, &mp_type_KeyboardInterrupt)) {
                microbit_display_exception(MP_STATE_THREAD(cur_exc));
            }
            MP_STATE_THREAD(cur_exc) = NULL;
            m_rs_clear_to_barrier();
        }
    }
}

static void do_strn(const char *src, size_t len) {
    m_rs_push_barrier();
    mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR___main__, src, len, 0);
    do_lexer(lex);
    m_rs_clear_to_barrier();
}

static void do_file(file_descriptor_obj *fd) {
    m_rs_push_barrier();
    mp_lexer_t *lex = microbit_file_lexer(MP_QSTR___main__, fd);
    do_lexer(lex);
    m_rs_clear_to_barrier();
}

typedef struct _appended_script_t {
    byte header[2]; // should be "MP"
    uint16_t len; // length of script stored little endian
    char str[]; // data of script
} appended_script_t;

#define APPENDED_SCRIPT ((appended_script_t*)microbit_mp_appended_script())

static volatile bool script_set = false;

EMSCRIPTEN_KEEPALIVE
extern void set_script(char *str)
{
    strcpy(APPENDED_SCRIPT->str, str);
    APPENDED_SCRIPT->len = strlen(str);
    script_set = true;
}

#ifdef MBED_CONF_APP_TEST

static uint32_t NUMBER_OF_TESTS;
static volatile bool run_all_tests = false;
static volatile bool running_test = false;
static int test_counter = 0;

EMSCRIPTEN_KEEPALIVE
extern void set_run_all_tests()
{
    run_all_tests = true;
    running_test = true;
    test_counter = 0;
}

EMSCRIPTEN_KEEPALIVE
extern void set_running_test()
{
    running_test = true;
}

static void setup_tests()
{
    NUMBER_OF_TESTS = EM_ASM_INT({
        MicroPythonTests = {};
        MicroPythonTests.currently_running = "";
        MicroPythonTests.failed_tests = [];

        MicroPythonTests.run_all = function() {
            ccall('set_run_all_tests', 'null');
            console.log("Reset MicroPython to start run.")
        };

        MicroPythonTests.run = function(test) {
            MicroPythonTests.currently_running = test;
            var request = new XMLHttpRequest();
            request.onreadystatechange = function() {
                if (request.readyState == 4 && request.status == 200) {
                    ccall('set_script', 'null',['string'], [request.responseText]);
                    ccall('set_running_test', 'null');
                    console.log("Test: " + test);
                }
            };
            request.open("GET", test + '.py', true);
            request.send(null);
            window.MbedJSHal.serial.write("\\r\\n\\nRunning Test <" + test + "> on reboot\\r\\n\\n");
        };

        MicroPythonTests.run_by_index = function(index) {
            MicroPythonTests.run(window.MbedJSUI.MicroPythonTestList[index]);
        };

        MicroPythonTests.log_result = function() {
            current_line_y = terminal.buffer.ybase + terminal.buffer.y;
            last_line = terminal.buffer.translateBufferLineToString(current_line_y - 1);
            if (!last_line.includes("TEST DONE")) {
                setTimeout(MicroPythonTests.log_result, 10);
                return;
            }

            last_line = terminal.buffer.translateBufferLineToString(current_line_y - 2);
            passed = last_line.includes("PASS");
            result = "";
            if (last_line.includes("SystemExit") || last_line.includes("Error") || last_line.includes("Exception")) {
                lines = [last_line];
                for (i = 3; !last_line.includes("Traceback"); i++) {
                    last_line = terminal.buffer.translateBufferLineToString(current_line_y - i);
                    lines.push(last_line);
                }
                last_line = terminal.buffer.translateBufferLineToString(current_line_y - i);
                if (last_line.includes("SKIP") || last_line.includes("FAIL")) {
                    lines.push(last_line);
                }
                for (i = lines.length - 1; i >= 0; i--) {
                    result += '\n\t' + lines[i];
                }
            }
            else {
                result = '\t' + last_line;
            }

            console.log(result);
            if (!passed) {
                MicroPythonTests.failed_tests.push([MicroPythonTests.currently_running, result]);
            }
            MicroPythonTests.currently_running = "";
        };

        return window.MbedJSUI.MicroPythonTestList.length;
    });
    mp_hal_delay_ms(1);
}

static void set_test(uint32_t test_index)
{
    mp_hal_stdout_tx_str("\r\n");
    EM_ASM_({
        MicroPythonTests.run_by_index($0);
    }, test_index);
    mp_hal_stdout_tx_str("\r\n");
    script_set = false;
    while (!script_set) {
        mp_hal_delay_ms(100);
    }
}

static void log_test_result() {
    mp_hal_stdout_tx_str("TEST DONE\r\n");
    EM_ASM({
        MicroPythonTests.log_result();
    });
    while (EM_ASM_INT({ return MicroPythonTests.currently_running == "" ? 0 : 1; })) {
        mp_hal_delay_ms(100);
    };
}

#endif

int main(void) {

    APPENDED_SCRIPT->header[0] = 'M';
    APPENDED_SCRIPT->header[1] = 'P';
#ifdef MBED_CONF_APP_TEST
    setup_tests();
#if MBED_CONF_APP_TEST > 1
    set_run_all_tests();
#endif
reset:
#endif
    // // Configure the soft reset button
    gpio_init_in(&reset_button_gpio, MICROBIT_PIN_BUTTON_RESET);
    gpio_mode(&reset_button_gpio, PullUp);
    gpio_irq_init(&reset_button_gpio_irq, MICROBIT_PIN_BUTTON_RESET, &reset_button_handler, 1 /* dummy, must be non-zero */);
    gpio_irq_set(&reset_button_gpio_irq, IRQ_FALL, 1);

    // Create dynamically-allocated DAL components
    ubit_accelerometer = &MicroBitAccelerometer::autoDetect();
    // ubit_compass = &MicroBitCompass::autoDetect(ubit_i2c);
    // ubit_compass_calibrator = new MicroBitCompassCalibrator(*ubit_compass, *ubit_accelerometer, ubit_display);

    for (;;) {

#ifdef MBED_CONF_APP_TEST
        if (run_all_tests) {
            set_test(test_counter);
            test_counter++;
            if (test_counter >= NUMBER_OF_TESTS) {
                run_all_tests = false;
            }
        }
        else if (!running_test)
#endif
        {
            EM_ASM({
                ccall('set_script', 'null',['string'], [window.document.getElementById("script").value]);
                window.MbedJSUI.MicrobitDisplay.prototype.micropython_mode();
            });
        }

        static uint32_t mp_heap[10240 / sizeof(uint32_t)];

        // Initialise memory regions: stack and MicroPython heap
        mp_stack_ctrl_init();
        // Simulate the stack space already taken up
        mp_stack_set_top(MP_STATE_THREAD(stack_top) - MBED_CONF_APP_MICROBIT_STARTING_STACK_USAGE);
        mp_stack_set_limit((1800 / sizeof(uint32_t)) * sizeof(uint32_t)); // stack is 2k

        gc_init(mp_heap, (uint8_t*)mp_heap + sizeof(mp_heap));

        // Initialise the MicroPython runtime
        mp_init();
        mp_hal_init();
        readline_init0();

        // Initialise the micro:bit peripherals
        microbit_seed_random();
        // ubit_display.disable();
        microbit_display_init();
        assert(microbit_filesystem_init() == 0);
        microbit_pin_init();
        // microbit_compass_init();
        pwm_init();
        // MP_STATE_PORT(radio_buf) = NULL;

        // Start our ticker
        // Note that the DAL has a separate ticker which is also running
        ticker_init(microbit_ticker);
        ticker_start();
        pwm_start();

        // Only run initial script (or import from microbit) if we are in "friendly REPL"
        // mode.  If we are in "raw REPL" mode then this will be skipped.
        if (pyexec_mode_kind == PYEXEC_MODE_FRIENDLY_REPL) {
            file_descriptor_obj *main_module;
            if ((main_module = microbit_file_open("main.py", 7, false, false))) {
                do_file(main_module);
            } else if (APPENDED_SCRIPT->header[0] == 'M' && APPENDED_SCRIPT->header[1] == 'P' && APPENDED_SCRIPT->len > 0) {
                // run appended script
                do_strn(APPENDED_SCRIPT->str, APPENDED_SCRIPT->len);
            } else {
                // from microbit import *
                mp_import_all(mp_import_name(MP_QSTR_microbit, mp_const_empty_tuple, MP_OBJ_NEW_SMALL_INT(0)));
            }
        }
#ifdef MBED_CONF_APP_TEST
        if (running_test) {
            log_test_result();
            if (!run_all_tests)
                running_test = false;
        }
        else
#endif
        {
            // Run the REPL until the user wants to exit
            for (;;) {
                if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
                    if (pyexec_raw_repl() != 0) {
                        break;
                    }
                } else {
                    if (pyexec_friendly_repl() != 0) {
                        break;
                    }
                }
            }
        }

        // Print the special string for pyboard.py to detect the soft reset
        mp_hal_stdout_tx_str("soft reboot\r\n");
        mp_hal_delay_ms(1);

        // Stop the ticker to prevent any background tasks from running
        ticker_stop();

        // Reset state associated with background tasks
        memset(&MP_STATE_PORT(async_data)[0], 0, sizeof(MP_STATE_PORT(async_data)));
        MP_STATE_PORT(audio_buffer) = NULL;
        MP_STATE_PORT(music_data) = NULL;
#ifdef MBED_CONF_APP_TEST
        goto reset;
#endif
    }
}

mp_import_stat_t mp_import_stat(const char *path) {
    if (microbit_find_file(path, strlen(path)) != FILE_NOT_FOUND) {
        return MP_IMPORT_STAT_FILE;
    }
    return MP_IMPORT_STAT_NO_EXIST;
}

// We need to override this function so that the linker does not pull in
// unnecessary code and static RAM usage for unused system exit functionality.
// There can be large static data structures to store the exit functions.
void __register_exitproc() {
}

}

void gc_collect(void) {
    void *stack_top = MP_STATE_THREAD(stack_top) + MBED_CONF_APP_MICROBIT_STARTING_STACK_USAGE;

    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    int dummy;
    gc_collect_start();
    gc_collect_root((void **)stack_top, ((mp_uint_t)(void*)(&dummy + 1) - (mp_uint_t)stack_top) / sizeof(mp_uint_t));
    gc_collect_end();
}
