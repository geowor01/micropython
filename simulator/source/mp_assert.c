#include "mp_assert.h"
#include "emscripten.h"
#include <stdio.h>

void mp_assert(bool x) {
    if (!x) {
        printf("Internal MicroPython error.\r\nSimulator will reset in 10 seconds.\r\n");
        EM_ASM({ console.log("Internal MicroPython error. Simulator will reset in 10 seconds."); });
        emscripten_sleep(10000);
        EM_ASM({ location.reload(false); });
        emscripten_sleep(10000);
    }
}