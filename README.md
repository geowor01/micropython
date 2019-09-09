MicroPython for the BBC micro:bit running as an in-browser simulator
====================================================================

This is the source code for MicroPython running on the BBC micro:bit running as an in-browser simulator!

To get involved with the micro:bit community join the Slack channel by signing up here:
https://tech.microbit.org/get-involved/where-to-find/

Various things are in this repository, including:
- Source code in source/ and inc/ directories.
- Example Python programs in the examples/ directory.
- Tools in the tools/ directory.

Introduction
============

This project is a fork of the https://github.com/bbcmicrobit/micropython project, with the oldest common ancestor being https://github.com/bbcmicrobit/micropython/commit/a92ca9b1f907c07a01116b0eb464ca4743a28bf1.

The code for MicroPython has been adapted to run on an in-browser simulation of the BBC micro:bit built using https://github.com/microbit-foundation/microbit-simulator. The result of this is used in the online MicroPython editor https://github.com/bbcmicrobit/PythonEditor to allow users to run their MicroPython scripts within the browser without having to flash their script to a physical board.

[Emscripten](https://emscripten.org/) is used to compile MicroPython to WebAssembly and Javascript, with the [Asyncify](https://emscripten.org/docs/porting/asyncify.html) option used to simulate multithreading.

Currently Supported Modules
===========================

Submodules of microbit:
- [x] reset
- [x] sleep
- [x] running_time
- [x] panic
- [x] temperature
- [x] pins
- [x] Image
- [x] display
- [x] buttons
- [x] accelerometer
- [ ] compass
- [ ] i2c
- [ ] uart
- [ ] spi

Other Modules:
- [x] love
- [x] os
- [x] time
- [x] antigravity
- [x] ucollections
- [x] array
- [x] math
- [x] random
- [x] ustruct
- [x] utime
- [x] builtins
- [x] micropython
- [x] struct
- [x] collections
- [x] sys
- [x] gc
- [x] this
- [x] radio
- [ ] machine
- [ ] audio
- [ ] speech
- [ ] music
- [ ] neopixel

Limitations
===========

- Due to the inability to pre-empt [Javascript](https://developer.mozilla.org/en-US/docs/Web/JavaScript/EventLoop) and thus an inability to reliably set a ticker to be called at a certain frequency, behaviour relying on sensitive timing may differ from hardware.
- Due to a different stack being used, the stack size will differ from that on hardware and thus stack overflow checks have been disabled.
- Inline assembly is not supported due to no emulator being used.

Major Modifications
===================

- Files containing functionality not yet supported have been removed.
- Flash memory and the heap are simulated using a memory buffer.
- The nRF hardware API cannot be used and has been reimplemented where necessary in `/simulator`.
- [EM_ASM](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html#interacting-with-code-call-javascript-from-native) is used in many places to interact with the Javascript implemented simulation of the hardware.
- Information such as the page size (MICROBIT_PAGE_SIZE) are stored in `mbed_app.json` and accessed by MBED_CONF_APP_MICROBIT_PAGE_SIZE.
- Where known MicroPython bugs exist, these failures have been caught and a safe crash performed by `crash_micropython`.
- The failure caused by assert.h has been replaced by a simulated failure performed in mp_assert.c so as to prevent the page crashing.
- The VM is regularly interrupted using `wait_ms`, which uses [`emscripten_sleep`](https://emscripten.org/docs/porting/asyncify.html) in order to yield to other javascript and the browser, ensuring responsiveness as well as preventing deadlock. This is done using the `YIELD_IF_NEEDED` macro which only yields if the time since last yield is equal to or larger than `YIELD_PERIOD`. It may be necessary to use `YIELD_IF_NEEDED` elsewhere in order to prevent blocking of the browser for possibly long running functionality that does not use the VM such as builtins.
- The hardware interrupts that act as the ticker have been simulated using Javascript [timeouts](https://www.w3schools.com/jsref/met_win_settimeout.asp). The ticker handler can be found in `simulator/source/nrf.cpp`.
- Use of [setjmp](https://en.cppreference.com/w/cpp/utility/program/setjmp) has been removed due to Asyncify's inability to handle it ["There are some known bugs with ASYNCIFY on things like exceptions and setjmp"](https://emscripten.org/docs/porting/emterpreter.html). Emscripten's emterpreter also failed to handle setjmp in certain situations. Exceptions are now handled manually, using the `RETURN_ON_EXCEPTION` macro to check whether an exception has been raised. `RETURN_ON_EXCEPTION` now needs to be called after every function call that may raise an exception, in order to replicate setjmps and not cause the program to crash.
- Due to emscripten storing some local variables in inaccessible Javascript variables, the garbage collector now also requires an explicit root pointer stack to be maintained. This means that every heap pointer created should be pushed on to the stack using `m_rs_push_ptr` while in use and popped off using `m_rs_pop_ptr` when no longer in use. Due to Javascript's single threaded, run to completion nature, pushing the pointer on to the stack is only strictly required when the garbage collector may be called while in use.
- In order to not require the correct pointers to be popped off the root stack while exceptions are being handled, popping during exception handling is ignored. In order to remove unnecessary pointers from the stack, a barrier is pushed on to the stack at the beginning of a block of code which might throw exceptions using `m_rs_push_barrier` and then all pointers remaining on the stack above the barrier at the end of the block are cleared using `m_rs_clear_to_barrier`.

Building
========

- Clone https://github.com/microbit-foundation/microbit-simulator and follow the build instructions in the README.

Alternatively, to simply run the simulator locally you can:
- Run a local copy of the [MicroPython editor](https://github.com/bbcmicrobit/PythonEditor).

Testing
=======

A testing framework has been added in `main.cpp` to allow the tests in `/tests` to be automatically run. This requires the `TEST` macro to be defined in mbed_app.json. Setting `TEST` to any value above 1 will run the tests on load, otherwise tests can be run using `MicroPythonTests.run_all()` in javascript and then pressing `Ctrl + D` in the REPL. Specific tests can be run using `MicroPythonTests.run('basics/andor')` for example.