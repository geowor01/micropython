#ifndef MICROPY_SIMULATOR_ASSERT
#define MICROPY_SIMULATOR_ASSERT

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void mp_assert(bool x);

#ifdef __cplusplus
}
#endif

#endif