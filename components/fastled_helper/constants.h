#pragma once

#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

#if defined(USE_ESP32)
#define ALIGN_PROGMEM(N)  __attribute__ ((aligned (N)))
#define PAL_PROGMEM       PROGMEM
#else
#define ALIGN_PROGMEM(N)
#define PAL_PROGMEM
#endif

}  // namespace esphome::fastled_helper
