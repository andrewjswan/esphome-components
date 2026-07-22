#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::music_leds {

enum State : uint8_t { STOPPED = 0, STARTING, RUNNING, STOPPING };

// Global DSP Constants (Optimized Pipeline Execution Parameters)
static const size_t SAMPLES_FFT = 512;                   // Number of samples in an FFT batch (Must be a power of 2)
static const size_t RING_BUFFER_SIZE = SAMPLES_FFT * 4;  // Lock-free safe ring buffer allocation capacity
static const size_t HOP_SIZE = SAMPLES_FFT / 4;          // 75% sliding block overlap stride for temporal fluidity

}  // namespace esphome::music_leds
