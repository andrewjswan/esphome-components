#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::music_leds {

// Mathematical constant defined as (e - 1) used for exponential volume scaling and psychoacoustic normalization
inline constexpr float E_MINUS_ONE = 1.7182818f;

// 16-bit amplitude scale for DSP
inline constexpr float AMPLITUDE_SCALE_16BIT = 32768.0f;

// clang-format off

// Global DSP Constants (Optimized Pipeline Execution Parameters)
inline constexpr size_t SAMPLES_FFT = 512;                   // Number of samples in an FFT batch (Must be a power of 2)
inline constexpr size_t MAX_VALID_BIN = SAMPLES_FFT / 2;     // Maximum valid spectral line index derived from the Nyquist sampling limit
inline constexpr size_t HOP_SIZE = SAMPLES_FFT / 4;          // 75% sliding block overlap stride for temporal fluidity
inline constexpr size_t RING_BUFFER_SIZE = SAMPLES_FFT * 4;  // Lock-free safe ring buffer allocation capacity
inline constexpr size_t NUM_GEQ_CHANNELS = 16;               // Number of frequency channels

// clang-format on

// GEQ Processor constants

// 17 frequency boundaries (in Hz) defining 16 bands.
// Standard log-scale frequency map boundaries used for multi-band audio analysis
inline constexpr float BAND_FREQ_BOUNDARIES[] = {
    43.06640625f,    // bin 1   — start of band 0
    129.19921875f,   // bin 3   — start of band 1
    215.33203125f,   // bin 5   — start of band 2
    344.53125f,      // bin 8   — start of band 3
    473.73046875f,   // bin 11  — start of band 4
    645.99609375f,   // bin 15  — start of band 5
    861.328125f,     // bin 20  — start of band 6
    1162.79296875f,  // bin 27  — start of band 7
    1550.390625f,    // bin 36  — start of band 8
    2024.12109375f,  // bin 47  — start of band 9
    2627.05078125f,  // bin 61  — start of band 10
    3402.24609375f,  // bin 79  — start of band 11
    4392.7734375f,   // bin 102 — start of band 12
    5641.69921875f,  // bin 131 — start of band 13
    7278.22265625f,  // bin 169 — start of band 14
    9388.4765625f,   // bin 218 — start of band 15
    11025.0f,        // bin 256 — end of band 15 (Nyquist at 22050/512)
};

// Fixed wrapper structure to allow raw C-style array returns from constexpr context
struct PinkNoiseBuffer {
  float data[16];

  /**
   * @brief Overloaded array subscript operator to maintain full backward compatibility.
   * @note Allows objects of this struct to be used with standard [index] syntax.
   */
  constexpr float operator[](size_t index) const { return data[index]; }

  // Non-const version required for compile-time buffer mutation steps
  constexpr float &operator[](size_t index) { return data[index]; }
};

// Hardcoded native multiplication table to flatten the microphone frequency response (Pink Noise Equalization)
inline constexpr PinkNoiseBuffer PINK_NOISE_CURVE = {
    {1.70f, 1.71f, 1.73f, 1.78f, 1.68f, 1.56f, 1.55f, 1.63f, 1.79f, 1.62f, 1.80f, 2.06f, 2.47f, 3.35f, 6.83f, 9.55f}};

constexpr PinkNoiseBuffer generate_normalized_pink_noise() {
  PinkNoiseBuffer normalized_array = {};

  // Dynamically divide each element by the sub-bass anchor coefficient (1.70f)
  // to enforce a true 1.0f baseline floor for the Bass macro band.
  for (size_t i = 0; i < 16; ++i) {
    normalized_array[i] = PINK_NOISE_CURVE.data[i] / 1.70f;
  }
  return normalized_array;
}

// Normalized Pink Noise Compensation Table (Generated completely at compile-time!)
inline constexpr PinkNoiseBuffer PINK_NOISE_CURVE_NORM = generate_normalized_pink_noise();

}  // namespace esphome::music_leds
