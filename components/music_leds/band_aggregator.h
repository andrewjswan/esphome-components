#pragma once

#include "constants.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

/// 17 frequency boundaries (in Hz) defining 16 bands.
/// Derived from the original WLED-based bin indices at 22050 Hz / 512-point FFT (43.066 Hz/bin).
/// Band i spans [BAND_FREQ_BOUNDARIES[i], BAND_FREQ_BOUNDARIES[i+1]).
static constexpr float BAND_FREQ_BOUNDARIES[17] = {
    43.06640625f,     // bin 1   — start of band 0
    129.19921875f,    // bin 3   — start of band 1
    215.33203125f,    // bin 5   — start of band 2
    344.53125f,       // bin 8   — start of band 3
    473.73046875f,    // bin 11  — start of band 4
    645.99609375f,    // bin 15  — start of band 5
    861.328125f,      // bin 20  — start of band 6
    1162.79296875f,   // bin 27  — start of band 7
    1550.390625f,     // bin 36  — start of band 8
    2024.12109375f,   // bin 47  — start of band 9
    2627.05078125f,   // bin 61  — start of band 10
    3402.24609375f,   // bin 79  — start of band 11
    4392.7734375f,    // bin 102 — start of band 12
    5641.69921875f,   // bin 131 — start of band 13
    7278.22265625f,   // bin 169 — start of band 14
    9388.4765625f,    // bin 218 — start of band 15
    11025.0f,         // bin 256 — end of band 15 (Nyquist at 22050/512)
};

struct BandDefinition {
  size_t bin_start;
  size_t bin_end; // exclusive
};

class BandAggregator {
 public:
  explicit BandAggregator(uint32_t sample_rate) {
    this->hz_per_bin_ = static_cast<float>(sample_rate) / static_cast<float>(SAMPLES_FFT);
    size_t num_bins = SAMPLES_FFT / 2;

    for (int b = 0; b < 16; b++) {
      size_t start = freq_to_bin(BAND_FREQ_BOUNDARIES[b]);
      size_t end = freq_to_bin(BAND_FREQ_BOUNDARIES[b + 1]);
      this->bands_[b] = {std::min(start, num_bins), std::min(end, num_bins)};
    }
  }

  void process(const float *magnitudes, float &out_bass, float &out_mid, float &out_high) {
    if (magnitudes == nullptr) {
      out_bass = out_mid = out_high = 0.0f;
      return;
    }

    // Calculate RMS energy for 16 bands
    float bands16[16]{0.0f};
    for (int b = 0; b < 16; b++) {
      bands16[b] = rms_slice(magnitudes, this->bands_[b].bin_start, this->bands_[b].bin_end);
    }

    // Cascade bands into final 3 macro outputs
    out_bass = rms_slice(bands16, 0, 4);   // Bass: 00-03
    out_mid  = rms_slice(bands16, 4, 10);  // Mid:  04-09
    out_high = rms_slice(bands16, 10, 16); // High: 10-15
  }

 private:
  float hz_per_bin_;
  BandDefinition bands_[16];

  size_t freq_to_bin(float freq_hz) const {
    return static_cast<size_t>(roundf(freq_hz / this->hz_per_bin_));
  }

  // Root-Mean-Square (RMS) calculation
  static float rms_slice(const float *data, size_t start, size_t end) {
    if (start >= end) return 0.0f;
    float sum = 0.0f;
    for (size_t i = start; i < end; i++) sum += data[i] * data[i];
    return sqrtf(sum / static_cast<float>(end - start));
  }
};

}  // namespace esphome::music_leds
