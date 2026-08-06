#pragma once

#include "common.h"
#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

class BandAggregator {
 public:
  explicit BandAggregator(uint32_t sample_rate) {
    // Fully parameterize scaling using the global SAMPLES_FFT constant to support future 1024 switches
    this->hz_per_bin_ = static_cast<float>(sample_rate) / static_cast<float>(SAMPLES_FFT);

    // Anti-Aliasing Brickwall Guard (Dynamic Scaling)
    // Don't use the last bins from 216 to 255. They are usually contaminated by aliasing (aka noise)
    // Enforces constraint dynamically. For 512 samples, it cuts off strictly at bin 216.
    // If scaled to 1024 samples, it automatically scales to keep the same physical frequency cutoff window.
    size_t absolute_safe_ceiling = static_cast<size_t>(static_cast<float>(MAX_VALID_BIN) * 0.84375f);

    for (int b = 0; b < NUM_GEQ_CHANNELS; b++) {
      size_t start = freq_to_bin(BAND_FREQ_BOUNDARIES[b]);
      size_t end = freq_to_bin(BAND_FREQ_BOUNDARIES[b + 1]);

      // Enforce the dynamic safety ceiling across all 16 calculated sub-bands
      this->bands_[b] = {std::min(start, absolute_safe_ceiling), std::min(end, absolute_safe_ceiling)};
    }
  }

  /**
   * @brief Aggregates raw FFT magnitudes directly into 3 high-fidelity physical macro bands.
   */
  void process(const float *magnitudes, float &out_bass, float &out_mid, float &out_high) {
    if (magnitudes == nullptr) {
      out_bass = out_mid = out_high = 0.0f;
      return;
    }

    // High-performance single-stage macro aggregation directly from raw safe spectrum bins.
    out_bass = rms_slice(magnitudes, this->bands_[0].bin_start, this->bands_[3].bin_end);    // Sub-bands 00-03 (Bass)
    out_mid = rms_slice(magnitudes, this->bands_[4].bin_start, this->bands_[9].bin_end);     // Sub-bands 04-09 (Mid)
    out_high = rms_slice(magnitudes, this->bands_[10].bin_start, this->bands_[15].bin_end);  // Sub-bands 10-15 (High)
  }

 private:
  float hz_per_bin_;
  BandDefinition bands_[NUM_GEQ_CHANNELS];

  /**
   * @brief Converts physical frequency in Hz to spectrum bin index with an integrated low-cut filter.
   */
  size_t freq_to_bin(float freq_hz) const {
    size_t calculated_bin = static_cast<size_t>(roundf(freq_hz / this->hz_per_bin_));

    // Integrated High-Pass Sub-Sonic Filter (10240hz Legacy Guard)
    // Clamps the lowest processed spectrum bin strictly to index 3 (~60 Hz).
    // This removes invisible sub-bass room rumble, DC jitter, and floor noise artifacts.
    return std::max(calculated_bin, static_cast<size_t>(3));
  }

  static float rms_slice(const float *data, size_t start, size_t end) {
    if (start >= end)
      return 0.0f;
    float sum = 0.0f;
    for (size_t i = start; i < end; i++) {
      sum += data[i] * data[i];
    }
    return sqrtf(sum / static_cast<float>(end - start));
  }
};

}  // namespace esphome::music_leds
