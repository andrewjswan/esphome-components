#pragma once

#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

// 17 frequency boundaries (in Hz) defining 16 bands.
// Standard log-scale frequency map boundaries used for multi-band audio analysis
static constexpr float BAND_FREQ_BOUNDARIES[] = {
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
  size_t bin_end;
};

class BandAggregator {
 public:
  explicit BandAggregator(uint32_t sample_rate) {
    // Fully parameterize scaling using the global SAMPLES_FFT constant to support future 1024 switches
    this->hz_per_bin_ = static_cast<float>(sample_rate) / static_cast<float>(SAMPLES_FFT);
    size_t num_bins = SAMPLES_FFT / 2; // Maximum positions in the magnitude spectrum

    // Anti-Aliasing Brickwall Guard (Dynamic Scaling)
    // Don't use the last bins from 216 to 255. They are usually contaminated by aliasing (aka noise) 
    // Enforces constraint dynamically. For 512 samples, it cuts off strictly at bin 216.
    // If scaled to 1024 samples, it automatically scales to keep the same physical frequency cutoff window.
    size_t absolute_safe_ceiling = static_cast<size_t>(static_cast<float>(num_bins) * 0.84375f);

    for (int b = 0; b < 16; b++) {
      size_t start = freq_to_bin(BAND_FREQ_BOUNDARIES[b]);
      size_t end = freq_to_bin(BAND_FREQ_BOUNDARIES[b + 1]);
      
      // Enforce the dynamic safety ceiling across all 16 calculated sub-bands
      this->bands_[b] = {
        std::min(start, absolute_safe_ceiling), 
        std::min(end, absolute_safe_ceiling)
      };
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
    out_bass = rms_slice(magnitudes, this->bands_[0].bin_start,  this->bands_[3].bin_end);   // Sub-bands 00-03 (Bass)
    out_mid  = rms_slice(magnitudes, this->bands_[4].bin_start,  this->bands_[9].bin_end);   // Sub-bands 04-09 (Mid)
    out_high = rms_slice(magnitudes, this->bands_[10].bin_start, this->bands_[15].bin_end);  // Sub-bands 10-15 (High)
  }

 private:
  float hz_per_bin_;
  BandDefinition bands_[16];

  /**
   * @brief Converts physical frequency in Hz to spectrum bin index with an integrated low-cut filter.
   */
  size_t freq_to_bin(float freq_hz) const {
    size_t calculated_bin = static_cast<size_t>(roundf(freq_hz / this->hz_per_bin_));
    
    // --- INTEGRATED HIGH-PASS SUB-SONIC FILTER (Andrew's 10240Hz Legacy Guard) ---
    // Clamps the lowest processed spectrum bin strictly to index 3 (~60 Hz).
    // This removes invisible sub-bass room rumble, DC jitter, and floor noise artifacts.
    return std::max(calculated_bin, static_cast<size_t>(3));
  }

  static float rms_slice(const float *data, size_t start, size_t end) {
    if (start >= end) return 0.0f;
    float sum = 0.0f;
    for (size_t i = start; i < end; i++) {
      sum += data[i] * data[i];
    }
    return sqrtf(sum / static_cast<float>(end - start));
  }
};

} // namespace esphome::music_leds
