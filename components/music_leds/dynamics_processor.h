#pragma once

#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

class DynamicsProcessor {
 public:
  explicit DynamicsProcessor() {
    this->last_execution_time_ = micros();
  }

  /**
   * @brief Applies AGC normalization and time-locked rate limiting to all pipeline features.
   * @param raw_vol Input instantaneous frame volume from peak detector.
   * @param smoothed_vol Output dynamic smoothed envelope destination.
   * @param bass In/Out reference for bass band energy processing.
   * @param mid In/Out reference for midrange band energy processing.
   * @param high In/Out reference for high frequency band energy processing.
   */
  void process(float &raw_vol, float &smoothed_vol, float &bass, float &mid, float &high) {
    // Calculate precise frame elapsed time delta in milliseconds (Matching absent42 concept)
    uint32_t now = micros();
    uint32_t delta_micros = now - this->last_execution_time_;
    this->last_execution_time_ = now;
    
    // Fallback filter capping extreme unexpected thread scheduling gaps
    float delta_ms = static_cast<float>(delta_micros) / 1000.0f;
    if (delta_ms > 200.0f) delta_ms = 20.0f;

    // Continuous AGC peak tracking for the baseline volume
    if (raw_vol > this->vol_agc_peak_) {
      this->vol_agc_peak_ = raw_vol;
    } else {
      this->vol_agc_peak_ = (this->vol_agc_peak_ * 0.9995f) + (raw_vol * 0.0005f); // Slow structural decay
    }

    float safe_vol_peak = std::max(this->vol_agc_peak_, 0.05f);
    float normalized_vol = std::clamp(raw_vol / safe_vol_peak, 0.0f, 1.0f);
    
    // Traditional exponential smoothing for the overall global loudness envelope
    float vol_coeff = (normalized_vol > smoothed_vol) ? 0.35f : 0.06f;
    smoothed_vol = (vol_coeff * normalized_vol) + ((1.0f - vol_coeff) * smoothed_vol);

    // Normalize and apply independent time-locked rate limiting per frequency band group
    bass = apply_time_limiter(bass, this->bass_agc_peak_, this->bass_smoothed_, 40.0f, 1200.0f, delta_ms); // Snappy bass
    mid  = apply_time_limiter(mid,  this->mid_agc_peak_,  this->mid_smoothed_,  60.0f, 1400.0f, delta_ms); // Voice/instruments
    high = apply_time_limiter(high, this->high_agc_peak_, this->high_smoothed_, 30.0f, 800.0f,  delta_ms); // Crisp cymbals
  }

 private:
  uint32_t last_execution_time_{0};
  
  // Persistent tracking fields for AGC history
  float vol_agc_peak_{0.05f};
  float bass_agc_peak_{0.05f};
  float mid_agc_peak_{0.05f};
  float high_agc_peak_{0.05f};

  // Persistent tracking fields for temporal rate-limiting histories
  float bass_smoothed_{0.0f};
  float mid_smoothed_{0.0f};
  float high_smoothed_{0.0f};

  /**
   * @brief Combines AGC peak tracking, normalization, and absent42's time-locked rate limiting into one fast method.
   */
  inline float apply_time_limiter(float raw_energy, float &agc_peak, float &last_value, float attack_ms, float decay_ms, float delta_ms) {
    // Track long term maximum peak energy of the channel
    if (raw_energy > agc_peak) {
      agc_peak = agc_peak + 0.15f * (raw_energy - agc_peak);
    } else {
      agc_peak = (agc_peak * 0.999f) + (raw_energy * 0.001f);
    }

    // Scale incoming value to a neat [0.0f .. 1.0f] target range
    float target_value = std::clamp(raw_energy / std::max(agc_peak, 0.02f), 0.0f, 1.0f);

    // Replicating absent42's rate limiting math (1.0f max scale instead of WLED's 196.0f)
    float max_rise = delta_ms / attack_ms;
    float max_fall = delta_ms / decay_ms;

    if (target_value > last_value) {
      last_value = std::min(target_value, last_value + max_rise); // Smooth attack limit
    } else {
      last_value = std::max(target_value, last_value - max_fall); // Smooth decay limit
    }

    return last_value;
  }
};

} // namespace esphome::music_leds
