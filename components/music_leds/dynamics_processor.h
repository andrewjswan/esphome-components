#pragma once

#include "constants.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

class DynamicsProcessor {
 public:
  /**
   * @brief Dynamic initialization linking the processor with the main scale factor.
   * @param sample_scale The amplitude division factor from the main component (e.g., 1.0f / 24.0f)
   */
  explicit DynamicsProcessor(float sample_scale) : sample_scale_(sample_scale) {
    this->last_execution_time_ = micros();

    // Pre-calculate hardware-calibrated structural constraints once on initialization
    float base_pcm_scale = AMPLITUDE_SCALE_16BIT * this->sample_scale_;

    this->agc_min_floor_shift_ = 0.05f * base_pcm_scale;
    this->safe_vol_peak_floor_ = 0.35f * base_pcm_scale;

    // Initialize the single global tracking field using the optimized base floor
    this->vol_agc_peak_ = this->agc_min_floor_shift_;
  }

  /**
   * @brief Applies linear AGC normalization and linear Slew-Rate temporal envelope limiting.
   * @note Implements a physics-compliant slew-rate tracker adjusted for a normalized [0.0f .. 1.0f] float scale.
   */
  /**
   * @brief Applies linear AGC normalization and temporal Slew-Rate envelope limiting.
   * @note Processes non-normalized input magnitudes and outputs standardized linear scales.
   */
  void process(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    uint32_t now = micros();
    uint32_t delta_micros = now - this->last_execution_time_;
    this->last_execution_time_ = now;

    float delta_ms = static_cast<float>(delta_micros) / 1000.0f;
    if (delta_ms > 200.0f)
      delta_ms = 20.0f;

    // Intercept closed gate zero-lines to freeze the AGC loop and prevent gain explosion
    if (bass == 0.0f && mid == 0.0f && high == 0.0f) {
      raw_vol = 0.0f;
      smoothed_vol = 0.0f;

      this->volume_smoothed_ = 0.0f;

      this->bass_smoothed_ = 0.0f;
      this->mid_smoothed_ = 0.0f;
      this->high_smoothed_ = 0.0f;

      this->vol_agc_peak_ = std::max(this->agc_min_floor_shift_, this->vol_agc_peak_ * 0.90f);
      return;
    }

    // High-performance single-precision computation of the input raw frame volume (Large scales)
    float incoming_raw_volume = (bass + mid + high) * 0.33333334f;

    // Continuous single-stage global AGC peak tracking for the baseline volume
    if (incoming_raw_volume > this->vol_agc_peak_) {
      this->vol_agc_peak_ = incoming_raw_volume;
    } else {
      this->vol_agc_peak_ = (this->vol_agc_peak_ * 0.9995f) + (incoming_raw_volume * 0.0005f);
    }

    float safe_vol_peak = std::max(this->vol_agc_peak_, this->safe_vol_peak_floor_);

    // Map current instantaneous frame volume to clean standardized [0.0f .. 1.0f] scale
    float normalized_vol = std::clamp(incoming_raw_volume / safe_vol_peak, 0.0f, 1.0f);

    // Isolated Linear Slew-Rate Limiter
    // Target step velocity factor translated to normalized floating-point container bounds
    const float step_magnitude_scale = 0.76862745f;
    float delta_sample = normalized_vol - this->volume_smoothed_;

    // Limit attack velocity slope (Reference context: 80.0ms)
    float max_attack_step = step_magnitude_scale * delta_ms / 80.0f;
    if (delta_sample > max_attack_step) {
      delta_sample = max_attack_step;
    }

    // Limit decay velocity slope (Reference context: 1400.0ms)
    float max_decay_step = -step_magnitude_scale * delta_ms / 1400.0f;
    if (delta_sample < max_decay_step) {
      delta_sample = max_decay_step;
    }

    // Update internal protected linear envelope tracking register
    this->volume_smoothed_ = std::clamp(this->volume_smoothed_ + delta_sample, 0.0f, 1.0f);

    // Normalize and apply independent time-locked linear rate limiting per frequency band group
    bass = apply_time_limiter(bass, this->bass_smoothed_, 40.0f, 1200.0f, delta_ms);
    mid = apply_time_limiter(mid, this->mid_smoothed_, 60.0f, 1400.0f, delta_ms);
    high = apply_time_limiter(high, this->high_smoothed_, 30.0f, 800.0f, delta_ms);

    // Write final clean linear post-AGC envelopes to reference interfaces
    smoothed_vol = this->volume_smoothed_;
    raw_vol = normalized_vol;
  }

  /**
   * @brief Final pipeline step compressing both envelopes and macro bands into perception-aligned scales.
   * @note Must be called AFTER the NoiseGate has completed its linear evaluation pass.
   */
  void apply_psychoacoustic_scaling(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    if (bass == 0.0f && mid == 0.0f && high == 0.0f) {
      raw_vol = 0.0f;
      smoothed_vol = 0.0f;
      return;
    }

    // Genuine Psychoacoustic Compression (Weber-Fechner Law Implementation)
    // Instead of destructive re-computation via arithmetic mean, we apply the non-linear
    // scaling mode to each parameter independently. This preserves the precise dynamic purpose
    // of raw_vol (instantaneous pulses) and smoothed_vol (smooth tracking) while lifting
    // them into the expected 120-180 byte bracket for visual rendering.
    switch (this->scaling_mode_) {
      case FFTScalingMode::SQUARE_ROOT:
        bass = sqrtf(bass);
        mid = sqrtf(mid);
        high = sqrtf(high);
        raw_vol = sqrtf(raw_vol);
        smoothed_vol = sqrtf(smoothed_vol);
        break;

      case FFTScalingMode::LOGARITHMIC:
        bass = logf(1.0f + bass * 1.7182818f);
        mid = logf(1.0f + mid * 1.7182818f);
        high = logf(1.0f + high * 1.7182818f);
        raw_vol = logf(1.0f + raw_vol * 1.7182818f);
        smoothed_vol = logf(1.0f + smoothed_vol * 1.7182818f);
        break;

      case FFTScalingMode::LINEAR:
      default:
        break;  // Keep everything fully linear without alterations
    }
  }

  void set_scaling_mode(FFTScalingMode mode) { this->scaling_mode_ = mode; }

 private:
  float sample_scale_;
  uint32_t last_execution_time_{0};
  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT};

  // Pre-calculated audio scale boundaries to eliminate intensive runtime FPU multiplications
  float agc_min_floor_shift_{0.0f};  // 0.05f threshold
  float safe_vol_peak_floor_{0.0f};  // 0.35f threshold

  // The single unified global AGC history peak tracking register
  float vol_agc_peak_{0.05f};

  // Persistent tracking fields for temporal rate-limiting histories
  float volume_smoothed_{0.0f};
  float bass_smoothed_{0.0f};
  float mid_smoothed_{0.0f};
  float high_smoothed_{0.0f};

  /**
   * @brief Combines strict linear slew-rate limiting with global AGC tracking to preserve inter-band ratios.
   */
  inline float apply_time_limiter(float raw_energy, float &last_value, float attack_ms, float decay_ms,
                                  float delta_ms) {
    // Scale incoming value to a clean range using highly optimized pre-calculated global floor registers
    float unified_peak = std::max(this->vol_agc_peak_, this->safe_vol_peak_floor_);
    float target_value = std::clamp(raw_energy / unified_peak, 0.0f, 1.0f);

    // Precise physical step sizing derived directly from hardware delta time constraints
    float max_rise = delta_ms / attack_ms;
    float max_fall = delta_ms / decay_ms;

    if (target_value > last_value) {
      last_value = std::min(target_value, last_value + max_rise);  // Non-blocking smooth attack limit
    } else {
      last_value = std::max(target_value, last_value - max_fall);  // Non-blocking smooth decay limit
    }

    return last_value;
  }
};

}  // namespace esphome::music_leds
