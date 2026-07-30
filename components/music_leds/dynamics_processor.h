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
    
    this->agc_min_floor_shift_   = 0.05f * base_pcm_scale;
    this->safe_vol_peak_floor_   = 0.35f * base_pcm_scale;

    // Initialize the single global tracking field using the optimized base floor
    this->vol_agc_peak_ = this->agc_min_floor_shift_;
  }

  /**
   * @brief Applies linear AGC normalization and linear Slew-Rate temporal rate limiting.
   * @note Generates clean, un-distorted linear volumes for precise subsequent NoiseGate evaluations.
   */
  void process(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    // Calculate precise frame elapsed time delta in milliseconds
    uint32_t now = micros();
    uint32_t delta_micros = now - this->last_execution_time_;
    this->last_execution_time_ = now;
    
    // Fallback filter capping extreme unexpected thread scheduling gaps
    float delta_ms = static_cast<float>(delta_micros) / 1000.0f;
    if (delta_ms > 200.0f) delta_ms = 20.0f;

    // Intercept closed gate zero-lines to freeze the AGC loop and prevent gain explosion
    if (bass == 0.0f && mid == 0.0f && high == 0.0f) {
      raw_vol = 0.0f;

      // Smoothly bleed off the remaining global envelope to maintain IIR filter continuity
      smoothed_vol = smoothed_vol * 0.80f;
      if (smoothed_vol < 0.001f) smoothed_vol = 0.0f;

      // Hard reset historical registers to eliminate infinite slow decay tails
      this->bass_smoothed_ = 0.0f;
      this->mid_smoothed_  = 0.0f;
      this->high_smoothed_ = 0.0f;

      // Bleed off the single global historical AGC peak during silence to prevent track-change freeze artifacts
      this->vol_agc_peak_ = std::max(this->agc_min_floor_shift_, this->vol_agc_peak_ * 0.95f);
      return; 
    }

    // High-performance single-precision computation of the input raw frame volume
    float incoming_raw_volume = (bass + mid + high) * 0.33333334f;

    // Continuous single-stage global AGC peak tracking for the baseline volume
    if (incoming_raw_volume > this->vol_agc_peak_) {
      this->vol_agc_peak_ = incoming_raw_volume;
    } else {
      // Slow structural decay to adapt to silent passages over time
      this->vol_agc_peak_ = (this->vol_agc_peak_ * 0.9995f) + (incoming_raw_volume * 0.0005f);
    }

    // Choose peak compared to pre-calculated dynamic noise protection constraint
    float safe_vol_peak = std::max(this->vol_agc_peak_, this->safe_vol_peak_floor_);
    float normalized_vol = std::clamp(incoming_raw_volume / safe_vol_peak, 0.0f, 1.0f);

    // Traditional exponential smoothing for the overall global loudness envelope
    float vol_coeff = (normalized_vol > smoothed_vol) ? 0.35f : 0.06f;
    smoothed_vol = (vol_coeff * normalized_vol) + ((1.0f - vol_coeff) * smoothed_vol);

    // Normalize and apply independent time-locked linear rate limiting per frequency band group
    // All bands share the same unified dynamic global peak to fully preserve physical music ratios!
    bass = apply_time_limiter(bass, this->bass_smoothed_, 40.0f, 1200.0f, delta_ms); // Snappy bass
    mid  = apply_time_limiter(mid,  this->mid_smoothed_,  60.0f, 1400.0f, delta_ms); // Voice/instruments
    high = apply_time_limiter(high, this->high_smoothed_, 30.0f, 800.0f,  delta_ms); // Crisp cymbals

    // Output true linear post-AGC instantaneous volume envelope instead of band mean.
    // This protects raw_vol from time-limiter band compression distortions, keeping it clean.
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

    // --- GENUINE PSYCHOACOUSTIC COMPRESSION (Weber-Fechner Law Implementation) ---
    // Instead of destructive re-computation via arithmetic mean, we apply the non-linear 
    // scaling mode to each parameter independently. This preserves the precise dynamic purpose 
    // of raw_vol (instantaneous pulses) and smoothed_vol (smooth tracking) while lifting 
    // them into the expected 120-180 byte bracket for visual rendering.
    switch (this->scaling_mode_) {
      case FFTScalingMode::SQUARE_ROOT:
        bass = sqrtf(bass);
        mid  = sqrtf(mid);
        high = sqrtf(high);
        raw_vol = sqrtf(raw_vol);
        smoothed_vol = sqrtf(smoothed_vol);
        break;

      case FFTScalingMode::LOGARITHMIC:
        bass = logf(1.0f + bass * 1.7182818f);
        mid  = logf(1.0f + mid  * 1.7182818f);
        high = logf(1.0f + high * 1.7182818f);
        raw_vol = logf(1.0f + raw_vol * 1.7182818f);
        smoothed_vol = logf(1.0f + smoothed_vol * 1.7182818f);
        break;

      case FFTScalingMode::LINEAR:
      default:
        break; // Keep everything fully linear without alterations
    }
  }

  void set_scaling_mode(FFTScalingMode mode) {
    this->scaling_mode_ = mode;
  }

 private:
  float sample_scale_; 
  uint32_t last_execution_time_{0};
  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT}; 
  
  // Pre-calculated audio scale boundaries to eliminate intensive runtime FPU multiplications
  float agc_min_floor_shift_{0.0f};   // 0.05f threshold
  float safe_vol_peak_floor_{0.0f};   // 0.35f threshold

  // The single unified global AGC history peak tracking register
  float vol_agc_peak_{0.05f};

  // Persistent tracking fields for temporal rate-limiting histories
  float bass_smoothed_{0.0f};
  float mid_smoothed_{0.0f};
  float high_smoothed_{0.0f};

  /**
   * @brief Combines strict linear slew-rate limiting with global AGC tracking to preserve inter-band ratios.
   */
  inline float apply_time_limiter(float raw_energy, float &last_value, float attack_ms, float decay_ms, float delta_ms) {
    // Scale incoming value to a clean range using highly optimized pre-calculated global floor registers
    float unified_peak = std::max(this->vol_agc_peak_, this->safe_vol_peak_floor_);
    float target_value = std::clamp(raw_energy / unified_peak, 0.0f, 1.0f);

    // Precise physical step sizing derived directly from hardware delta time constraints
    float max_rise = delta_ms / attack_ms;
    float max_fall = delta_ms / decay_ms;

    if (target_value > last_value) {
      last_value = std::min(target_value, last_value + max_rise); // Non-blocking smooth attack limit
    } else {
      last_value = std::max(target_value, last_value - max_fall); // Non-blocking smooth decay limit
    }

    return last_value;
  }
};

} // namespace esphome::music_leds
