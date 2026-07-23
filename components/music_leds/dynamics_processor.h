#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

enum class FFTScalingMode : uint8_t {
  LINEAR = 0,
  LOGARITHMIC = 1,
  SQUARE_ROOT = 2
};

class DynamicsProcessor {
 public:
  explicit DynamicsProcessor() {
    this->last_execution_time_ = micros();
  }

  /**
   * @brief Applies linear AGC normalization and linear Slew-Rate temporal rate limiting.
   * @note Generates clean, un-distorted linear volumes for precise subsequent NoiseGate evaluations.
   */
  void process(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    // Calculate precise frame elapsed time delta in milliseconds (Matching absent42 concept)
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
      return; 
    }

    // High-performance single-precision computation of the input raw frame volume
    float incoming_raw_volume = (bass + mid + high) * 0.33333334f;

    // Continuous AGC peak tracking for the baseline volume
    if (incoming_raw_volume > this->vol_agc_peak_) {
      this->vol_agc_peak_ = incoming_raw_volume;
    } else {
      // Slow structural decay to adapt to silent passages over time
      this->vol_agc_peak_ = (this->vol_agc_peak_ * 0.9995f) + (incoming_raw_volume * 0.0005f);
    }

    // Establishes floor constraints calibrated precisely for 28-35dB room background to block over-amplification
    float safe_vol_peak = std::max(this->vol_agc_peak_, 0.35f); 
    float normalized_vol = std::clamp(incoming_raw_volume / safe_vol_peak, 0.0f, 1.0f);
    
    // Traditional exponential smoothing for the overall global loudness envelope
    float vol_coeff = (normalized_vol > smoothed_vol) ? 0.35f : 0.06f;
    smoothed_vol = (vol_coeff * normalized_vol) + ((1.0f - vol_coeff) * smoothed_vol);

    // Normalize and apply independent time-locked linear rate limiting per frequency band group
    bass = apply_time_limiter(bass, this->bass_agc_peak_, this->bass_smoothed_, 40.0f, 1200.0f, delta_ms); // Snappy bass
    mid  = apply_time_limiter(mid,  this->mid_agc_peak_,  this->mid_smoothed_,  60.0f, 1400.0f, delta_ms); // Voice/instruments
    high = apply_time_limiter(high, this->high_agc_peak_, this->high_smoothed_, 30.0f, 800.0f,  delta_ms); // Crisp cymbals

    // Output true linear post-AGC mean volume used as an anchor by the NoiseGate
    raw_vol = (bass + mid + high) * 0.33333334f;
  }

  /**
   * @brief Final pipeline step compressing linear energies into perception-aligned scales for Core 0.
   * @note Must be called AFTER the NoiseGate has completed its linear evaluation pass.
   */
  void apply_psychoacoustic_scaling(float &raw_vol, float &bass, float &mid, float &high) {
    // Intercept closed gate vectors to prevent FPU anomalies and smooth transition artifacts
    if (bass == 0.0f && mid == 0.0f && high == 0.0f) {
      raw_vol = 0.0f;
      return;
    }

    switch (this->scaling_mode_) {
      case FFTScalingMode::SQUARE_ROOT:
        // Square Root scaling: Significantly boosts quiet details and transients, 
        // preventing the LED strip from looking dead during low-volume passages.
        bass = sqrtf(bass);
        mid  = sqrtf(mid);
        high = sqrtf(high);
        break;

      case FFTScalingMode::LOGARITHMIC:
        // Logarithmic scaling: Matches true human hearing curves.
        // Uses logf(1.0f + x * (e - 1.0f)) to ensure safe [0.0f .. 1.0f] tracking bounds.
        bass = logf(1.0f + bass * 1.7182818f);
        mid  = logf(1.0f + mid  * 1.7182818f);
        high = logf(1.0f + high * 1.7182818f);
        break;

      case FFTScalingMode::LINEAR:
      default:
        return; // Leave linear values intact as calculated by Stage 1
    }

    // Re-compute the final perception-calibrated mean volume matching the newly scaled macro bands
    raw_vol = (bass + mid + high) * 0.33333334f;
  }

  /**
   * @brief Configuration injector allowing runtime switching of scaling behaviors via YAML/C++.
   */
  void set_scaling_mode(FFTScalingMode mode) {
    this->scaling_mode_ = mode;
  }

 private:
  uint32_t last_execution_time_{0};
  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT}; // Defacto gold standard for lighting
  
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
   * @brief Combines strict linear slew-rate limiting with high-speed, non-inverting AGC peak tracking.
   */
  inline float apply_time_limiter(float raw_energy, float &agc_peak, float &last_value, float attack_ms, float decay_ms, float delta_ms) {
    // Immediate or highly aggressive attack tracking to prevent artificial volume inversion artifacts
    if (raw_energy > agc_peak) {
      // Instant latch to accurately scale rapid audio transients
      agc_peak = raw_energy;
    } else {
      // Smooth decay using single-precision literals to avoid emulated double precision FPU overhead
      agc_peak = (agc_peak * 0.999f) + (raw_energy * 0.001f);
    }

    // Scale incoming value to a clean range, enforcing safe floor constraint against low-level digital noise division
    float target_value = std::clamp(raw_energy / std::max(agc_peak, 0.10f), 0.0f, 1.0f);

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
