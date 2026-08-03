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
  explicit DynamicsProcessor(float sample_scale) {
    this->last_execution_time_ = micros();

    // Pre-calculate hardware-calibrated structural constraints once on initialization
    this->base_pcm_scale_ = AMPLITUDE_SCALE_16BIT * sample_scale;

    this->agc_min_floor_shift_ = 0.05f * this->base_pcm_scale_;
    this->safe_vol_peak_floor_ = 0.35f * this->base_pcm_scale_;

    // Initialize the single global tracking field using the optimized base floor
    this->vol_agc_peak_ = this->agc_min_floor_shift_;
  }

  /**
   * @brief Processes frequency bands using internal smoothers and calculates
   *        overall raw/smoothed time-domain audio loudness envelopes.
   * @note Processes non-normalized input magnitudes for volume and outputs standardized linear scales for bands.
   */
  void process(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    // Static hardware noise floor threshold to filter out microphone preamp electrical hiss
    constexpr float NOISE_GATE_THRESHOLD = 40.0f;

    // Single-pole Infinite Impulse Response (IIR) Low-Pass Filter for temporal smoothing.
    // FILTER_SMOOTHING_FACTOR (Beta) defines the responsiveness to incoming transient acoustic peaks.
    // FILTER_RETENTION_FACTOR (1 - Beta) acts as the exponential decay memory coefficient.
    constexpr float FILTER_SMOOTHING_FACTOR = 0.05f;
    constexpr float FILTER_RETENTION_FACTOR = 0.95f;

    uint32_t now = micros();
    uint32_t delta_micros = now - this->last_execution_time_;
    this->last_execution_time_ = now;

    float delta_ms = static_cast<float>(delta_micros) / 1000.0f;
    if (delta_ms > 200.0f)
      delta_ms = 20.0f;

    // Capture the pure raw time-domain max_sample() passed down from the FFT engine interface
    float mic_data_real = raw_vol;

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

    // Frequency Bands Tracking Block
    // High-performance single-precision computation of the input raw frame volume (Large scales)
    float mean_band_energy = (bass + mid + high) * 0.33333334f;

    // Continuous single-stage global AGC peak tracking for the baseline volume
    if (mean_band_energy > this->vol_agc_peak_) {
      this->vol_agc_peak_ = mean_band_energy;
    } else {
      this->vol_agc_peak_ = (this->vol_agc_peak_ * 0.9995f) + (mean_band_energy * 0.0005f);
    }

    // Normalize and apply independent time-locked linear rate limiting per frequency band group
    bass = apply_time_limiter(bass, this->bass_smoothed_, 40.0f, 1200.0f, delta_ms);
    mid = apply_time_limiter(mid, this->mid_smoothed_, 60.0f, 1400.0f, delta_ms);
    high = apply_time_limiter(high, this->high_smoothed_, 30.0f, 800.0f, delta_ms);

    // Time-Domain Volume Envelope Generation
    // Apply noise floor deduction and map strictly to clean signed 16-bit safe integer boundaries
    float calculated_raw = std::clamp(mic_data_real - NOISE_GATE_THRESHOLD, 0.0f, static_cast<float>(INT16_MAX));

    // Execute exponential moving average temporal smoothing pass
    this->volume_smoothed_ = (calculated_raw * FILTER_SMOOTHING_FACTOR) +
                             (this->volume_smoothed_ * FILTER_RETENTION_FACTOR);

    // Dynamic hardware mapping: translate large 16-bit boundaries into standard [0.0f .. 1.0f] float scales
    raw_vol = std::clamp(calculated_raw / this->base_pcm_scale_, 0.0f, 1.0f);
    smoothed_vol = std::clamp(this->volume_smoothed_ / this->base_pcm_scale_, 0.0f, 1.0f);
  }

  /**
   * @brief Final pipeline step compressing macro bands into perception-aligned scales.
   * @note Volume envelopes (raw_vol, smoothed_vol) bypass this pass to preserve full 16-bit time-domain dynamics.
   */
  void apply_psychoacoustic_scaling(float &smoothed_vol, float &raw_vol, float &bass, float &mid, float &high) {
    if (bass == 0.0f && mid == 0.0f && high == 0.0f) {
      raw_vol = 0.0f;
      smoothed_vol = 0.0f;
      return;
    }

    // Genuine Psychoacoustic Compression (Weber-Fechner Law Implementation)
    // Applied strictly to frequency bands to emulate human auditory perception curves,
    // while global volume metrics bypass this to maintain native high-fidelity scaling.
    switch (this->scaling_mode_) {
      case FFTScalingMode::SQUARE_ROOT:
        raw_vol = sqrtf(raw_vol);
        smoothed_vol = sqrtf(smoothed_vol);
        bass = sqrtf(bass);
        mid = sqrtf(mid);
        high = sqrtf(high);
        break;

      case FFTScalingMode::LOGARITHMIC:
        raw_vol = logf(1.0f + raw_vol * 1.7182818f);
        smoothed_vol = logf(1.0f + smoothed_vol * 1.7182818f);
        bass = logf(1.0f + bass * 1.7182818f);
        mid = logf(1.0f + mid * 1.7182818f);
        high = logf(1.0f + high * 1.7182818f);
        break;

      case FFTScalingMode::LINEAR:
      default:
        break;  // Keep frequency bands fully linear without alterations
    }
  }

  void set_scaling_mode(FFTScalingMode mode) { this->scaling_mode_ = mode; }

 private:
  uint32_t last_execution_time_{0};
  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT};

  // Cached hardware amplitude limit to eliminate runtime multipliers during volume normalization
  float base_pcm_scale_{AMPLITUDE_SCALE_16BIT};

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
