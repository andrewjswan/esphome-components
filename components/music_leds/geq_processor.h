#pragma once

#include "common.h"
#include "constants.h"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>

// #define DEBUG

#ifdef DEBUG
#include "debug.h"
#endif

namespace esphome::music_leds {

constexpr float FFT_DOWNSCALE = 0.40f;
// constexpr float FFT_DOWNSCALE = 0.39f;
// constexpr float FFT_DOWNSCALE = 0.38f;

class GEQProcessor {
 public:
  explicit GEQProcessor(float sample_scale, uint8_t sample_gain = 60) : sample_gain_(sample_gain) {
    this->base_pcm_scale_ = AMPLITUDE_SCALE_16BIT * sample_scale;

    // Native 22050Hz mapping matrix from softhack007
    this->bands_[0] = {1, 2};       // 43Hz   - 86Hz sub-bass
    this->bands_[1] = {2, 3};       // 86Hz   - 129Hz bass
    this->bands_[2] = {3, 5};       // 129Hz  - 216Hz bass
    this->bands_[3] = {5, 7};       // 216Hz  - 301Hz bass + midrange
    this->bands_[4] = {7, 10};      // 301Hz  - 430Hz midrange
    this->bands_[5] = {10, 13};     // 430Hz  - 560Hz midrange
    this->bands_[6] = {13, 19};     // 560Hz  - 818Hz midrange
    this->bands_[7] = {19, 26};     // 818Hz  - 1120Hz midrange (1Khz center anchor)
    this->bands_[8] = {26, 33};     // 1120Hz - 1421Hz midrange
    this->bands_[9] = {33, 44};     // 1421Hz - 1895Hz midrange
    this->bands_[10] = {44, 56};    // 1895Hz - 2412Hz midrange + high mid
    this->bands_[11] = {56, 70};    // 2412Hz - 3015Hz high mid
    this->bands_[12] = {70, 86};    // 3015Hz - 3704Hz high mid
    this->bands_[13] = {86, 104};   // 3704Hz - 4479Hz high mid
    this->bands_[14] = {104, 165};  // 4479Hz - 7106Hz high mid + high
    this->bands_[15] = {165, 215};  // 7106Hz - 9259Hz high (Nyquist noise guard ceiling)

    std::memset(this->fft_calc_, 0, sizeof(this->fft_calc_));
    std::memset(this->fft_avg_, 0, sizeof(this->fft_avg_));
  }

  void process(const float *magnitudes, uint8_t *output_array, bool is_gate_closed) {
    // Standardize input user slider gain into a proportional linear scaler
    float dynamic_gain_scaler = static_cast<float>(this->sample_gain_) / 40.0f + 1.0f / 16.0f;

#ifdef DEBUG
    // Zero-initialize tracking arrays to capture pure current frame execution state
    float trace_raw_sum[16] = {0.0f};
    float trace_after_gain[16] = {0.0f};
    float trace_compressed[16] = {0.0f};
#endif

    for (uint8_t i = 0; i < 16; i++) {
      // Gate Open: Compute fresh channel energy
      if (!is_gate_closed && magnitudes != nullptr) {
        // Compute Band Energy via True Integrated RMS Amplitude
        float bin_energy_sum = 0.00001f;
        int start_bin = this->bands_[i].bin_start;
        int end_bin = this->bands_[i].bin_end;
        int bin_count = end_bin - start_bin + 1;

        for (int b = start_bin; b <= end_bin; b++) {
          // Accumulate raw spectral energy using the squared magnitude domain.
          // This models acoustic power conservation (Parseval's theorem) and protects short,
          // high-frequency transient energy spikes from being mathematically diluted by wide bands.
          bin_energy_sum += (magnitudes[b] * magnitudes[b]);
        }

#ifdef DEBUG
        // Retain tracking sums (Legacy compatibility placeholder for raw diagnostics)
        trace_raw_sum[i] = bin_energy_sum;
#endif

        // Compute true Root Mean Square (RMS) linear amplitude.
        // Extract total band amplitude from power domain using square root first,
        // then divide by the physical bin count to normalize spectral density.
        // Then apply the attenuation factor (1/16).
        float rms_amplitude = sqrtf(bin_energy_sum) / static_cast<float>(bin_count);
        this->fft_calc_[i] = rms_amplitude * 0.0625f;

        // Apply high-frequency dampening curve (Converted to squared power domain multipliers)
        if (i == 14) {
          this->fft_calc_[i] *= 0.88f;  // High-mid dampener
        } else if (i == 15) {
          this->fft_calc_[i] *= 0.70f;  // Nyquist guard dampener
        }

        // Apply pink noise equalization curve
        this->fft_calc_[i] *= PINK_NOISE_CURVE[i];

        // Apply window downscale and volume multipliers
        if (this->scaling_mode_ != FFTScalingMode::LINEAR) {
          this->fft_calc_[i] *= FFT_DOWNSCALE;
        }
        this->fft_calc_[i] *= dynamic_gain_scaler;

        if (this->fft_calc_[i] < 0.0f) {
          this->fft_calc_[i] = 0.0f;
        }
      }
      // Gate Closed: Smoothly decay existing value to absolute zero
      else {
        this->fft_calc_[i] *= 0.85f;
        if (this->fft_calc_[i] < 5.0f) {
          this->fft_calc_[i] = 0.0f;
        }
      }

#ifdef DEBUG
      trace_after_gain[i] = this->fft_calc_[i];
#endif

      // Filters Phase: Execute asymmetric temporal filters
      if (this->fft_calc_[i] > this->fft_avg_[i]) {
        // Rapid adaptive attack tracking on fast transient energy bursts
        this->fft_avg_[i] = (this->fft_calc_[i] * 0.75f) + (this->fft_avg_[i] * 0.25f);
      } else {
        // Smooth standard release decay profile tracking (Default 1400ms decay window)
        this->fft_avg_[i] = (this->fft_calc_[i] * 0.17f) + (this->fft_avg_[i] * 0.83f);
      }

      // Compression Phase: Map the instant frame value into the 8-bit viewport
      float current_result = this->fft_calc_[i];

      // Convert the internal spectrum register safely into a clean fraction [0.0 .. 1.0]
      // using the authentic hardware-calibrated PCM limit
      float normalized_fraction = current_result / this->base_pcm_scale_;
      normalized_fraction = std::clamp(normalized_fraction, 0.0f, 1.0f);

      switch (this->scaling_mode_) {
        case FFTScalingMode::SQUARE_ROOT: {
          // Standard psychoacoustic curve execution on a pure fraction.
          // Quiet signals are expanded, while maximum peaks converge perfectly at 1.0f.
          float compressed_curve = sqrtf(normalized_fraction);

          // Apply balanced ISO 226 human hearing equalization curve.
          // Gently scales higher channels from 0.85 up to 1.60 to prevent flatline clipping.
          compressed_curve *= (0.85f + (static_cast<float>(i) / 20.0f));

          // Map directly into the full 8-bit viewport array [0 .. 255]
          current_result = compressed_curve * 255.0f;
          break;
        }

        case FFTScalingMode::LOGARITHMIC: {
          // Normalized natural logarithm transformation avoiding log(0) exceptions.
          // Scaled explicitly so that 0.0f maps to 0.0f, and 1.0f maps perfectly to 1.0f.
          float compressed_curve = logf(normalized_fraction * 9.0f + 1.0f) / 2.30258509f;  // Divide by log(10)

          // Tailored high-frequency balance adapted for aggressive logarithmic density
          compressed_curve *= (0.85f + (static_cast<float>(i) / 25.0f));

          current_result = compressed_curve * 255.0f;
          break;
        }

        case FFTScalingMode::LINEAR:
        default: {
          // Clean, uncompressed baseline viewport mapping profile.
          // Relies on a sharper pre-emphasis slope to maintain high frequency visibility.
          float linear_curve = normalized_fraction * (0.85f + (static_cast<float>(i) / 15.0f));

          current_result = linear_curve * 255.0f;
          break;
        }
      }

#ifdef DEBUG
      trace_compressed[i] = current_result;
#endif

      // Push final bytes back into shared viewport packet array for FastLED execution.
      // The final clamp cleanly absorbs any headroom saturation during explosive musical moments.
      output_array[i] = static_cast<uint8_t>(std::clamp(current_result, 0.0f, 255.0f));
    }

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("GEQ_TRACE", "=====================================================");
      const uint8_t target_channels[] = {0, 4, 15};
      for (uint8_t ch : target_channels) {
        ESP_LOGD("GEQ_TRACE",
                 "CH[%02d] Bins[%d..%d] | RawSum:%.2f | AfterGain:%.2f | Compressed:%.2f | FFTCalc:%.2f | FFTAvg:%.2f "
                 "| Byte:%d | GateClosed:%s",
                 ch, this->bands_[ch].bin_start, this->bands_[ch].bin_end, trace_raw_sum[ch], trace_after_gain[ch],
                 trace_compressed[ch], this->fft_calc_[ch], this->fft_avg_[ch], output_array[ch],
                 is_gate_closed ? "YES" : "NO");
      }
    }
#endif
  }

  void set_sample_gain(uint8_t gain) { this->sample_gain_ = gain; }
  void set_scaling_mode(FFTScalingMode mode) { this->scaling_mode_ = mode; }

 private:
  BandDefinition bands_[NUM_GEQ_CHANNELS];
  float fft_calc_[NUM_GEQ_CHANNELS];
  float fft_avg_[NUM_GEQ_CHANNELS];

  uint8_t sample_gain_{60};
  float base_pcm_scale_{AMPLITUDE_SCALE_16BIT};
  FFTScalingMode scaling_mode_{FFTScalingMode::SQUARE_ROOT};
};

}  // namespace esphome::music_leds
