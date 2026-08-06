#pragma once

#include "constants.h"

/**
 * arduinoFFT's defs.h #defines DDR(x) as ((x)-1) for AVR-ish "data direction
 * register" math. ESP-IDF's xtensa specreg.h #defines DDR as 104 (debug-data
 * register address). When both headers land in the same translation unit
 * (esphome.h pulls FreeRTOS->xtensa, then audio_reactive.h pulls arduinoFFT)
 * the DDR redefinition emits two warnings per compile. Save / undef the
 * xtensa macro before including arduinoFFT, then restore after — arduinoFFT's
 * DDR ends up shadowed by the restored xtensa macro, and nobody downstream
 * sees the redefinition.
 */
#pragma push_macro("DDR")
#undef DDR
#include <arduinoFFT.h>
#pragma pop_macro("DDR")

// Set strict compile flags for the underlying library if not already declared
#ifndef FFT_SPEED_OVER_PRECISION
#define FFT_SPEED_OVER_PRECISION
#endif
#ifndef FFT_SQRT_APPROXIMATION
#define FFT_SQRT_APPROXIMATION
#endif

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>

#include "esphome/core/defines.h"

/**
 * @brief Spectrum pre-emphasis switch for musical color tracking.
 * @details If defined, it applies a parabolic attenuation curve below 200Hz to the 'v_real_' scratchpad.
 *          This prevents dominant low-frequency kicks (40Hz-60Hz) from masking the midrange, allowing
 *          pitch-trackers (Waterfall, Gravfreq) to follow vocals and melodies instead of freezing on bass beats.
 *          The pristine physical spectrum is preserved in 'magnitudes_', resulting in zero distortion for GEQ.
 *          If commented out, preprocessor completely strips this block for true zero runtime overhead.
 */
// #define PITCH_SPECTRUM_HPF

namespace esphome::music_leds {

class FFTEngine {
 public:
  /**
   * @brief Constructor allocating required vector tables for complex processing.
   * @param sample_rate Physical I2S microphone sample frequency (e.g. 22050 or 44100).
   */
  FFTEngine(uint32_t sample_rate)
      : sample_rate_(sample_rate),
        v_real_(SAMPLES_FFT, 0.0f),
        v_imag_(SAMPLES_FFT, 0.0f),
        magnitudes_(MAX_VALID_BIN, 0.0f),
        fft_(v_real_.data(), v_imag_.data(), SAMPLES_FFT, static_cast<float>(sample_rate), true) {}

  /**
   * @brief Computes forward Radix-4 FFT using dedicated Blackman-Harris windowing.
   * @param incoming_window Pointer to the raw sliding time-domain audio sample array.
   */
  void process(const float *incoming_window) {
    // Ingest sliding raw time-domain buffer into active processing registers
    std::memcpy(this->v_real_.data(), incoming_window, SAMPLES_FFT * sizeof(float));
    std::memset(this->v_imag_.data(), 0, SAMPLES_FFT * sizeof(float));

    this->max_sample_ = 0.0f;  // Max sample from FFT batch
    for (size_t i = 0; i < SAMPLES_FFT; i++) {
      // Pick our current mic sample - we take the max value from all samples that go into FFT
      // Skip extreme values - normally these are artefacts
      if ((this->v_real_[i] <= static_cast<float>(INT16_MAX - 1024)) &&
          static_cast<float>(this->v_real_[i] >= (INT16_MIN + 1024)))
        if (std::fabs(this->v_real_[i]) > this->max_sample_)
          this->max_sample_ = std::fabs(this->v_real_[i]);
    }

    // High-Accuracy Hardware Dc Blocker Filter
    // Calculates the true arithmetic mean of the current frame and subtracts it
    // with 100% precision. This eliminates the -1426.4 offset leakage before windowing.
    float dc_sum = 0.0f;
    for (size_t i = 0; i < SAMPLES_FFT; i++) {
      dc_sum += this->v_real_[i];
    }
    float exact_dc_offset = dc_sum / static_cast<float>(SAMPLES_FFT);

    for (size_t i = 0; i < SAMPLES_FFT; i++) {
      this->v_real_[i] -= exact_dc_offset;
    }

    // Remove DC offset to balance the signal envelope around zero axis
    this->fft_.dcRemoval();

    // Weigh data using the Blackman-Harris windowing algorithm.
    // Provides exceptional sideband rejection (-92dB) and narrow main lobes,
    // ensuring clean frequency separation and preventing bass from bleeding into midrange.
    this->fft_.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);

    // Compute Radix-4 Forward complex Fast Fourier Transform on the hardware FPU
    this->fft_.compute(FFTDirection::Forward);

    // Convert complex outputs to absolute magnitude coefficients (Overwrites v_real_)
    this->fft_.complexToMagnitude();

    // The remaining DC offset on the signal produces a strong spike on position 0
    // that should be eliminated to avoid issues.
    this->v_real_[0] = 0.0f;

    // Safely isolate and export computed frequencies to the persistent output array
    // We execute this step immediately to secure an un-altered physical spectrum snapshot.
    std::memcpy(this->magnitudes_.data(), this->v_real_.data(), (MAX_VALID_BIN) * sizeof(float));

#ifdef PITCH_SPECTRUM_HPF
    float hz_per_bin = static_cast<float>(this->sample_rate_) / static_cast<float>(SAMPLES_FFT);

    // Apply inline destructive Pre-emphasis to the leftover v_real_ workspace.
    // Attenuates frequency bins below 200Hz using an exponential curve to balance the spectral landscape.
    size_t max_bass_bin = static_cast<size_t>(200.0f / hz_per_bin);
    if (max_bass_bin >= (MAX_VALID_BIN)) {
      max_bass_bin = (MAX_VALID_BIN) - 1;
    }

    for (size_t b = 1; b <= max_bass_bin; b++) {
      float freq = static_cast<float>(b) * hz_per_bin;
      float linear_fraction = freq / 200.0f;
      float attenuation_curve = 0.05f + (linear_fraction * linear_fraction) * 0.95f;

      this->v_real_[b] *= attenuation_curve;
    }
#endif

    // Identify the most dominant frequency peak.
    // Evaluates the shaded scratchpad if PITCH_SPECTRUM_HPF is defined, or the native array if not.
    float major_peak_hz = 0.0f;
    float peak_magnitude = 0.0f;
    this->fft_.majorPeak(&major_peak_hz, &peak_magnitude);

    // Restrict frequency scale to standard ranges expected by visual effects engines
    float high_nyquist_bound = static_cast<float>(this->sample_rate_) / 2.0f;
    this->dominant_frequency_hz_ = std::clamp(major_peak_hz, 1.0f, high_nyquist_bound);

#ifdef PITCH_SPECTRUM_HPF
    // Calculate peak_bin and extract high-fidelity magnitude strictly when the spectrum modifier is enabled.
    size_t peak_bin = static_cast<size_t>((major_peak_hz + (hz_per_bin / 2.0f)) / hz_per_bin);
    if (peak_bin >= (MAX_VALID_BIN)) {
      peak_bin = (MAX_VALID_BIN) - 1;
    }
    // Pull the un-altered physical magnitude coefficient from our magnitudes_ backup map
    this->magnitude_ = this->magnitudes_[peak_bin];
#else
    // Zero-overhead native fallback path using the direct library output
    this->magnitude_ = peak_magnitude;
#endif
  }

  // --- Read-Only Component Data Accessors ---
  const float *magnitudes() const { return this->magnitudes_.data(); }
  float dominant_frequency_hz() const { return this->dominant_frequency_hz_; }
  float magnitude() const { return this->magnitude_; }
  float max_sample() const { return this->max_sample_; }
  size_t spectrum_size() const { return MAX_VALID_BIN; }

 protected:
  uint32_t sample_rate_{0.0f};

  std::vector<float> v_real_;
  std::vector<float> v_imag_;
  std::vector<float> magnitudes_;

  float dominant_frequency_hz_{1.0f};
  float magnitude_{0.0f};
  float max_sample_{0.0f};

  ArduinoFFT<float> fft_;
};

}  // namespace esphome::music_leds
