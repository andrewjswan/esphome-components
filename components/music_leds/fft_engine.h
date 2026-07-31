#pragma once

#include "constants.h"

// arduinoFFT's defs.h #defines DDR(x) as ((x)-1) for AVR-ish "data direction
// register" math. ESP-IDF's xtensa specreg.h #defines DDR as 104 (debug-data
// register address). When both headers land in the same translation unit
// (esphome.h pulls FreeRTOS->xtensa, then audio_reactive.h pulls arduinoFFT)
// the DDR redefinition emits two warnings per compile. Save / undef the
// xtensa macro before including arduinoFFT, then restore after — arduinoFFT's
// DDR ends up shadowed by the restored xtensa macro, and nobody downstream
// sees the redefinition.
#pragma push_macro("DDR")
#undef DDR
#include <arduinoFFT.h>
#pragma pop_macro("DDR")

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

#pragma once

#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <arduinoFFT.h>
#include "esphome/core/log.h"

// Set strict compile flags for the underlying library if not already declared
#ifndef FFT_SPEED_OVER_PRECISION
#define FFT_SPEED_OVER_PRECISION
#endif
#ifndef FFT_SQRT_APPROXIMATION
#define FFT_SQRT_APPROXIMATION
#endif

class FFTEngine {
 public:
  /**
   * @brief Constructor allocating required vector tables for complex processing.
   * @param sample_rate Physical I2S microphone sample frequency (e.g. 22050 or 44100).
   * @param samples_fft Total samples per processing window (Must be a power of two).
   */
  FFTEngine(uint32_t sample_rate, size_t samples_fft = 512)
      : sample_rate_(sample_rate),
        samples_fft_(samples_fft),
        v_real_(samples_fft, 0.0f),
        v_imag_(samples_fft, 0.0f),
        magnitudes_(samples_fft / 2, 0.0f),
        fft_(v_real_.data(), v_imag_.data(), samples_fft, static_cast<float>(sample_rate), true) {
  }

  /**
   * @brief Computes forward Radix-4 FFT using dedicated Blackman-Harris windowing.
   * @param incoming_window Pointer to the raw sliding time-domain audio sample array.
   */
  void process(const float *incoming_window) {
    // Ingest sliding raw time-domain buffer into active processing registers
    std::memcpy(this->v_real_.data(), incoming_window, this->samples_fft_ * sizeof(float));
    std::memset(this->v_imag_.data(), 0, this->samples_fft_ * sizeof(float));

    // High-Accuracy Hardware Dc Blocker Filter
    // Calculates the true arithmetic mean of the current frame and subtracts it
    // with 100% precision. This eliminates the -1426.4 offset leakage before windowing.
    float dc_sum = 0.0f;
    for (size_t i = 0; i < this->samples_fft_; i++) {
      dc_sum += this->v_real_[i];
    }
    float exact_dc_offset = dc_sum / static_cast<float>(this->samples_fft_);

    for (size_t i = 0; i < this->samples_fft_; i++) {
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

    // The remaining DC offset on the signal produces a strong spike on position 0 that should be eliminated to avoid issues.
    this->v_real_[0] = 0.0f;

    // Identify the most dominant frequency peak and its absolute magnitude value.
    float major_peak_hz = 0.0f;
    float peak_magnitude = 0.0f;
    this->fft_.majorPeak(&major_peak_hz, &peak_magnitude);
    this->magnitude_ = peak_magnitude;

    // Restrict frequency scale to standard ranges expected by visual effects engines
    float high_nyquist_bound = static_cast<float>(this->sample_rate_) / 2.0f;
    this->dominant_frequency_hz_ = std::clamp(major_peak_hz, 1.0f, high_nyquist_bound);

    // Safely isolate and export computed frequencies to the persistent output array
    std::memcpy(this->magnitudes_.data(), this->v_real_.data(), (this->samples_fft_ / 2) * sizeof(float));
  }

  // --- Read-Only Component Data Accessors ---
  const float* magnitudes() const { return this->magnitudes_.data(); }
  float dominant_frequency_hz() const { return this->dominant_frequency_hz_; }
  float magnitude() const { return this->magnitude_; }
  size_t spectrum_size() const { return this->samples_fft_ / 2; }

 protected:
  uint32_t sample_rate_;
  size_t samples_fft_;

  std::vector<float> v_real_;
  std::vector<float> v_imag_;
  std::vector<float> magnitudes_;

  float dominant_frequency_hz_{1.0f};
  float magnitude_{0.0f};

  ArduinoFFT<float> fft_;
};

} // namespace esphome::music_leds
