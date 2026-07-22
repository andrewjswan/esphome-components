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

class FFTEngine {
 public:
  explicit FFTEngine(uint32_t sample_rate)
      : sample_rate_(sample_rate),
        v_real_(SAMPLES_FFT, 0.0f),
        v_imag_(SAMPLES_FFT, 0.0f),
        magnitudes_(SAMPLES_FFT / 2, 0.0f),
        fft_(v_real_.data(), v_imag_.data(), SAMPLES_FFT, static_cast<float>(sample_rate), true) {
  }

  void process(const float *incoming_window) {
    std::memcpy(this->v_real_.data(), incoming_window, SAMPLES_FFT * sizeof(float));
    std::memset(this->v_imag_.data(), 0, SAMPLES_FFT * sizeof(float));

    // Remove DC offset to balance the signal envelope around zero axis
    this->fft_.dcRemoval();

    // Weigh data using "Flat Top" function for optimal amplitude accuracy
    this->fft_.windowing(FFTWindow::Flat_top, FFTDirection::Forward);

    // Compute Radix-4 Forward complex Fast Fourier Transform
    this->fft_.compute(FFTDirection::Forward);

    // Convert complex outputs to absolute voltage magnitude coefficients
    this->fft_.complexToMagnitude();

    // Eliminate the persistent DC offset spike on position 0 to avoid artifacts
    this->v_real_[0] = 0.0f;

    // Identify the most dominant frequency peak and its magnitude value
    float major_peak_hz = 0.0f;
    float peak_magnitude = 0.0f;
    this->fft_.majorPeak(&major_peak_hz, &peak_magnitude);

    // Restrict value to range expected by visual effects engines [1.0f .. 11025.0f]
    this->dominant_frequency_hz_ = std::clamp(major_peak_hz, 1.0f, 11025.0f);
    // this->fft_magnitude_ = std::abs(peak_magnitude);

    // Safely isolate and export computed frequencies to the persistent output array
    std::memcpy(this->magnitudes_.data(), this->v_real_.data(), (SAMPLES_FFT / 2) * sizeof(float));
  }

  const float* magnitudes() const { return this->magnitudes_.data(); }
  float dominant_frequency_hz() const { return this->dominant_frequency_hz_; }

 protected:
  uint32_t sample_rate_;

  std::vector<float> v_real_;
  std::vector<float> v_imag_;

  std::vector<float> magnitudes_;
  float dominant_frequency_hz_{1.0f};

  ArduinoFFT<float> fft_;
};

} // namespace esphome::music_leds
