#pragma once

#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

class BeatDetector {
 public:
  // Acoustic boundaries for kick drum transient analysis fixed at compile-time
  static constexpr float MIN_KICK_HZ = 40.0f;  // 60.0f;
  static constexpr float MAX_KICK_HZ = 130.0f;
  static constexpr float BASS_NOISE_FLOOR = 300.0f;  // 1500.0f;

  /**
   * @param sample_rate Physical pipeline sampling frequency configured during initialization.
   * @param sensitivity Standard UI input slider value mapped to the adaptive onset multiplier.
   */
  explicit BeatDetector(float sample_rate, int sensitivity = 65) : sample_rate_(sample_rate) {
    this->set_sensitivity(sensitivity);
    this->reset();
  }

  /**
   * @brief Evaluates raw FFT magnitudes to isolate sharp sub-bass transient energy bursts.
   * @param raw_fft_magnitudes Pointer to the unfiltered linear frequency magnitude spectrum array.
   * @return True for exactly ONE audio processing frame when a valid onset breach occurs.
   */
  bool process(const float *raw_fft_magnitudes) {
    // Calculate frequency resolution per individual FFT spectral line
    float hz_per_bin = this->sample_rate_ / static_cast<float>(SAMPLES_FFT);

    // Map physical frequency limits directly to exact discrete FFT bin indices
    uint16_t start_bin = std::max(1, static_cast<int>(MIN_KICK_HZ / hz_per_bin));
    uint16_t end_bin = static_cast<int>(MAX_KICK_HZ / hz_per_bin);
    if (end_bin >= (MAX_VALID_BIN)) {
      end_bin = (MAX_VALID_BIN) -1;
    }

    // Integrate total energy contained strictly within the targeted bass spectrum corridor
    float current_bass_energy = 0.0f;
    for (uint16_t i = start_bin; i <= end_bin; i++) {
      current_bass_energy += raw_fft_magnitudes[i];
    }

    // Hardware Noise Floor Protection: bypass analysis if the accumulated signal is silent
    if (current_bass_energy < BASS_NOISE_FLOOR) {
      this->history_envelope_ = (current_bass_energy * 0.10f) + (this->history_envelope_ * 0.90f);
      return false;
    }

    bool instant_beat_triggered = false;

    // Compute dynamic adaptive activation threshold: Rolling Background History * Sensitivity Factor
    float dynamic_threshold = this->history_envelope_ * this->multiplier_;

    // Evaluate transient onset attack condition
    if (current_bass_energy > dynamic_threshold) {
      instant_beat_triggered = true;

      // Dynamic Latch: clamp the baseline history index directly to the peak magnitude.
      // Acts as an immediate acoustic brake to prevent bounce multi-triggering on the wave crest.
      this->history_envelope_ = current_bass_energy;
    } else {
      // Exponential moving average tracking governed by acoustic membrane decay profiles
      if (current_bass_energy > this->history_envelope_) {
        // Fast tracking pass: adapt threshold baseline quickly during non-breaching volume rises
        this->history_envelope_ = (current_bass_energy * 0.20f) + (this->history_envelope_ * 0.80f);
      } else {
        // Slow decay memory pass: maintain elevated threshold values while the bass note attenuates
        this->history_envelope_ = (current_bass_energy * 0.04f) + (this->history_envelope_ * 0.96f);
      }
    }

    return instant_beat_triggered;
  }

  /**
   * @brief Inversely maps the standard linear UI range into a tight exponential scaling coefficient.
   */
  void set_sensitivity(int value) {
    int clamped = std::clamp(value, 1, 100);
    // Maps UI [1..100] to a proportional background envelope multiplier [2.20x down to 1.08x]
    this->multiplier_ = 2.20f - ((clamped / 100.0f) * 1.12f);
  }

  void reset() { this->history_envelope_ = 500.0f; }

 private:
  float sample_rate_{0.0f};
  float multiplier_{1.25f};

  // Persistent Single-Pole IIR memory tracking register for the baseline sound profile
  float history_envelope_{500.0f};
};

}  // namespace esphome::music_leds
