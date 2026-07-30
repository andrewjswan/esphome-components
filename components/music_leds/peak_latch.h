#pragma once

#include <algorithm>

namespace esphome::music_leds {

class PeakLatch {
 public:
  /**
   * @brief Explicit constructor to define timing windows and the volume jump threshold.
   */
  explicit PeakLatch(uint32_t frequency_lockout_ms = 100, 
                     uint32_t volume_lockout_ms = 80, 
                     uint32_t hold_ms = 50,
                     float volume_threshold = 0.5f)
      : freq_lockout_ms_(frequency_lockout_ms),
        vol_lockout_ms_(volume_lockout_ms),
        hold_ms_(hold_ms),
        vol_threshold_(volume_threshold) {
    this->last_freq_peak_time_ = millis();
    this->last_vol_peak_time_ = millis();
    this->global_peak_time_ = millis();
    this->last_execution_time_ = micros();
  }

  /**
   * @brief Combines frequency-domain beat tracking and time-domain peak volume updates into a unified latch.
   * @param is_beat Input flag derived from the statistical variance onset engine (Core 1).
   * @param raw_volume Input calibrated post-AGC overall envelope amplitude [0.0f .. 1.0f].
   * @param current_sample_peak Out reference to synchronize the unified time-locked latch state back to features.
   */
  void process(bool is_beat, float raw_volume, bool &current_sample_peak) {
    uint32_t now_ms = millis();
    
    // Calculate accurate delta time for time-locked envelope decay
    uint32_t now_us = micros();
    float delta_ms = static_cast<float>(now_us - this->last_execution_time_) / 1000.0f;
    this->last_execution_time_ = now_us;
    if (delta_ms > 200.0f) delta_ms = 20.0f;

    bool trigger_activated = false;

    // 1. Frequency-Domain Statistical Beat Attack
    if (is_beat && (now_ms - this->last_freq_peak_time_ >= this->freq_lockout_ms_)) {
      trigger_activated = true;
      this->last_freq_peak_time_ = now_ms;
    }

    // 2. Amplitude-Domain Volume Surge Peak (True Peak Envelope Follower)
    if (raw_volume > this->sample_max_) {
      this->sample_max_ = raw_volume; // Instant attack latch for transients
    } else {
      // Time-locked exponential decay toward zero to adapt to song dynamics
      // At ~100Hz processing rate, decay constant yields stable peak holding
      float decay_factor = delta_ms / 300.0f; // 300ms structural discharge window
      this->sample_max_ -= decay_factor * this->sample_max_;
      if (this->sample_max_ < 0.0f) this->sample_max_ = 0.0f;
    }

    // Secondary fallback logic: fires on intense loudness jumps breaching the threshold barrier
    if (raw_volume > this->vol_threshold_ && (raw_volume >= this->sample_max_ * 0.9f)) {
      if (now_ms - this->last_vol_peak_time_ >= this->vol_lockout_ms_) {
        trigger_activated = true;
        this->last_vol_peak_time_ = now_ms;
      }
    }

    // 3. Unified Latch Engine State Machine
    if (trigger_activated) {
      current_sample_peak = true;
      this->global_peak_time_ = now_ms;
      ESP_LOGVV("music_leds.latch", "Peak Latch Engaged! Source: %s", is_beat ? "FREQ" : "AMPLITUDE");
    }

    // Automatically lower the flag once the active hold window (50ms) expires
    if (current_sample_peak && (now_ms - this->global_peak_time_ >= this->hold_ms_)) {
      current_sample_peak = false;
    }
  }

 private:
  uint32_t last_freq_peak_time_{0};
  uint32_t last_vol_peak_time_{0};
  uint32_t global_peak_time_{0};
  uint32_t last_execution_time_{0};

  float sample_max_{0.0f};

  // Configurable DSP pipeline configuration metrics
  uint32_t freq_lockout_ms_{100};
  uint32_t vol_lockout_ms_{80};
  uint32_t hold_ms_{50};
  float vol_threshold_{0.5f};
};

}  // namespace esphome::music_leds
