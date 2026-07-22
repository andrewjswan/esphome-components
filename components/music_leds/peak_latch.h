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
  }

  /**
   * @brief Combines frequency-domain beat tracking and time-domain peak volume updates into a unified latch.
   * @param is_beat Input flag derived from the statistical variance onset engine (Core 1).
   * @param raw_volume Input instantaneous overall envelope amplitude [0.0f .. 1.0f].
   * @param current_sample_peak Out reference to synchronize the unified time-locked latch state back to features.
   */
  void process(bool is_beat, float raw_volume, bool &current_sample_peak) {
    uint32_t now_ms = millis();
    bool trigger_activated = false;

    // Frequency-Domain Statistical Beat Attack
    if (is_beat && (now_ms - this->last_freq_peak_time_ > this->freq_lockout_ms_)) {
      trigger_activated = true;
      this->last_freq_peak_time_ = now_ms;
    }

    // Amplitude-Domain Volume Surge Peak
    // Evaluates dynamic filtering tracking if the raw frame envelope breaks the threshold barrier
    if (raw_volume > this->sample_max_) {
      // Smoothly adjust internal tracking register (filtering the envelope rise)
      this->sample_max_ += 0.5f * (raw_volume - this->sample_max_);
      
      // Secondary fallback logic: fires on intense un-metered loudness jumps (Matching legacy backup)
      if ((raw_volume > this->vol_threshold_) && (now_ms - this->last_vol_peak_time_ > this->vol_lockout_ms_)) {
        trigger_activated = true;
        this->last_vol_peak_time_ = now_ms;
      }
    } else {
      // Decay the maximum volume register slowly over time to reset the floor
      this->sample_max_ *= 0.985f; 
    }

    // Unified Latch Engine
    if (trigger_activated) {
      current_sample_peak = true;
      this->global_peak_time_ = now_ms;
    }

    // Automatically lower the flag once the active hold window (50ms) expires
    if (current_sample_peak && (now_ms - this->global_peak_time_ > this->hold_ms_)) {
      current_sample_peak = false;
    }
  }

 private:
  uint32_t last_freq_peak_time_{0};
  uint32_t last_vol_peak_time_{0};
  uint32_t global_peak_time_{0};

  float sample_max_{0.0f};

  // Configurable DSP pipeline configuration metrics
  uint32_t freq_lockout_ms_{100};
  uint32_t vol_lockout_ms_{80};
  uint32_t hold_ms_{50};
  float vol_threshold_{0.5f};
};

}  // namespace esphome::music_leds
