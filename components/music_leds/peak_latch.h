#pragma once

#include "constants.h"

#include <cstdint>
#include <algorithm>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

class PeakLatch {
 public:
  // Standard duration window to hold the peak active for asynchronous rendering loops
  static constexpr uint32_t LATCH_TIME_MS = 50;

  // Hard window barrier to block repetitive machine-gun multi-triggering
  static constexpr uint32_t MIN_LOCKOUT_MS = 110;

  explicit PeakLatch() { this->reset(); }

  /**
   * @brief Evaluates multi-domain transient bursts and stretches them into timed windows.
   * @param instant_beat Clean single-frame frequency-domain trigger from BeatDetector.
   * @param raw_volume Normalized current frame volume [0.0f .. 1.0f].
   * @param smoothed_volume Normalized smoothed AGC background baseline volume [0.0f .. 1.0f].
   * @param[out] output_sample_peak Main pipeline target state reference (features_.sample_peak).
   */
  void process(bool instant_beat, float raw_volume, float smoothed_volume, bool &output_sample_peak) {
    uint32_t current_time = millis();

    // Evaluate frequency-domain indicator from the Bass Band aggregation pass
    if (instant_beat) {
      this->is_active_ = true;
      this->last_trigger_ms_ = current_time;
    }

    // Evaluate amplitude-domain differential delta burst prior to non-linear distortion
    float amplitude_delta = raw_volume - smoothed_volume;
    bool lockout_expired = (this->last_trigger_ms_ == 0) || ((current_time - this->last_trigger_ms_) >= MIN_LOCKOUT_MS);

    // 0.25f marks a severe 25% instantaneous volume surge above the normalized AGC baseline track
    if (amplitude_delta > 0.25f && lockout_expired) {
      this->is_active_ = true;
      this->last_trigger_ms_ = current_time;
    }

    // Manage the temporal Pulse Stretcher window decay
    if (this->is_active_) {
      if (current_time - this->last_trigger_ms_ > LATCH_TIME_MS) {
        this->is_active_ = false;  // Gracefully shut down the latch after 50ms has elapsed
      }
    }

    // Write back the unified stabilized flag straight to the pipeline features register
    output_sample_peak = this->is_active_;
  }

  void reset() {
    this->last_trigger_ms_ = 0;
    this->is_active_ = false;
  }

 private:
  uint32_t last_trigger_ms_{0};
  bool is_active_{false};
};

}  // namespace esphome::music_leds
