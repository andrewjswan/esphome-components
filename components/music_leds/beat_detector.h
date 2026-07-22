#pragma once

#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>

#include "esphome/core/defines.h"

namespace esphome::music_leds {

class BeatDetector {
 public:
  /**
   * @brief Initializes the rolling statistical window for bass onset tracking.
   * @param sensitivity Sensitivity slider 1-100 (higher = triggers more easily).
   * @param min_interval_ms Minimum time lock-out between hits to eliminate flicker.
   */
  explicit BeatDetector(int sensitivity = 65, uint32_t min_interval_ms = 160)
      : min_interval_ms_(min_interval_ms) {
    this->set_sensitivity(sensitivity);
    this->reset();
  }

  /**
   * @brief Analyzes the fresh normalized bass energy using rolling standard deviation.
   * @param normalized_bass Clean, AGC-scaled bass band energy [0.0f .. 1.0f].
   * @return True for exactly ONE frame when a valid rhythmic hit is isolated.
   */
  bool process(float normalized_bass) {
    uint32_t timestamp_ms = millis(); // Dynamic runtime scheduling clock tracking

    // 1. Manage circular buffer accumulation mechanics and incremental statistics
    if (this->history_count_ >= WINDOW_SIZE) {
      float evicted_value = this->history_ring_[this->history_head_];
      this->history_sum_ -= evicted_value;
      this->history_sq_sum_ -= evicted_value * evicted_value;
    }

    this->history_ring_[this->history_head_] = normalized_bass;
    this->history_sum_ += normalized_bass;
    this->history_sq_sum_ += normalized_bass * normalized_bass;
    this->history_head_ = (this->history_head_ + 1) % WINDOW_SIZE;

    if (this->history_count_ < WINDOW_SIZE) {
      this->history_count_++;
    }

    // 2. Compute dynamic background noise threshold using mean and standard deviation
    if (this->history_count_ < WINDOW_SIZE / 2) {
      return false; // Skip execution until history data buffer is sufficiently warmed up
    }

    float n = static_cast<float>(this->history_count_);
    float mean = this->history_sum_ / n;
    float variance = (this->history_sq_sum_ / n) - (mean * mean);
    float std_dev = sqrtf(std::max(0.0f, variance));
    std_dev = std::max(std_dev, mean * 0.1f); // Establish floor constraints

    // The core absent42 psychoacoustic trigger threshold boundary formula
    float threshold = mean + (this->multiplier_ * std_dev);

    // 3. Evaluate trigger conditions with hysteresis and temporal lockouts
    bool interval_ok = (this->last_onset_ms_ == 0) || ((timestamp_ms - this->last_onset_ms_) >= this->min_interval_ms_);
    bool triggered = false;

    if (normalized_bass > threshold && interval_ok && this->hysteresis_armed_) {
      triggered = true;
      this->hysteresis_armed_ = false; // Disarm immediately upon beat confirmation
    } else if (normalized_bass < threshold * 0.7f) {
      this->hysteresis_armed_ = true;  // Rearm safely only when energy drops below 70% threshold
    }

    if (triggered) {
      this->last_onset_ms_ = timestamp_ms;
    }

    return triggered;
  }

  /**
   * @brief Maps standard 1-100 UI sensitivity to internal mathematical scaling triggers.
   */
  void set_sensitivity(int value) {
    int clamped = std::max(1, std::min(100, value));
    // Maps 1-100 into multipliers 3.0f (low sensitivity) down to 0.5f (high sensitivity)
    this->multiplier_ = 3.0f - (static_cast<float>(clamped) / 100.0f) * 2.5f;
  }

  void reset() {
    this->history_count_ = 0;
    this->history_head_ = 0;
    this->history_sum_ = 0.0f;
    this->history_sq_sum_ = 0.0f;
    std::memset(this->history_ring_, 0, sizeof(this->history_ring_));
    this->last_onset_ms_ = 0;
    this->hysteresis_armed_ = true;
  }

 private:
  uint32_t min_interval_ms_;
  uint32_t last_onset_ms_{0};
  float multiplier_{1.5f};
  bool hysteresis_armed_{true};

  // Fixed fixed-size internal array tracking ~3 seconds of execution timeline history
  static constexpr size_t WINDOW_SIZE = 60;
  float history_ring_[WINDOW_SIZE];
  size_t history_count_{0};
  size_t history_head_{0};
  float history_sum_{0.0f};
  float history_sq_sum_{0.0f};
};

} // namespace esphome::music_leds
