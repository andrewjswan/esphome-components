#pragma once

#include "constants.h"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

// #define DEBUG

#ifdef DEBUG
#include "debug.h"
#endif

namespace esphome::music_leds {

class BeatDetector {
 public:
  /**
   * @brief Initializes the rolling statistical window for bass onset tracking.
   * @param sample_scale The amplitude division factor passed from the main component (e.g., 1.0f / 24.0f)
   * @param sensitivity Sensitivity slider 1-100 (higher = triggers more easily).
   * @param min_interval_ms Minimum time lock-out between hits to eliminate flicker.
   */
  explicit BeatDetector(float sample_scale, int sensitivity = 65, uint32_t min_interval_ms = 160)
      : sample_scale_(sample_scale), min_interval_ms_(min_interval_ms) {
    this->set_sensitivity(sensitivity);
    this->reset();
  }
  /**
   * @brief Analyzes the fresh normalized bass energy using rolling standard deviation.
   * @param raw_bass Pure, sharp, un-smoothed physical bass energy
   * @return True for exactly ONE frame when a valid rhythmic hit is isolated.
   */
  bool process(float raw_bass) {
    uint32_t timestamp_ms = millis(); // Dynamic runtime scheduling clock tracking

    float ref_max_bass = AMPLITUDE_SCALE_16BIT * this->sample_scale_;
    if (ref_max_bass <= 0.0f) ref_max_bass = 1.0f;
    float normalized_bass = raw_bass / ref_max_bass;

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("BEAT", "Input: Raw Bass: %.2f Normalized Bass: %.2f", raw_bass, normalized_bass);
    }
#endif

    // Manage circular buffer accumulation mechanics and incremental statistics
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

    // Compute dynamic background noise threshold using mean and standard deviation
    if (this->history_count_ < WINDOW_SIZE / 2) {
      return false; // Skip execution until history data buffer is sufficiently warmed up
    }

    // Optimization: replace divisions with a single hardware multiplication inverse
    float inv_n = 1.0f / static_cast<float>(this->history_count_);
    float mean = this->history_sum_ * inv_n;
    float variance = (this->history_sq_sum_ * inv_n) - (mean * mean);
    float std_dev = sqrtf(std::max(0.0f, variance));
    std_dev = std::max(std_dev, mean * 0.1f); // Establish baseline structural variance floor

    // Core psychoacoustic trigger threshold with a strict global minimum constraint (0.08f).
    // This absolute floor prevents false ghost triggers when the buffer clears to 0.0f during silence.
    float threshold = std::max(mean + (this->multiplier_ * std_dev), 0.08f);

    // Evaluate trigger conditions with hysteresis and temporal lockouts
    bool interval_ok = (this->last_onset_ms_ == 0) || ((timestamp_ms - this->last_onset_ms_) >= this->min_interval_ms_);
    bool triggered = false;

    if (normalized_bass > threshold && interval_ok && this->hysteresis_armed_) {
      triggered = true;
      this->hysteresis_armed_ = false; // Disarm immediately upon beat confirmation
#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("BEAT", "Beat Detected! Bass: %.2f, Threshold: %.2f", normalized_bass, threshold);
    }
#endif
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
    this->last_onset_ms_ = 0;
    this->hysteresis_armed_ = true;
    std::memset(this->history_ring_, 0, sizeof(this->history_ring_));
  }

 private:
  float sample_scale_{0.0f};
  uint32_t min_interval_ms_{0};
  uint32_t last_onset_ms_{0};
  float multiplier_{1.5f};
  bool hysteresis_armed_{true};

  // Fixed fixed-size internal array tracking execution timeline history
  static constexpr size_t WINDOW_SIZE = 60;
  float history_ring_[WINDOW_SIZE];
  size_t history_count_{0};
  size_t history_head_{0};
  float history_sum_{0.0f};
  float history_sq_sum_{0.0f};
};

} // namespace esphome::music_leds
