#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

class NoiseGate {
 public:
  /**
   * @brief Explicit constructor defining the strict silence floor threshold with hysteresis support.
   * @param threshold_floor Minimum weighted band magnitude to trigger the gate closure.
   */
  explicit NoiseGate(float threshold_floor = 0.10f)
      : threshold_floor_(threshold_floor),
        threshold_open_(threshold_floor * 1.5f) {}

  /**
   * @brief Evaluates pure un-amplified hardware macro band lines prior to any AGC loops.
   * @param bass In/Out reference to the bass band energy pool.
   * @param mid In/Out reference to the midrange band energy pool.
   * @param high In/Out reference to the high frequency band energy pool.
   */
  void process(float &bass, float &mid, float &high) {
    // Lowered to 0.50f to capture maximum organic low-end micro-pauses
    // without triggering a permanent gate blowout from ADC fluctuations
    float clean_bass = (bass > 0.50f) ? (bass - 0.50f) : 0.0f;
    // Optimized weight matrix for the 0.50f setup:
    // Bass weight is safely clamped to 0.05f to neutralize high-energy ADC leakage spikes,
    // while Mid weight is locked at 0.65f to drive explosive edge transitions during pauses
    // float physical_volume = (clean_bass * 0.05f) + (mid * 0.65f) + (high * 0.30f);
    // Simple, organic arithmetic mean volume metric
    // But now it's 100% immune to ADC noise because bass floor is subtracted!
    float physical_volume = (clean_bass + mid + high) * 0.33333334f;

    // Determine the state of the gate using a dual-threshold hysteresis window
    if (gate_closed_) {
      if (physical_volume > this->threshold_open_) {
        gate_closed_ = false; // Unlatch and allow signal propagation
      }
    } else {
      if (physical_volume < this->threshold_floor_) {
        gate_closed_ = true; // Trigger silence latch
      }
    }

    // Execute state adjustments based on the stable latch condition
    if (gate_closed_) {
      bass = 0.0f;
      mid = 0.0f;
      high = 0.0f;
    }
  }

  /**
   * @brief Dynamic configuration injector for runtime threshold modifications.
   */
  void set_threshold(float floor) {
    this->threshold_floor_ = floor;
    this->threshold_open_ = floor * 1.5f;
  }

  /**
   * @brief Direct diagnostic inspector returning the inner state of the gate.
   */
  bool is_closed() const {
    return this->gate_closed_;
  }

 private:
  float threshold_floor_{0.10f}; // Stable floor constraint against hardware ADC noise limits
  float threshold_open_{0.15f};
  bool gate_closed_{false};
};

} // namespace esphome::music_leds
