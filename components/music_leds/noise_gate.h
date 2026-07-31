#pragma once

#include "constants.h"
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

class NoiseGate {
 public:
  /**
   * @brief Explicit constructor defining the strict silence floor threshold with hardware scale calibration.
   * @param sample_scale The amplitude division factor passed from the main component (e.g., 1.0f / 24.0f).
   * @param threshold_floor Minimum weighted matrix magnitude (0.0 to 1.0) to close the gate.
   */
  explicit NoiseGate(float sample_scale, float threshold_floor = 0.10f) : sample_scale_(sample_scale) {
    this->set_threshold(threshold_floor);
  }

  /**
   * @brief Evaluates pure un-amplified hardware macro band lines prior to any AGC loops.
   * @param bass In/Out reference to the bass band energy pool.
   * @param mid In/Out reference to the midrange band energy pool.
   * @param high In/Out reference to the high frequency band energy pool.
   */
  void process(float &bass, float &mid, float &high) {
    // Clean unity scale setup: 100% immune to low-frequency clipping during loud tracks
    float dynamic_bass_cutoff = this->threshold_floor_;

    // Isolate genuine musical bass from the hardware ADC noise floor
    float clean_bass = (bass > dynamic_bass_cutoff) ? (bass - dynamic_bass_cutoff) : 0.0f;

    // Psychoacoustic weight matrix: Bass is compressed to 5% to block low-frequency lines hum,
    // while Mid is boosted to 65% to capture lightning-fast transient edges and vocal pauses.
    float physical_volume = (clean_bass * 0.05f) + (mid * 0.65f) + (high * 0.30f);

    // Determine the state of the gate using a dual-threshold hysteresis window
    if (this->gate_closed_) {
      if (physical_volume > this->threshold_open_) {
        this->gate_closed_ = false;  // Unlatch and allow signal propagation
      }
    } else {
      if (physical_volume < this->threshold_floor_) {
        this->gate_closed_ = true;  // Trigger silence latch
      }
    }

    // Zero out macro bands if the noise gate is closed.
    // This stops background room sounds and open window rumble from leaking
    // into the beat detector, completely eliminating false triggers in silence.
    if (this->gate_closed_) {
      bass = 0.0f;
      mid = 0.0f;
      high = 0.0f;
    }
  }

  /**
   * @brief Dynamic configuration injector for runtime threshold modifications with scale safety.
   */
  void set_threshold(float floor) {
    // Combined scale correction: applies sample_scale_ factor to properly normalize boundaries
    float calibrated_pcm_scale = AMPLITUDE_SCALE_16BIT * this->sample_scale_;
    if (calibrated_pcm_scale <= 0.0f)
      calibrated_pcm_scale = 1.0f;

    // Standard matrix threshold boundaries
    this->threshold_floor_ = floor * calibrated_pcm_scale;
    this->threshold_open_ = this->threshold_floor_ * 1.5f;
  }

  /**
   * @brief Direct diagnostic inspector returning the inner state of the gate.
   */
  bool is_closed() const { return this->gate_closed_; }

 private:
  float sample_scale_{0.0f};
  float threshold_floor_{0.0f};
  float threshold_open_{0.0f};
  bool gate_closed_{false};
};

}  // namespace esphome::music_leds
