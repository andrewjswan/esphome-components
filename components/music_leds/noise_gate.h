#pragma once

#include <algorithm>

namespace esphome::music_leds {

class NoiseGate {
 public:
  /**
   * @brief Explicit constructor defining the strict silence floor threshold.
   * @param threshold_floor Minimum raw volume magnitude to allow signal propagation.
   */
  explicit NoiseGate(float threshold_floor = 0.05f) 
      : threshold_floor_(threshold_floor) {}

  /**
   * @brief Evaluates the frame metrics and enforces structural silence if beneath the floor.
   * @param raw_vol In/Out reference to the frame volume.
   * @param smoothed_vol In/Out reference to the filtered global volume.
   * @param bass In/Out reference to the bass band energy pool.
   * @param mid In/Out reference to the midrange band energy pool.
   * @param high In/Out reference to the high frequency band energy pool.
   */
  void process(float &raw_vol, float &smoothed_vol, float &bass, float &mid, float &high, bool &is_beat) {
    // Check if the current frame volume drops below the ambient noise floor
    if (raw_vol < this->threshold_floor_) {
      raw_vol = 0.0f;
      smoothed_vol = 0.0f;
      bass = 0.0f;
      mid = 0.0f;
      high = 0.0f;
      is_beat = false;
    }
  }

 private:
  float threshold_floor_{0.05f};
};

}  // namespace esphome::music_leds

