#pragma once

namespace esphome::music_leds {

class PreAmplifier {
 public:
  /**
   * @brief Explicit constructor defining the default hardware DSP gain multiplier.
   * @param dynamic_gain Input multiplier constant matching microphone dynamic curves.
   */
  explicit PreAmplifier(float dynamic_gain = 4.5f) 
      : dynamic_gain_(dynamic_gain) {}

  /**
   * @brief Sets a new runtime gain value (perfect for future slider integration).
   */
  void set_gain(float new_gain) { this->dynamic_gain_ = new_gain; }

  /**
   * @brief Amplifies all aggregated frequency bands to restore nominal sensitivity.
   */
  void process(float &bass, float &mid, float &high) const {
    bass *= this->dynamic_gain_;
    mid  *= this->dynamic_gain_;
    high *= this->dynamic_gain_;
  }

 private:
  float dynamic_gain_{4.5f};
};

}  // namespace esphome::music_leds
