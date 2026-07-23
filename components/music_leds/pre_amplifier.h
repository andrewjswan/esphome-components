#pragma once

#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

// Normalized Pink Noise Compensation Table
// Scaled down by 1.70f to enforce a true 1.0f baseline floor for the Bass band,
// ensuring no artificial over-amplification or hard clipping occurs in silence.
static constexpr float FFT_RESULT_PINK[] = {
  1.0000f, 1.0058f, 1.0176f, 1.0470f, 0.9882f, 0.9176f, 0.9117f, 0.9588f,
  1.0529f, 0.9529f, 1.0588f, 1.2117f, 1.4529f, 1.9705f, 4.0176f, 5.6176f
};

class PreAmplifier {
 public:
  /**
   * @brief Constructor injecting the main linear software pre-amp sensitivity gain.
   * @param global_gain Multiplier scale defined in YAML. Defaults to a safe 1.0f unity gain.
   */
  explicit PreAmplifier(float global_gain = 1.0f)
      : global_gain_(global_gain) {}

  /**
   * @brief Safely balances frequency macro bands using normalized psychoacoustic curves.
   * @param bass In/Out reference for total Bass energy pool.
   * @param mid In/Out reference for total Midrange energy pool.
   * @param high In/Out reference for total High frequency energy pool.
   */
  void process(float &bass, float &mid, float &high) {
    // Accumulate normalized Pink Noise compensation scaling sub-factors per macro group.
    // Bass spans sub-bands 0 to 3
    float pink_bass = (FFT_RESULT_PINK[0] + FFT_RESULT_PINK[1] + FFT_RESULT_PINK[2] + FFT_RESULT_PINK[3]) * 0.25f;

    // Mid spans sub-bands 4 to 9
    float pink_mid  = (FFT_RESULT_PINK[4] + FFT_RESULT_PINK[5] + FFT_RESULT_PINK[6] +
                       FFT_RESULT_PINK[7] + FFT_RESULT_PINK[8] + FFT_RESULT_PINK[9]) * 0.16666667f;

    // High spans sub-bands 10 to 15
    float pink_high = (FFT_RESULT_PINK[10] + FFT_RESULT_PINK[11] + FFT_RESULT_PINK[12] +
                       FFT_RESULT_PINK[13] + FFT_RESULT_PINK[14] + FFT_RESULT_PINK[15]) * 0.16666667f;

    // Multiatively couple the dynamic pink curves with the global preamp multiplier.
    // Unity default (1.0f) guarantees absolute fidelity matching raw room acoustics.
    bass *= (pink_bass * this->global_gain_);
    mid  *= (pink_mid  * this->global_gain_);
    high *= (pink_high * this->global_gain_);

    // Enforce structural safety boundaries [0.0f .. 1.0f] prior to entering the Dynamics Engine
    bass = std::clamp(bass, 0.0f, 1.0f);
    mid  = std::clamp(mid,  0.0f, 1.0f);
    high = std::clamp(high, 0.0f, 1.0f);
  }

  void set_global_gain(float gain) {
    this->global_gain_ = gain;
  }

 private:
  float global_gain_{1.0f};
};

} // namespace esphome::music_leds
