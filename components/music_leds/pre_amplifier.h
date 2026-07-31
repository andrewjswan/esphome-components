#pragma once

#include "constants.h"
#include <algorithm>
#include <cstdint>

namespace esphome::music_leds {

class PreAmplifier {
 public:
  /**
   * @brief Constructor injecting the main linear software pre-amp sensitivity gain and hardware scale factor.
   * @param sample_scale The amplitude division factor passed from the main component (e.g., 1.0f / 24.0f).
   * @param global_gain Multiplier scale defined in YAML. Defaults to a safe 1.0f unity gain.
   */
  explicit PreAmplifier(float sample_scale, float global_gain = 1.0f)
      : sample_scale_(sample_scale), global_gain_(global_gain) {
    this->calculate_constraints();
  }

  /**
   * @brief Safely balances frequency macro bands using normalized psychoacoustic curves.
   * @param bass In/Out reference for total Bass energy pool.
   * @param mid In/Out reference for total Midrange energy pool.
   * @param high In/Out reference for total High frequency energy pool.
   */
  void process(float &bass, float &mid, float &high) {
    // clang-format off
    // Accumulate normalized Pink Noise compensation scaling sub-factors per macro group.
    // Bass spans sub-bands 0 to 3
    float pink_bass =
        (PINK_NOISE_CURVE_NORM[0] + PINK_NOISE_CURVE_NORM[1] + PINK_NOISE_CURVE_NORM[2] + PINK_NOISE_CURVE_NORM[3]) *
        0.25f;

    // Mid spans sub-bands 4 to 9
    float pink_mid = (PINK_NOISE_CURVE_NORM[4] + PINK_NOISE_CURVE_NORM[5] + PINK_NOISE_CURVE_NORM[6] +
                      PINK_NOISE_CURVE_NORM[7] + PINK_NOISE_CURVE_NORM[8] + PINK_NOISE_CURVE_NORM[9]) *
                     0.16666667f;

    // High spans sub-bands 10 to 15
    float pink_high = (PINK_NOISE_CURVE_NORM[10] + PINK_NOISE_CURVE_NORM[11] + PINK_NOISE_CURVE_NORM[12] +
                       PINK_NOISE_CURVE_NORM[13] + PINK_NOISE_CURVE_NORM[14] + PINK_NOISE_CURVE_NORM[15]) *
                      0.16666667f;
    // clang-format on

    // Multiplicatively couple the dynamic pink curves with the global preamp multiplier.
    bass *= (pink_bass * this->global_gain_);
    mid *= (pink_mid * this->global_gain_);
    high *= (pink_high * this->global_gain_);

    // Scale Clamping
    // Enforces constraints adjusted directly for the current spectrum power limits.
    // Prevents clipping flat-tops on transients while securing safe upper FPU boundaries.
    bass = std::clamp(bass, 0.0f, this->max_energy_ceiling_);
    mid = std::clamp(mid, 0.0f, this->max_energy_ceiling_);
    high = std::clamp(high, 0.0f, this->max_energy_ceiling_);
  }

  void set_global_gain(float gain) { this->global_gain_ = gain; }

  /**
   * @brief Dynamic injector allowing scale factor updates on runtime parameters adjustments.
   */
  void set_sample_scale(float sample_scale) {
    this->sample_scale_ = sample_scale;
    this->calculate_constraints();
  }

 private:
  float sample_scale_;
  float global_gain_{1.0f};
  float max_energy_ceiling_{32768.0f};

  /**
   * @brief Re-calculates maximum ceiling limits once to offload the FPU main pipeline thread.
   */
  void calculate_constraints() {
    // Calibrates upper limit constraints proportional to the current sample division vector.
    if (this->sample_scale_ > 0.0f) {
      this->max_energy_ceiling_ = AMPLITUDE_SCALE_16BIT / this->sample_scale_;
    } else {
      this->max_energy_ceiling_ = AMPLITUDE_SCALE_16BIT;  // Safe fallback configuration
    }
  }
};

}  // namespace esphome::music_leds
