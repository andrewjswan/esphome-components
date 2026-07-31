#pragma once

namespace esphome::music_leds {

enum State : uint8_t { STOPPED = 0, STARTING, RUNNING, STOPPING };

enum class FFTScalingMode : uint8_t { LINEAR = 0, LOGARITHMIC = 1, SQUARE_ROOT = 2 };

struct BandDefinition {
  size_t bin_start;
  size_t bin_end;
};

}  // namespace esphome::music_leds
