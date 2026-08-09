#include "fastled_helper.h"
#include "palettes.h"
#include "utils.h"

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

namespace esphome::fastled_helper {

#ifdef USE_PALETTES
void FastledHelper::set_current_palette(int index) {
#ifdef USE_MUSIC_LEDS
  if (index >= 0 && index < array_size(paletteArr) + 4)
#else
  if (index >= 0 && index < array_size(paletteArr))
#endif
  {
    current_palette = index;
  }
}  // set_script()
#endif

void FastledHelper::dump_config() {
  ESP_LOGCONFIG(TAG, "Fastled Helper version: %s", FASTLED_HELPER_VERSION);
  ESP_LOGCONFIG(TAG, "         Gamma correct: %.2f", GAMMA_CORRECT);
#ifdef USE_PALETTES
  ESP_LOGCONFIG(TAG, "              Palettes: %d", array_size(paletteArr));
#else
  ESP_LOGCONFIG(TAG, "              Palettes: No");
#endif
#ifdef USE_MUSIC_LEDS
  ESP_LOGCONFIG(TAG, "    Music Leds support: Yes");
#else
  ESP_LOGCONFIG(TAG, "    Music Leds support: No");
#endif
}  // dump_config()

}  // namespace esphome::fastled_helper
