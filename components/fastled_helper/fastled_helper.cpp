#include "fastled_helper.h"

#include "esphome/core/defines.h"
#include "esphome/core/log.h"

namespace esphome::fastled_helper {

#ifdef PALETTES
void FastledHelper::set_current_palette(int index) {
#ifdef MUSIC_LEDS
  if (index >= 0 && index < ARRAY_SIZE(paletteArr) + 4)
#else
  if (index >= 0 && index < ARRAY_SIZE(paletteArr))
#endif
  {
    current_palette = index;
  }
}  // set_script()
#endif

void FastledHelper::dump_config() {
  ESP_LOGCONFIG(TAG, "Fastled Helper version: %s", FASTLED_HELPER_VERSION);
#ifdef PALETTES
  ESP_LOGCONFIG(TAG, "              Palettes: %d", ARRAY_SIZE(paletteArr));
#else
  ESP_LOGCONFIG(TAG, "              Palettes: No");
#endif
#ifdef MUSIC_LEDS
  ESP_LOGCONFIG(TAG, "    Music Leds support: Yes");
#else
  ESP_LOGCONFIG(TAG, "    Music Leds support: No");
#endif
}  // dump_config()

}  // namespace esphome::fastled_helper
