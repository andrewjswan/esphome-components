#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

static const char *const FASTLED_HELPER_VERSION = "2026.7.5";
static const char *const TAG = "fastled_helper";

class FastledHelper final : public Component {
 public:
  void dump_config() override;

#ifdef PALETTES
  void set_current_palette(int index);
#endif
};  // FastledHelper

}  // namespace esphome::fastled_helper
