#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

static const char *const FASTLED_HELPER_VERSION = "2025.10.1";
static const char *const TAG = "fastled_helper";

class FastledHelper : public Component {
 public:
  void dump_config() override;

#ifdef PALETTES
  void set_current_palette(int index);
#endif
};  // FastledHelper

}  // namespace esphome::fastled_helper
