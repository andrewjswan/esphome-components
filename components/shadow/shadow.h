#pragma once

#include "esphome/core/component.h"
#include "esphome/components/script/script.h"

namespace esphome {
namespace shadow {

static const char *const SHADOW_VERSION = "2025.7.1";
static const char *const TAG = "shadow";

class Shadow : public Component {
 public:
  void setup() override;
  void start();
  void stop();

  void dump_config() override;
  void set_script(script::Script<> *script);
  void set_shadow_interval(uint8_t shadow_interval) { this->shadow_interval_ = shadow_interval; }

 protected:
  TaskHandle_t shadow_handle{nullptr};
  script::Script<> *script{nullptr};
  uint8_t shadow_interval_{60};

  static void shadow_function(void *params);
};  // Shadow

}  // namespace shadow
}  // namespace esphome
