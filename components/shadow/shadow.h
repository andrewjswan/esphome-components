#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/script/script.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

namespace esphome {
namespace shadow {

static const char *const SHADOW_VERSION = "2025.7.1";
static const char *const TAG = "shadow";

class Shadow : public Component
#ifdef USE_OTA_STATE_LISTENER
    ,
               public ota::OTAGlobalStateListener
#endif
{
 public:
  void setup() override;
  void start();
  void stop();

  void dump_config() override;
  void set_script(script::Script<> *script);
  void set_shadow_interval(uint8_t shadow_interval) { this->shadow_interval_ = shadow_interval; }

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

protected:
  TaskHandle_t shadow_handle{nullptr};
  script::Script<> *script{nullptr};
  uint8_t shadow_interval_{60};

  static void shadow_function(void *params);
};  // Shadow

}  // namespace shadow
}  // namespace esphome
