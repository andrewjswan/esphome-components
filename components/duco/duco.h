#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome::duco {

static const char *const TAG = "duco";
static const char *const DUCO_VERSION = "2026.6.1";

class Duco : public Component
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

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

 protected:
  TaskHandle_t miner1_handle = nullptr;
  TaskHandle_t miner2_handle = nullptr;

  static void duco_function(void *params);
};  // Duco

}  // namespace esphome::duco
