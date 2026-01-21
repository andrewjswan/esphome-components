#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome {
namespace nerdminer {

static const char *const TAG = "nerdminer";
static const char *const NERDMINER_VERSION = "2025.7.1-1.7.0";

class NerdMiner : public Component
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

  bool getMinerState();
  uint32_t getTotalHashes();
  uint32_t getBlockTemplates();
  double getBestDiff();
  uint32_t get32BitShares();
  uint64_t getHores();
  uint32_t getValidBlocks();
  double getHashrate();
  uint32_t getKHashes();

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

 protected:
  TaskHandle_t stratum_handle = nullptr;
  TaskHandle_t monitor_handle = nullptr;
  TaskHandle_t miner1_handle = nullptr;
  TaskHandle_t miner2_handle = nullptr;

};  // NerdMiner

}  // namespace nerdminer
}  // namespace esphome
