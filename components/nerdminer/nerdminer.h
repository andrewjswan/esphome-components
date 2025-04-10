#pragma once

#include "esphome/core/component.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace esphome {
namespace nerdminer {

static const char *const TAG = "nerdminer";
static const char *const NERDMINER_VERSION = "2025.4.5-1.7.0";

class NerdMiner : public Component {
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

 protected:
  TaskHandle_t stratum_handle = nullptr;
  TaskHandle_t monitor_handle = nullptr;
  TaskHandle_t miner1_handle = nullptr;
  TaskHandle_t miner2_handle = nullptr;

};  // NerdMiner

}  // namespace nerdminer
}  // namespace esphome
