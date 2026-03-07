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
static const char *const NERDMINER_VERSION = "2026.3.1-1.7.0";

class NerdMiner : public Component
#ifdef USE_OTA_STATE_LISTENER
    ,
                  public ota::OTAGlobalStateListener
#endif
{
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override;
  void start();
  void stop();

  void dump_config() override;

  void set_wallet_id(const char *wallet_id) { this->wallet_id_ = wallet_id; }
  void set_worker_name(const char *worker_name) { this->worker_name_ = worker_name; }
  void set_pool(const char *pool) { this->pool_ = pool; }
  void set_pool_pass(const char *pool_pass) { this->pool_pass_ = pool_pass; }
  void set_pool_port(uint16_t pool_port) { this->pool_port_ = pool_port; }

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
  TaskHandle_t stratum_handle{nullptr};
  TaskHandle_t monitor_handle{nullptr};
  TaskHandle_t miner0_handle{nullptr};
  TaskHandle_t miner1_handle{nullptr};

  const char *wallet_id_{nullptr};
  const char *worker_name_{nullptr};
  const char *pool_{nullptr};
  const char *pool_pass_{nullptr};
  uint16_t pool_port_{0};

};  // NerdMiner

}  // namespace nerdminer
}  // namespace esphome
