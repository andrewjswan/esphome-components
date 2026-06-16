#pragma once

#include "mining.h"

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>

namespace esphome::duco {

static const char *const TAG = "duco";
static const char *const DUCO_VERSION = "2026.6.3";

struct MiningConfig {
    std::string DUCO_USER = "";
    std::string RIG_IDENTIFIER = "";
    std::string MINER_KEY = "";
    std::string MINER_VER = DUCO_VERSION;
    uint16_t WALLET_ID = 0;

    std::string chip_id = "";
    std::string node_id = "";
    std::string host = "";
    uint16_t port = 0;

    std::atomic<bool> is_ready{false};

    MiningConfig(std::string DUCO_USER, std::string RIG_IDENTIFIER, std::string MINER_KEY)
            : DUCO_USER(DUCO_USER), RIG_IDENTIFIER(RIG_IDENTIFIER), MINER_KEY(MINER_KEY) {}
};

class Duco : public Component
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
  void loop() override;

  void dump_config() override;

#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) override;
#endif

  void on_share_found_callback();

  template<typename F> void add_on_share_found_callback(F &&callback) {
    this->share_found_callback.add(std::forward<F>(callback));
  }

  bool getMinerState();
  std::string getPool();
  uint32_t getHashRate();
  uint32_t getTotalShares();
  uint32_t getAcceptedShares();
  uint32_t getDifficulty();
  float getShareRate();
  float getAcceptedRate();
  uint32_t getPing();
  
 protected:
  uint32_t last_fetch_time{0}; 

  TaskHandle_t miner1_handle{nullptr};
  TaskHandle_t miner2_handle{nullptr};

  MiningConfig *configuration{nullptr};
  MiningJob *job[SOC_CPU_CORES_NUM];

  bool fetch_pool_node();
  void generate_identifier();

  static void duco_thread_entry(void *params);

  CallbackManager<void()> share_found_callback;
};  // Duco

}  // namespace esphome::duco
