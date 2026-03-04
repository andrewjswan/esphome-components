#include "config.h"
#include "nerdminer.h"
#include "miner.h"
#include "stratum_types.h"
#include "stratum.h"
#include "monitor.h"
#include "utils.h"

#include "esphome/core/log.h"

#include <esp_task_wdt.h>
#include <esp_pm.h>
#include <soc/soc_caps.h>

// 15 minutes WDT for miner task
#define WDT_MINER_TIMEOUT 900000

namespace esphome {
namespace nerdminer {

void NerdMiner::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NerdMiner...");

  // Disable power management (no CPU throttling/sleep)
  setup_powermanagement();

  // Initialize mining subsystem
  miner_init();

  // Initialize stratum subsystem
  stratum_init();
  stratum_set_pool(this->pool_, this->pool_port_, this->wallet_id_, this->pool_pass_, this->worker_name_);

  // Initialize monitor (live stats - display already initialized)
  monitor_init();

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  ESP_LOGCONFIG(TAG, "Setup complete.");

  this->start();
}  // setup()

#ifdef USE_OTA_STATE_LISTENER
void NerdMiner::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->stop();
  }
}
#endif

void NerdMiner::start() {
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_MINER_TIMEOUT,
      .idle_core_mask = (1 << SOC_CPU_CORES_NUM) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_reconfigure(&wdt_config);

  xTaskCreatePinnedToCore(monitor_task, "Monitor", MONITOR_STACK, (void *) "Monitor", MONITOR_PRIORITY, &monitor_handle,
                          MONITOR_CORE);
  xTaskCreatePinnedToCore(stratum_task, "Stratum", STRATUM_STACK, (void *) "Stratum", STRATUM_PRIORITY, &stratum_handle,
                          STRATUM_CORE);

#if (SOC_CPU_CORES_NUM >= 2)
  // Dual-core: Run miners on both cores
  xTaskCreatePinnedToCore(miner_task_core0, "Miner-0", MINER_0_STACK, (void *) 1, MINER_0_PRIORITY, &miner0_handle,
                          MINER_0_CORE);
  // Miner on Core 1 (high priority, dedicated core)
  xTaskCreatePinnedToCore(miner_task_core1, "Miner-1", MINER_1_STACK, (void *) 1, MINER_1_PRIORITY, &miner1_handle,
                          MINER_1_CORE);
#else
  // Single-core (C3, S2): Run only one miner task, not pinned
  // Must yield frequently to let WiFi/Stratum work
  xTaskCreate(miner_task_core0, "Miner-0", MINER_0_STACK, (void *) 0, MINER_0_PRIORITY, &miner0_handle);
#endif

  ESP_LOGCONFIG(TAG, "NerdMiner started...");
}  // start()

void NerdMiner::stop() {
  vTaskDelete(miner0_handle);
  miner0_handle = nullptr;
  vTaskDelete(miner1_handle);
  miner1_handle = nullptr;

  vTaskDelete(stratum_handle);
  stratum_handle = nullptr;

  vTaskDelete(monitor_handle);
  monitor_handle = nullptr;

  ESP_LOGCONFIG(TAG, "NerdMiner stopped.");
}  // stop()

void NerdMiner::dump_config() {
  ESP_LOGCONFIG(TAG, "NerdMiner version: %s", NERDMINER_VERSION);
  ESP_LOGCONFIG(TAG, "           Worker: %s", this->worker_name_);
  ESP_LOGCONFIG(TAG, "             Pool: %s", this->pool_);
  ESP_LOGCONFIG(TAG, "             Port: %d", this->pool_port_);
  ESP_LOGCONFIG(TAG, "            Cores: %d", SOC_CPU_CORES_NUM);
#ifdef HARDWARE_SHA265
  ESP_LOGCONFIG(TAG, "  Hardware SHA265: Yes");
#endif
}  // dump_config()

bool NerdMiner::getMinerState() {
  // monitor_data mData = getMonitorData();
  // return mData.Status;
}

uint32_t NerdMiner::getTotalHashes() {
  // mining_data mData = getMiningData();
  // return mData.totalMHashes;
}  // getTotalHashes()

uint32_t NerdMiner::getBlockTemplates() {
  // mining_data mData = getMiningData();
  // return mData.templates;
}  // getBlockTemplates()

double NerdMiner::getBestDiff() {
  // mining_data mData = getMiningData();
  // return mData.bestDiff;
}  // getBestDiff()

uint32_t NerdMiner::get32BitShares() {
  // mining_data mData = getMiningData();
  // return mData.completedShares;
}  // get32BitShares()

uint64_t NerdMiner::getHores() {
  // mining_data mData = getMiningData();
  // return mData.timeMining;
}  // getHores()

uint32_t NerdMiner::getValidBlocks() {
  // mining_data mData = getMiningData();
  // return mData.valids;
}  // getValidBlocks()

double NerdMiner::getHashrate() {
  // mining_data mData = getMiningData();
  // return mData.currentHashRate;
}  // getHashrate()

uint32_t NerdMiner::getKHashes() {
  // mining_data mData = getMiningData();
  // return mData.totalKHashes;
}  // getKHashes()

}  // namespace nerdminer
}  // namespace esphome
