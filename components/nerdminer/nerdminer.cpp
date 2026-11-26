#include "nerdminer.h"
#include "mining.h"
#include "monitor.h"

#include "esphome/core/log.h"

#include <soc/soc_caps.h>

namespace esphome::nerdminer {

void NerdMiner::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NerdMiner...");

  init_miner_sha();

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  ESP_LOGCONFIG(TAG, "Setup complete.");

  this->start();
}

#ifdef USE_OTA_STATE_LISTENER
void NerdMiner::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->stop();
  }
}
#endif

void NerdMiner::start() {
  // Keep network and monitoring tasks on Core 0 alongside Wi-Fi
  xTaskCreatePinnedToCore(runMonitor, "Monitor", 4096, const_cast<char *>("Monitor"), 3, &this->monitor_handle_, 0);
  xTaskCreatePinnedToCore(runStratumWorker, "Stratum", 12288, const_cast<char *>("Stratum"), 4, &this->stratum_handle_,
                          0);

#if (SOC_CPU_CORES_NUM >= 2)
  // Dual-core: dedicate Core 1 to hardware SHA miner, run software miner on Core 0.
  // Priority 1 allows periodic vTaskDelay(1) yields to feed standard ESPHome IDLE WDT.
  xTaskCreatePinnedToCore(minerWorkerHw, "MinerHW-1", 6144, const_cast<char *>("MinerHW-1"), 1, &this->miner_hw_handle_,
                          1);
  xTaskCreatePinnedToCore(minerWorkerSw, "MinerSW-0", 6144, const_cast<char *>("MinerSW-0"), 1, &this->miner_sw_handle_,
                          0);
#else
  // Single-core (ESP32-C3 / ESP32-S2): run single hardware miner at low priority
  xTaskCreatePinnedToCore(minerWorkerHw, "MinerHW-0", 6144, const_cast<char *>("MinerHW-0"), 1, &this->miner_hw_handle_,
                          0);
#endif

  ESP_LOGCONFIG(TAG, "NerdMiner started...");
}

void NerdMiner::stop() {
  stopMining();

  // Wait up to 500 ms for tasks to exit cleanly on their own (closing sockets & releasing HW)
  for (int i = 0; i < 25; i++) {
    bool any_running = false;
    if (this->miner_hw_handle_ != nullptr && eTaskGetState(this->miner_hw_handle_) != eDeleted) {
      any_running = true;
    }
    if (this->miner_sw_handle_ != nullptr && eTaskGetState(this->miner_sw_handle_) != eDeleted) {
      any_running = true;
    }
    if (this->stratum_handle_ != nullptr && eTaskGetState(this->stratum_handle_) != eDeleted) {
      any_running = true;
    }
    if (this->monitor_handle_ != nullptr && eTaskGetState(this->monitor_handle_) != eDeleted) {
      any_running = true;
    }
    if (!any_running) {
      break;
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  this->miner_hw_handle_ = nullptr;
  this->miner_sw_handle_ = nullptr;
  this->stratum_handle_ = nullptr;
  this->monitor_handle_ = nullptr;

  ESP_LOGCONFIG(TAG, "NerdMiner stopped.");
}

void NerdMiner::dump_config() {
  ESP_LOGCONFIG(TAG, "NerdMiner client: %s", NERDMINER_CLIENT);
  ESP_LOGCONFIG(TAG, "         version: %s", NERDMINER_VERSION);
  ESP_LOGCONFIG(TAG, "          Worker: %s", NERDMINER_WORKER);
  ESP_LOGCONFIG(TAG, "            Pool: %s", NERDMINER_POOL);
  ESP_LOGCONFIG(TAG, "            Port: %d", NERDMINER_POOL_PORT);
  ESP_LOGCONFIG(TAG, "           Cores: %d", SOC_CPU_CORES_NUM);
}

bool NerdMiner::getMinerState() {
  monitor_data m_data = getMonitorData();
  return m_data.Status;
}

uint32_t NerdMiner::getTotalHashes() {
  mining_data m_data = getMiningData();
  return m_data.totalMHashes;
}

uint32_t NerdMiner::getBlockTemplates() {
  mining_data m_data = getMiningData();
  return m_data.templates;
}

double NerdMiner::getBestDiff() {
  mining_data m_data = getMiningData();
  return m_data.bestDiff;
}

uint32_t NerdMiner::get32BitShares() {
  mining_data m_data = getMiningData();
  return m_data.completedShares;
}

uint64_t NerdMiner::getHores() {
  mining_data m_data = getMiningData();
  return m_data.timeMining;
}

uint32_t NerdMiner::getValidBlocks() {
  mining_data m_data = getMiningData();
  return m_data.valids;
}

double NerdMiner::getHashrate() {
  mining_data m_data = getMiningData();
  return m_data.currentHashRate;
}

uint32_t NerdMiner::getKHashes() {
  mining_data m_data = getMiningData();
  return m_data.totalKHashes;
}

}  // namespace esphome::nerdminer
