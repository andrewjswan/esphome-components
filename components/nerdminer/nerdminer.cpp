#include "esphome.h"
#include "nerdminer.h"
#include "mining.h"
#include "monitor.h"
#include "timeconst.h"

#include "esphome/core/log.h"

#include <esp_task_wdt.h>
#include <soc/soc_caps.h>

// 15 minutes WDT for miner task
#define WDT_MINER_TIMEOUT 900000

namespace esphome {
namespace nerdminer {

void NerdMiner::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NerdMiner...");
  this->start();
}  // setup()

void NerdMiner::start() {
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_MINER_TIMEOUT,
      .idle_core_mask = (1 << SOC_CPU_CORES_NUM) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_reconfigure(&wdt_config);

  BaseType_t res1 = xTaskCreatePinnedToCore(runMonitor, "Monitor", 10000, (void *) "Monitor", 5, &monitor_handle, 1);
  BaseType_t res2 =
      xTaskCreatePinnedToCore(runStratumWorker, "Stratum", 15000, (void *) "Stratum", 4, &stratum_handle, 1);

#ifdef HARDWARE_SHA265
  xTaskCreate(minerWorkerHw, "MinerHW-0", 4096, (void *) 0, 3, &miner1_handle);
#else
  xTaskCreate(minerWorkerSw, "MinerSW-0", 6000, (void *) 0, 1, &miner1_handle);
#endif
  esp_task_wdt_add(miner1_handle);

#if (SOC_CPU_CORES_NUM >= 2)
  xTaskCreate(minerWorkerSw, "MinerSW-1", 6000, (void *) 1, 1, &miner2_handle);
  esp_task_wdt_add(miner2_handle);
#endif

  vTaskPrioritySet(NULL, 4);

  ESP_LOGCONFIG(TAG, "NerdMiner started...");
}  // start()

void NerdMiner::stop() {
  vTaskDelete(miner1_handle);
  miner1_handle = nullptr;
  vTaskDelete(miner2_handle);
  miner2_handle = nullptr;

  vTaskDelete(stratum_handle);
  stratum_handle = nullptr;

  vTaskDelete(monitor_handle);
  monitor_handle = nullptr;

  ESP_LOGCONFIG(TAG, "NerdMiner stopped.");
}  // stop()

void NerdMiner::dump_config() {
  ESP_LOGCONFIG(TAG, "NerdMiner version: %s", NERDMINER_VERSION);
  ESP_LOGCONFIG(TAG, "           Worker: %s", NERDMINER_WORKER);
  ESP_LOGCONFIG(TAG, "             Pool: %s", NERDMINER_POOL);
  ESP_LOGCONFIG(TAG, "             Port: %d", NERDMINER_POOL_PORT);
  ESP_LOGCONFIG(TAG, "            Cores: %d", SOC_CPU_CORES_NUM);
#ifdef HARDWARE_SHA265
  ESP_LOGCONFIG(TAG, "  Hardware SHA265: Yes");
#endif
}  // dump_config()

bool NerdMiner::getMinerState() {
  monitor_data mData = getMonitorData();
  return mData.Status;
}

uint32_t NerdMiner::getTotalHashes() {
  mining_data mData = getMiningData();
  return mData.totalMHashes;
}  // getTotalHashes()

uint32_t NerdMiner::getBlockTemplates() {
  mining_data mData = getMiningData();
  return mData.templates;
}  // getBlockTemplates()

double NerdMiner::getBestDiff() {
  mining_data mData = getMiningData();
  return mData.bestDiff;
}  // getBestDiff()

uint32_t NerdMiner::get32BitShares() {
  mining_data mData = getMiningData();
  return mData.completedShares;
}  // get32BitShares()

uint64_t NerdMiner::getHores() {
  mining_data mData = getMiningData();
  return mData.timeMining;
}  // getHores()

uint32_t NerdMiner::getValidBlocks() {
  mining_data mData = getMiningData();
  return mData.valids;
}  // getValidBlocks()

double NerdMiner::getHashrate() {
  mining_data mData = getMiningData();
  return mData.currentHashRate;
}  // getHashrate()

uint32_t NerdMiner::getKHashes() {
  mining_data mData = getMiningData();
  return mData.totalKHashes;
}  // getKHashes()

}  // namespace nerdminer
}  // namespace esphome
