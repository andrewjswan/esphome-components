#include "esphome.h"
#include "mining.h"
#include "monitor.h"
#include "timeconst.h"
#include <esp_task_wdt.h>

//15 minutes WDT for miner task
#define WDT_MINER_TIMEOUT 900

namespace esphome {
namespace nerdminer {

void NerdMiner::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NerdMiner...");
  this->start();
}  // setup()


void NerdMiner::start() {
  esp_task_wdt_init(WDT_MINER_TIMEOUT, true);

  BaseType_t res1 = xTaskCreatePinnedToCore(runMonitor, "Monitor", 10000, (void*)"Monitor", 4, &monitor_handle, 1);
  BaseType_t res2 = xTaskCreatePinnedToCore(runStratumWorker, "Stratum", 15000, (void*)"Stratum", 3, &stratum_handle, 1);
  
  xTaskCreate(runMiner, "Miner0", 6000, (void*)0, 1, &miner1_handle);
  xTaskCreate(runMiner, "Miner1", 6000, (void*)1, 1, &miner2_handle);
 
  esp_task_wdt_add(miner1_handle);
  esp_task_wdt_add(miner2_handle);
  
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
}  // dump_config()


bool NerdMiner::getMinerState() {
  monitor_data mData = getMonitorData();
  return mData.Status;
}


uint32_t NerdMiner::getTotalHashes() {
  mining_data mData = getMiningData();
  return mData.totalMHashes;
} // getTotalHashes()


uint32_t NerdMiner::getBlockTemplates() {
  mining_data mData = getMiningData();
  return mData.templates;
} // getBlockTemplates()


double NerdMiner::getBestDiff() {
  mining_data mData = getMiningData();
  return mData.bestDiff;
} // getBestDiff()


uint32_t NerdMiner::get32BitShares() {
  mining_data mData = getMiningData();
  return mData.completedShares;
} // get32BitShares()


uint64_t NerdMiner::getHores() {
  mining_data mData = getMiningData();
  return mData.timeMining;
} // getHores()


uint32_t NerdMiner::getValidBlocks() {
  mining_data mData = getMiningData();
  return mData.valids;
} // getValidBlocks()


double NerdMiner::getHashrate() {
  mining_data mData = getMiningData();
  return mData.currentHashRate;
} // getHashrate()


uint32_t NerdMiner::getKHashes() {
  mining_data mData = getMiningData();
  return mData.totalKHashes;
} // getKHashes()

}  // namespace nerdminer
}  // namespace esphome
