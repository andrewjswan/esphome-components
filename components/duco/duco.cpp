#include "esphome.h"
#include "duco.h"
#include "mining.h"

#include "esphome/core/log.h"

#include <esp_task_wdt.h>
#include <soc/soc_caps.h>

namespace esphome::duco {

// 15 minutes WDT for miner task
#define WDT_MINER_TIMEOUT 900000

MiningJob *job[SOC_CPU_CORES_NUM];

void Duco::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Duco...");

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  this->start();
}  // setup()

#ifdef USE_OTA_STATE_LISTENER
void Duco::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->stop();
  }
}
#endif

void task_func(void *task_id) {
  unsigned int miner_id = (uint32_t) task_id;
  ESP_LOGCONFIG(TAG, "[MINER] %d Started...", miner_id);

  while (1) {
     job[0]->mine();
  }
}

void Duco::start() {
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_MINER_TIMEOUT,
      .idle_core_mask = (1 << SOC_CPU_CORES_NUM) - 1,  // Bitmask of all cores
      .trigger_panic = true,
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_reconfigure(&wdt_config);
  
  configuration.wallet_id = String(random(0, 2811));   // Needed for miner grouping in the wallet
  
  job[0] = new MiningJob(0, configuration);
  xTaskCreatePinnedToCore(Duco::duco_function, "Miner-0", 10000, (void *) 0, 1, &miner1_handle, 0);
  esp_task_wdt_add(miner1_handle);

#if (SOC_CPU_CORES_NUM >= 2)
  job[1] = new MiningJob(1, configuration);
  xTaskCreatePinnedToCore(Duco::duco_function, "Miner-1", 10000, (void *) 1, 1, &miner2_handle, 1);
  esp_task_wdt_add(miner2_handle);
#endif

  ESP_LOGCONFIG(TAG, "Duco started...");
}  // start()

void Duco::stop() {
  vTaskDelete(miner1_handle);
  miner1_handle = nullptr;
  vTaskDelete(miner2_handle);
  miner2_handle = nullptr;

  ESP_LOGCONFIG(TAG, "Duco stopped.");
}  // stop()

void Duco::dump_config() {
  ESP_LOGCONFIG(TAG, "Duco version: %s", NERDMINER_VERSION);
  ESP_LOGCONFIG(TAG, "      Worker: %s", NERDMINER_WORKER);
  ESP_LOGCONFIG(TAG, "       Cores: %d", SOC_CPU_CORES_NUM);
}  // dump_config()

void Duco::duco_function(void *params) {
  uint8_t miner_id = (uint8_t) params;
  ESP_LOGCONFIG(TAG, "[MINER] %d Started...", miner_id);
  
  for (;;) {
    job[miner_id]->mine();
  }  // for(;;)

  vTaskDelete(NULL);
}  // duco_function()

}  // namespace esphome::duco
