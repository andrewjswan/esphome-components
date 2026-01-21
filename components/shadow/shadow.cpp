#include "shadow.h"
#include "esphome/core/log.h"

namespace esphome {
namespace shadow {

void Shadow::setup() {
  ESP_LOGCONFIG(TAG, "Setting up shadow...");

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  this->start();
}  // setup()

#ifdef USE_OTA_STATE_LISTENER
void Shadow::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->stop();
  }
}
#endif

void Shadow::start() {
  xTaskCreatePinnedToCore(Shadow::shadow_function,  // Function to implement the task
                          TAG,                      // Name of the task
                          8192,                     // Stack size in words
                          (void *) this,            // Task input parameter
                          1,                        // Priority of the task
                          &shadow_handle,           // Task handle
                          tskNO_AFFINITY);          // Core

  ESP_LOGCONFIG(TAG, "Running script in shadow...");
}  // start()

void Shadow::shadow_function(void *params) {
  Shadow *this_task = (Shadow *) params;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(this_task->shadow_interval_ * 1000));

    if (this_task->script == nullptr) {
      ESP_LOGE(TAG, "The script came out of the shadow. Skip.");
      continue;
    }

    if (this_task->script->is_running()) {
      continue;
    }

    this_task->script->execute();
  }  // for(;;)

  vTaskDelete(NULL);
}  // shadow_function()

void Shadow::stop() {
  if (this->script != nullptr && this->script->is_running()) {
    this->script->stop();
  }

  vTaskDelete(shadow_handle);
  shadow_handle = nullptr;

  ESP_LOGCONFIG(TAG, "Everyone came out of the shadow.");
}  // stop()

void Shadow::set_script(script::Script<> *script) {
  this->script = script;
  ESP_LOGCONFIG(TAG, "Add script in the shadow...");
}  // set_script()

void Shadow::dump_config() {
  ESP_LOGCONFIG(TAG, "Shadow version: %s", SHADOW_VERSION);
  ESP_LOGCONFIG(TAG, "      Interval: %ds", this->shadow_interval_);
}  // dump_config()

}  // namespace shadow
}  // namespace esphome
