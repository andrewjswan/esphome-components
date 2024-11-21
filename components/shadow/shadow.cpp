#include "esphome.h"

namespace esphome {
namespace shadow {

void Shadow::setup() {
  ESP_LOGCONFIG(TAG, "Setting up shadow...");
  this->start();
}  // setup()

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
    vTaskDelay(pdMS_TO_TICKS(SHADOW_INTERVAL * 1000));

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
  ESP_LOGCONFIG(TAG, "      Interval: %ds", SHADOW_INTERVAL);
}  // dump_config()

}  // namespace shadow
}  // namespace esphome
