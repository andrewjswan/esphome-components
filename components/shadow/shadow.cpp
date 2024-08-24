#include "esphome.h"

namespace esphome
{
  void SHADOW::setup()
  {
    ESP_LOGCONFIG(TAG, "Setting up shadow...");
    this->start();
  }

  void SHADOW::start()
  {
    xTaskCreatePinnedToCore(
      SHADOW::shadow_function,      // Function to implement the task
      TAG,                          // Name of the task
      8192,                         // Stack size in words
      (void *) this,                // Task input parameter
      1,                            // Priority of the task
      &shadow_handle,               // Task handle
      tskNO_AFFINITY);              // Core

    ESP_LOGCONFIG(TAG, "Running script in shadow...");
  }

  void SHADOW::shadow_function(void *params)
  {
    SHADOW *this_task = (SHADOW *) params;
    for(;;)
    {
      vTaskDelay(SHADOW_INTERVAL * 1000 / portTICK_RATE_MS);

      if (this_task->script == nullptr)
      {
        ESP_LOGE(TAG, "The script came out of the shadow. Skip.");
        continue;
      }

      if (this_task->script->is_running())
      {
        continue;
      }

      this_task->script->execute();
    } // for(;;)

    vTaskDelete( NULL );
  } // shadow_function()

  void SHADOW::stop()
  {
    if (this->script != nullptr && this->script->is_running())
    {
      this->script->stop();
    }

    vTaskDelete(shadow_handle);
    shadow_handle = nullptr;

    ESP_LOGCONFIG(TAG, "Everyone came out of the shadow.");
  }

  void SHADOW::set_script(script::Script<> *script)
  {
    this->script = script;
    ESP_LOGCONFIG(TAG, "Add script in the shadow...");
  }

  void SHADOW::dump_config()
  {
    ESP_LOGCONFIG(TAG, "Shadow version: %s", SHADOW_VERSION);
    ESP_LOGCONFIG(TAG, "  Interval: %ds", SHADOW_INTERVAL);
  }
}
