#pragma once

static const char *const SHADOW_VERSION = "2024.9.5";
static const char *const TAG = "shadow";

namespace esphome {
class SHADOW : public Component {
 public:
  void setup() override;
  void start();
  void stop();

  void dump_config() override;
  void set_script(script::Script<> *script);

 protected:
  TaskHandle_t shadow_handle = nullptr;
  script::Script<> *script;

  static void shadow_function(void *params);
};
} // namespace esphome
