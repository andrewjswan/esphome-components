#pragma once

namespace esphome {
namespace shadow {

static const char *const SHADOW_VERSION = "2024.11.5";
static const char *const TAG = "shadow";

class Shadow : public Component {
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
};  // Shadow

}  // namespace shadow
}  // namespace esphome
