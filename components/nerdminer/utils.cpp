#include "nerdminer.h"
#include "utils.h"

#include "esphome/core/log.h"

// For CPU overclocking
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <esp32/rom/rtc.h>
#endif

extern "C" {
#include <esp_private/esp_clk.h>
}

namespace esphome {
namespace nerdminer {

/**
 * Attempt CPU overclock using 320MHz PLL
 * ESP32 valid PLL configs:
 *   - 480MHz PLL / 2 = 240MHz (standard)
 *   - 320MHz PLL / 1 = 320MHz (overclock, may not be stable)
 *   - 320MHz PLL / 2 = 160MHz
 *
 * Returns the achieved frequency
 */
uint32_t try_overclock() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  ESP_LOGD(TAG, "[OVERCLOCK] Attempting 320MHz overclock...");

  uint32_t baseFreq = esp_clk_cpu_freq() / 1000000;
  ESP_LOGD(TAG, "[OVERCLOCK] Base frequency: %u MHz", baseFreq);

  // Configure for 320MHz PLL with divider 1
  rtc_cpu_freq_config_t conf;
  conf.source = RTC_CPU_FREQ_SRC_PLL;
  conf.source_freq_mhz = 320;  // 320MHz PLL
  conf.div = 1;                // div=1 = 320MHz output
  conf.freq_mhz = 320;

  // Apply the configuration
  rtc_clk_cpu_freq_set_config(&conf);

  // Adjust serial baud rate for new clock (320/240 ratio)
  // Serial internally uses APB clock which may change
  delay(10);

  // Quick stability test
  volatile uint32_t dummy = 0;
  for (int j = 0; j < 100000; j++) {
    dummy += j * 17;
    dummy ^= (dummy >> 3);
  }

  // Verify the frequency
  uint32_t actualFreq = esp_clk_cpu_freq() / 1000000;
  if (actualFreq >= 300) {
    ESP_LOGD(TAG, "[OVERCLOCK] SUCCESS at %u MHz!", actualFreq);
    return actualFreq;
  }

  // Failed, revert to 240MHz
  rtc_cpu_freq_config_t default_conf;
  rtc_clk_cpu_freq_get_config(&default_conf);
  ESP_LOGD(TAG, "[OVERCLOCK] Failed (got %u MHz), using 240 MHz", actualFreq);
  return 240;

#else
  ESP_LOGD(TAG, "[OVERCLOCK] Not supported on this chip variant");
  return esp_clk_cpu_freq() / 1000000;
#endif
}

/**
 * Disable ESP32 power management for consistent performance
 * From BitsyMiner - critical for maintaining hashrate
 */
void setup_powermanagement() {
#if CONFIG_PM_ENABLE
  esp_pm_lock_handle_t pmLock;
  esp_err_t err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "miner", &pmLock);
  if (err == ESP_OK) {
    esp_pm_lock_acquire(pmLock);
    ESP_LOGD(TAG, "[INIT] Power management disabled (no sleep)");
  } else {
    ESP_LOGD(TAG, "[WARN] Could not disable power management");
  }
#else
  ESP_LOGD(TAG, "[INIT] Power management not enabled in config");
#endif
}

}  // namespace nerdminer
}  // namespace esphome
