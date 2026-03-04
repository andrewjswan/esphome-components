/*
 * SparkMiner - Monitor Task Implementation
 * Coordinates display updates and live stats fetching
 */

#include "config.h"
#include "nerdminer.h"
#include "monitor.h"
#include "miner.h"
#include "stratum.h"

#include "esphome/components/network/util.h"
#include "esphome/core/hal.h"

// Update intervals
#define DISPLAY_UPDATE_MS 1000    // 1 second
#define STATS_UPDATE_MS 10000     // 10 seconds
#define PERSIST_STATS_MS 3600000  // 1 hour - save to flash for persistence
#define EARLY_SAVE_MS 300000      // 5 minutes - initial save interval before first hourly
#define LED_UPDATE_MS 50          // 50ms for smooth LED animations

namespace esphome {
namespace nerdminer {

static bool s_initialized = false;
static uint32_t s_lastDisplayUpdate = 0;
static uint32_t s_lastStatsUpdate = 0;
static uint32_t s_lastPersistSave = 0;
static uint32_t s_lastLedUpdate = 0;
static uint32_t s_startTime = 0;
static uint32_t s_lastActivityTime = 0;   // Screen timeout tracking
static bool s_earlySaveDone = false;      // Track if we've done the early save
static uint32_t s_lastAcceptedCount = 0;  // Track shares for first-share save
static uint32_t s_lastLedShareCount = 0;  // Track shares for LED flash

// Track session start values to calculate deltas for persistence
static uint64_t s_sessionStartHashes = 0;
static uint32_t s_sessionStartShares = 0;
static uint32_t s_sessionStartAccepted = 0;
static uint32_t s_sessionStartRejected = 0;
static uint32_t s_sessionStartBlocks = 0;

// ============================================================
// Helper Functions
// ============================================================

static void updateDisplayData(display_data_t *data) {
  // Get mining stats (current session)
  mining_stats_t *mstats = miner_get_stats();

  // Display lifetime totals (persistent + current session)
  data->totalHashes = mstats->hashes;
  data->sharesAccepted = mstats->accepted;
  data->sharesRejected = mstats->rejected;
  data->blocksFound = mstats->blocks;

  // Best difficulty: max of lifetime best and current session best
  data->bestDifficulty = mstats->bestDifficulty;

  // Session-only values (these make sense per-session)
  data->templates = mstats->templates;
  data->blocks32 = mstats->matches32;
  data->uptimeSeconds = (millis() - s_startTime) / 1000;
  data->avgLatency = mstats->avgLatency;

  // Calculate hashrate with EMA smoothing
  static uint64_t lastHashes = 0;
  static uint32_t lastHashTime = 0;
  static double smoothedHashRate = 0.0;
  static bool firstSample = true;

  uint32_t now = millis();
  uint32_t elapsed = now - lastHashTime;

  if (elapsed >= 1000) {
    uint64_t deltaHashes = mstats->hashes - lastHashes;
    double instantRate = (double) deltaHashes * 1000.0 / elapsed;

    // Exponential moving average (alpha=0.15 for smooth but responsive updates)
    // Lower alpha = smoother but slower to respond
    // Higher alpha = more responsive but jumpier
    const double alpha = 0.15;

    if (firstSample) {
      smoothedHashRate = instantRate;
      firstSample = false;
    } else {
      smoothedHashRate = alpha * instantRate + (1.0 - alpha) * smoothedHashRate;
    }

    data->hashRate = smoothedHashRate;
    lastHashes = mstats->hashes;
    lastHashTime = now;
  }

  // Pool info
  data->poolConnected = stratum_is_connected();
  data->poolName = stratum_get_pool();

  // Get pool difficulty from miner
  data->poolDifficulty = miner_get_difficulty();

  // Network info
  data->wifiConnected = network::is_connected();
  data->ipAddress = network::get_use_address();
}

// ============================================================
// Public API
// ============================================================

void monitor_init() {
  if (s_initialized)
    return;

// Initialize LED status driver (for headless builds with RGB LED)
#ifdef USE_LED_STATUS
  led_status_init();
  led_status_set(LED_STATUS_CONNECTING);
#endif

  s_startTime = millis();
  s_lastPersistSave = millis();
  s_lastActivityTime = millis();
  s_initialized = true;

  ESP_LOGD(TAG, "[MONITOR] Initialized.");
}

void monitor_task(void *param) {
  ESP_LOGD(TAG, "[MONITOR] Task started on core %d", xPortGetCoreID());

  if (!s_initialized) {
    monitor_init();
  }

  display_data_t displayData;
  memset(&displayData, 0, sizeof(displayData));

  while (true) {
    uint32_t now = millis();

    // Update display
    if (now - s_lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
      updateDisplayData(&displayData);

      // Also print to serial for headless/debug
      static uint32_t lastSerialPrint = 0;
      if (now - lastSerialPrint >= 10000) {
        ESP_LOGD(TAG, "[STATS] Hashrate: %.2f H/s | Shares: %u/%u | Ping: %u ms | Best: %.4f", displayData.hashRate,
                 displayData.sharesAccepted, displayData.sharesAccepted + displayData.sharesRejected,
                 displayData.avgLatency, displayData.bestDifficulty);

        if (displayData.poolName) {
          ESP_LOGD(TAG, "[STATS] Pool: %s (%d workers) %s", displayData.poolName, displayData.poolWorkersTotal,
                   (displayData.poolFailovers > 0) ? "[FAILOVER]" : "");
        }

        if (displayData.btcPrice > 0) {
          ESP_LOGD(TAG, "[STATS] BTC: $%.0f | Block: %u | Fee: %d sat/vB", displayData.btcPrice,
                   displayData.blockHeight, displayData.halfHourFee);
        }

        // DEBUG: Per-core hash contribution
        extern volatile uint64_t s_core0Hashes;
        extern volatile uint64_t s_core1Hashes;
        ESP_LOGD(TAG, "[STATS] Core0: %llu hashes, Core1: %llu hashes", s_core0Hashes, s_core1Hashes);

        lastSerialPrint = now;
      }

      s_lastDisplayUpdate = now;
    }

// Update LED status for headless builds
#ifdef USE_LED_STATUS
    if (now - s_lastLedUpdate >= LED_UPDATE_MS) {
      // Determine current status based on connection state
      if (!displayData.wifiConnected) {
        // Check if in AP mode
        if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
          led_status_set(LED_STATUS_AP_MODE);
        } else {
          led_status_set(LED_STATUS_CONNECTING);
        }
      } else if (!displayData.poolConnected) {
        led_status_set(LED_STATUS_CONNECTING);
      } else if (displayData.hashRate > 0) {
        led_status_set(LED_STATUS_MINING);
      }

      // Check for new share accepted - flash LED
      mining_stats_t *ledStats = miner_get_stats();
      if (ledStats->accepted > s_lastLedShareCount) {
        led_status_share_found();
        s_lastLedShareCount = ledStats->accepted;
      }

      // Check for block found - celebration!
      if (ledStats->blocks > 0) {
        static uint32_t lastBlockCount = 0;
        if (ledStats->blocks > lastBlockCount) {
          led_status_block_found();
          lastBlockCount = ledStats->blocks;
        }
      }

      // Update LED animation
      led_status_update();
      s_lastLedUpdate = now;
    }
#endif

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

}  // namespace nerdminer
}  // namespace esphome
