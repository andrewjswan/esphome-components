#pragma once

/*
 * SparkMiner - Monitor Task
 * Coordinates display updates and live stats fetching
 */

namespace esphome {
namespace nerdminer {

// Display data structure
// NOTE: Using fixed char arrays instead of Arduino String to prevent heap fragmentation
// The struct tag display_data_s is used by display_interface.h forward declaration
struct display_data_t {
  // Mining stats
  uint64_t totalHashes;
  double hashRate;
  double bestDifficulty;
  uint32_t sharesAccepted;
  uint32_t sharesRejected;
  uint32_t templates;
  uint32_t blocks32;
  uint32_t blocksFound;
  uint32_t uptimeSeconds;
  uint32_t avgLatency;  // Average pool latency in ms
  uint32_t cpuMhz;      // CPU frequency in MHz

  // Pool info
  bool poolConnected;
  const char *poolName;
  double poolDifficulty;
  int poolFailovers;  // Number of failovers (for warning color)

  // Pool stats (from API) - fixed char arrays
  int poolWorkersTotal;      // Total workers on pool
  int poolWorkersAddress;    // Workers on your address
  char poolHashrate[24];     // Pool total hashrate
  char workerHashrate[24];   // Your combined worker hashrate
  char addressBestDiff[24];  // Your best difficulty on pool

  // Network info
  bool wifiConnected;
  int8_t wifiRssi;  // WiFi signal strength in dBm
  const char *ipAddress;

  // Live stats (from API) - fixed char arrays
  float btcPrice;
  uint32_t blockHeight;
  char networkHashrate[24];
  char networkDifficulty[24];
  int halfHourFee;

  // Difficulty Adjustment
  float difficultyProgress;
  float difficultyChange;
  uint32_t difficultyRetargetBlocks;
};

/**
 * Initialize monitor subsystem
 */
void monitor_init();

/**
 * Monitor task (runs on Core 0)
 * Updates display and fetches live stats periodically
 */
void monitor_task(void *param);

}  // namespace nerdminer
}  // namespace esphome
