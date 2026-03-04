#pragma once

#define MINER_NAME "NerdMinerV2"

// ============================================================
// FreeRTOS Task Configuration (from BitsyMiner)
// ============================================================

// Core 0 - Shared tasks (WiFi, Stratum, Display, etc.)
#define CORE_0 0

// Core 1 - Dedicated mining (highest priority)
#define CORE_1 1

// Miner on Core 0 (lower priority, yields to other tasks)
#define MINER_0_CORE CORE_0
#define MINER_0_PRIORITY 1
#define MINER_0_STACK 8000  // Increased for SHA stack usage

// Core 0 mining yield configuration
// Higher = more hashes per yield, but UI/WiFi may lag
// Lower = more responsive UI, but fewer hashes from Core 0
// Values: 128 (responsive), 256 (default), 512 (throughput), 1024 (max)
#ifndef CORE_0_YIELD_COUNT
#define CORE_0_YIELD_COUNT 256
#endif

// Miner on Core 1 (highest priority, dedicated)
#define MINER_1_CORE CORE_1
#define MINER_1_PRIORITY 19  // Near-max priority (FreeRTOS max is 24)
#define MINER_1_STACK 8000   // Increased for SHA stack usage

// Stratum task
#define STRATUM_CORE CORE_0
#define STRATUM_PRIORITY 2
#define STRATUM_STACK 12288

// Monitor/Display task
// NOTE: Needs large stack for HTTPClient + JSON parsing + TFT rendering
#define MONITOR_CORE CORE_0
#define MONITOR_PRIORITY 1
#define MONITOR_STACK 10000

// Stats API task
// NOTE: Needs large stack for WiFiClientSecure SSL context (~10-15KB)
#define STATS_CORE CORE_0
#define STATS_PRIORITY 1
#define STATS_STACK 12000

// ============================================================
// Pool Configuration
// ============================================================

#define POOL_TIMEOUT_MS 60000    // 60s inactivity
#define POOL_KEEPALIVE_MS 30000  // 30s keepalive
#define POOL_FAILOVER_MS 30000   // 30s before failover

// ============================================================
// String Limits
// ============================================================

#define MAX_SSID_LENGTH 63  // Note: ESP-IDF uses MAX_SSID_LEN=32
#define MAX_PASSWORD_LEN 64
#define MAX_POOL_URL_LEN 80
#define MAX_WALLET_LEN 120
#define MAX_JOB_ID_LEN 64
