#pragma once

#include <stdint.h>

// Mining
#define MAX_NONCE_STEP 5000000U
#define MAX_NONCE 25000000U
#define TARGET_NONCE 471136297U
#define DEFAULT_DIFFICULTY 0.00015
#define KEEPALIVE_TIME_ms 30000
#define POOLINACTIVITY_TIME_ms 60000

// #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#define HARDWARE_SHA265
// #endif

#define TARGET_BUFFER_SIZE 64

namespace esphome {
namespace nerdminer {

typedef struct {
  uint8_t bytearray_target[32];
  uint8_t bytearray_pooltarget[32];
  uint8_t merkle_result[32];
  uint8_t bytearray_blockheader[128];
} miner_data;

void runMonitor(void *name);
void runStratumWorker(void *name);
void minerWorkerSw(void *task_id);
void minerWorkerHw(void *task_id);

void resetStat();

}  // namespace nerdminer
}  // namespace esphome
