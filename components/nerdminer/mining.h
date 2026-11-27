#pragma once

#include <stdint.h>

namespace esphome::nerdminer {

#define MAX_NONCE_STEP 4096U
#define MAX_NONCE 0xFFFFFFFFU
#define TARGET_BUFFER_SIZE 64

typedef struct {
  uint8_t bytearray_target[32];
  uint8_t bytearray_blockheader[128];
  uint8_t merkle_result[32];
  uint8_t bytearray_blockheader2[128];
} miner_data;

void init_miner_sha();
void stopMining();

void runMonitor(void *name);
void runStratumWorker(void *name);
void minerWorkerSw(void *task_id);
void minerWorkerHw(void *task_id);

}  // namespace esphome::nerdminer
