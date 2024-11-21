#include "esphome.h"
#include "mining.h"
#include "utils.h"
#include "monitor.h"

namespace esphome {
namespace nerdminer {

extern uint32_t templates;
extern uint32_t Mhashes;
extern uint32_t totalKHashes;
extern uint32_t elapsedKHs;
extern uint64_t upTime;

extern uint32_t shares;  // increase if blockhash has 32 bits of zeroes
extern uint32_t valids;  // increased if blockhash <= targethalfshares

extern double best_diff;  // track best diff

extern unsigned long mElapsed;

extern monitor_data mMonitor;

void setup_monitor(void) {
}

double getCurrentHashRate(unsigned long mElapsed) {
  return (1.0 * (elapsedKHs * 1000.0)) / mElapsed;
}

monitor_data getMonitorData() {
  return mMonitor;
}

mining_data getMiningData() {
  mining_data data;

  uint64_t secElapsed = upTime + (esp_timer_get_time() / 1000000);

  data.completedShares = shares;
  data.totalMHashes = Mhashes;
  data.totalKHashes = totalKHashes;
  data.currentHashRate = getCurrentHashRate(mElapsed);
  data.templates = templates;
  data.bestDiff = best_diff;
  data.timeMining = secElapsed;
  data.valids = valids;

  return data;
}

}  // namespace nerdminer
}  // namespace esphome
