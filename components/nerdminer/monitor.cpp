#include "mining.h"
#include "utils.h"
#include "monitor.h"

#include <list>

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

void setup_monitor(void) {}

static std::list<double> s_hashrate_avg_list;
static double s_hashrate_summ = 0.0;
static uint8_t s_hashrate_recalc = 0;

double getCurrentHashRate() {
  double hashrate = (double) elapsedKHs * 1000.0 / (double) mElapsed;

  s_hashrate_summ += hashrate;
  s_hashrate_avg_list.push_back(hashrate);
  if (s_hashrate_avg_list.size() > 10) {
    s_hashrate_summ -= s_hashrate_avg_list.front();
    s_hashrate_avg_list.pop_front();
  }

  ++s_hashrate_recalc;
  if (s_hashrate_recalc == 0) {
    s_hashrate_summ = 0.0;
    for (auto itt = s_hashrate_avg_list.begin(); itt != s_hashrate_avg_list.end(); ++itt) {
      s_hashrate_summ += *itt;
    }
  }

  double avg_hashrate = s_hashrate_summ / (double) s_hashrate_avg_list.size();
  if (avg_hashrate < 0.0) {
    avg_hashrate = 0.0;
  }
  return avg_hashrate;
}

monitor_data getMonitorData() { return mMonitor; }

mining_data getMiningData() {
  mining_data data;

  data.completedShares = shares;
  data.totalMHashes = Mhashes;
  data.totalKHashes = totalKHashes;
  data.currentHashRate = getCurrentHashRate();
  data.templates = templates;
  data.bestDiff = best_diff;
  data.timeMining = upTime;
  data.valids = valids;

  return data;
}

}  // namespace nerdminer
}  // namespace esphome
