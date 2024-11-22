#ifndef MONITOR_API_H
#define MONITOR_API_H

#define NEXT_HALVING_EVENT 1050000  // 840000
#define HALVING_BLOCKS 210000

namespace esphome {
namespace nerdminer {

enum NMState { NM_Connecting, NM_Hashing };

typedef struct {
  bool Status;
  NMState NerdStatus;
} monitor_data;

typedef struct {
  uint32_t totalMHashes;     // Total Hashes
  uint32_t templates;        // Block Templates
  double bestDiff;           // Best Difficulty
  uint32_t completedShares;  // 32Bit Shares
  uint64_t timeMining;       // Hores
  uint32_t valids;           // Valid Blocks
  double currentHashRate;    // Hashrate
  uint32_t totalKHashes;     // KHashes
} mining_data;

void setup_monitor(void);

monitor_data getMonitorData();
mining_data getMiningData();

}  // namespace nerdminer
}  // namespace esphome

#endif  // MONITOR_API_H
