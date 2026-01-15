#include "nerdSHA256plus.h"
#include "stratum.h"
#include "mining.h"
#include "utils.h"
#include "monitor.h"
#include "timeconst.h"
#include "nerdminer.h"

#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <mutex>
#include <list>
#include <map>

#ifdef HARDWARE_SHA265
#include <sha/sha_dma.h>
#include <hal/sha_hal.h>
#include <hal/sha_ll.h>

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <sha/sha_parallel_engine.h>
#endif
#endif

namespace esphome {
namespace nerdminer {

// Jobs per second
#define NONCE_PER_JOB_SW 4096
#define NONCE_PER_JOB_HW 16 * 1024
#define JOB_QUEUE_SIZE 4      // Normal queue size
#define RESULT_QUEUE_SIZE 16  // Normal result queue

#define RANDOM_NONCE_MASK 0xFFFFC000

uint32_t templates = 0;
uint32_t hashes = 0;
uint32_t Mhashes = 0;
uint32_t totalKHashes = 0;
uint32_t elapsedKHs = 0;
uint64_t upTime = 0;

volatile uint32_t shares;  // increase if blockhash has 32 bits of zeroes
volatile uint32_t valids;  // increased if blockhash <= target

// Track best diff
double best_diff = 0.0;

IPAddress serverIP(1, 1, 1, 1);  // Temporally save poolIPaddres

// Global work data
static std::unique_ptr<esphome::socket::Socket> pool_socket;
static miner_data mMiner;  // Global miner data (Create a miner class TODO)
mining_subscribe mWorker;
mining_job mJob;
monitor_data mMonitor;
static bool volatile isMinerSuscribed = false;
unsigned long mLastTXtoPool = millis();

bool checkPoolConnection(void) {
  if (pool_socket != nullptr) {
    return true;
  }

  isMinerSuscribed = false;
  ESP_LOGD(TAG, "Client not connected, trying to connect...");

  pool_socket = esphome::socket::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  auto addr = esphome::network::get_ip_address(NERDMINER_POOL);
  if (!addr.has_value()) return false;

  // Try connecting pool IP
  struct sockaddr_in s_addr;
  s_addr.sin_family = AF_INET;
  s_addr.sin_port = htons(NERDMINER_POOL_PORT);
  s_addr.sin_addr.s_addr = uint32_t(addr.value());

  if (pool_socket->connect((struct sockaddr *)&s_addr, sizeof(s_addr)) != 0) {
    if (errno != EINPROGRESS) {
      ESP_LOGW(TAG, "Imposible to connect to: %s", NERDMINER_POOL);
      pool_socket = nullptr;
      return false;
    }
  }
  return true;
}

// Implements a socketKeepAlive function and
// checks if pool is not sending any data to reconnect again.
// Even connection could be alive, pool could stop sending new job NOTIFY
unsigned long mStart0Hashrate = 0;

bool checkPoolInactivity(unsigned int keepAliveTime, unsigned long inactivityTime) {
  unsigned long currentKHashes = (Mhashes * 1000) + hashes / 1000;
  unsigned long elapsedKHs = currentKHashes - totalKHashes;

  uint32_t time_now = millis();

  // If no shares sent to pool
  // send something to pool to hold socket oppened
  if (time_now < mLastTXtoPool) {  // 32bit wrap
    mLastTXtoPool = time_now;
  }
  if (time_now > mLastTXtoPool + keepAliveTime) {
    mLastTXtoPool = time_now;
    ESP_LOGD(TAG, "Sending: KeepAlive - suggest_difficulty");
    tx_suggest_difficulty(pool_socket.get(), DEFAULT_DIFFICULTY);
  }

  if (elapsedKHs == 0) {
    // Check if hashrate is 0 during inactivityTIme
    if (mStart0Hashrate == 0) {
      mStart0Hashrate = time_now;
    }
    if ((time_now - mStart0Hashrate) > inactivityTime) {
      mStart0Hashrate = 0;
      return true;
    }
    return false;
  }

  mStart0Hashrate = 0;
  return false;
}

struct JobRequest {
  uint32_t id;
  uint32_t nonce_start;
  uint32_t nonce_count;
  double difficulty;
  uint8_t sha_buffer[128];
  uint32_t midstate[8];
  uint32_t bake[16];
};

struct JobResult {
  uint32_t id;
  uint32_t nonce;
  uint32_t nonce_count;
  double difficulty;
  uint8_t hash[32];
};

static std::mutex s_job_mutex;
std::list<std::shared_ptr<JobRequest>> s_job_request_list_sw;
#ifdef HARDWARE_SHA265
std::list<std::shared_ptr<JobRequest>> s_job_request_list_hw;
#endif
std::list<std::shared_ptr<JobResult>> s_job_result_list;
static volatile uint8_t s_working_current_job_id = 0xFF;

static void JobPush(std::list<std::shared_ptr<JobRequest>> &job_list, uint32_t id, uint32_t nonce_start,
                    uint32_t nonce_count, double difficulty, const uint8_t *sha_buffer, const uint32_t *midstate,
                    const uint32_t *bake) {
  std::shared_ptr<JobRequest> job = std::make_shared<JobRequest>();
  job->id = id;
  job->nonce_start = nonce_start;
  job->nonce_count = nonce_count;
  job->difficulty = difficulty;
  memcpy(job->sha_buffer, sha_buffer, sizeof(job->sha_buffer));
  memcpy(job->midstate, midstate, sizeof(job->midstate));
  memcpy(job->bake, bake, sizeof(job->bake));
  job_list.push_back(job);
}

struct Submition {
  double diff;
  bool is32bit;
  bool isValid;
};

static void MiningJobStop(uint32_t &job_pool, std::map<uint32_t, std::shared_ptr<Submition>> &submition_map) {
  {
    std::lock_guard<std::mutex> lock(s_job_mutex);
    s_job_result_list.clear();
    s_job_request_list_sw.clear();
#ifdef HARDWARE_SHA265
    s_job_request_list_hw.clear();
#endif
  }
  s_working_current_job_id = 0xFF;
  job_pool = 0xFFFFFFFF;
  submition_map.clear();
}

#ifdef RANDOM_NONCE
uint64_t s_random_state = 1;
static uint32_t RandomGet() {
  s_random_state += 0x9E3779B97F4A7C15ull;
  uint64_t z = s_random_state;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}
#endif

void runStratumWorker(void *name) {
  // TEST: https://bitcoin.stackexchange.com/questions/22929/full-example-data-for-scrypt-stratum-client
  ESP_LOGCONFIG(TAG, "[STRATUM] Started. Running %s on core %d", (char *) name, xPortGetCoreID());

#ifdef DEBUG_MEMORY
  ESP_LOGD(TAG, "### [Total Heap / Free heap / Min free heap]: %d / %d / %d", ESP.getHeapSize(), ESP.getFreeHeap(),
           ESP.getMinFreeHeap());
#endif

  std::map<uint32_t, std::shared_ptr<Submition>> s_submition_map;

  // Connect to pool
  double currentPoolDifficulty = DEFAULT_DIFFICULTY;

  uint32_t nonce_pool = 0;
  uint32_t job_pool = 0xFFFFFFFF;
  uint32_t last_job_time = millis();

  while (true) {
    if (!network::is_connected()) {
      // Network is disconnected, so wait for reconnect
      mMonitor.NerdStatus = NM_Connecting;
      MiningJobStop(job_pool, s_submition_map);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!checkPoolConnection()) {
      // If server is not reachable add random delay for connection retries
      // Generate value between 1 and 60 secs
      MiningJobStop(job_pool, s_submition_map);
      vTaskDelay(((1 + rand() % 60) * 1000) / portTICK_PERIOD_MS);
      continue;
    }

    if (!isMinerSuscribed) {
      // Stop miner current jobs
      mWorker = init_mining_subscribe();

      // STEP 1: Pool server connection (SUBSCRIBE)
      if (!tx_mining_subscribe(pool_socket.get(), mWorker)) {
        pool_close(pool_socket);
        MiningJobStop(job_pool, s_submition_map);
        continue;
      }

      // STEP 2: Pool authorize work (Block Info)
      strcpy(mWorker.wName, NERDMINER_POOL_WORKER);
      strcpy(mWorker.wPass, NERDMINER_POOL_PASS);
      tx_mining_auth(pool_socket.get(), mWorker.wName, mWorker.wPass);  // Don't verifies authoritzation, TODO
      // tx_mining_auth2(pool_socket.get(), mWorker.wName, mWorker.wPass); // Don't verifies authoritzation, TODO

      // STEP 3: Suggest pool difficulty
      tx_suggest_difficulty(pool_socket.get(), currentPoolDifficulty);

      isMinerSuscribed = true;
      uint32_t time_now = millis();
      mLastTXtoPool = time_now;
      last_job_time = time_now;
    }

    // Check if pool is down for almost 5minutes and then restart connection with pool (1min=600000ms)
    if (checkPoolInactivity(KEEPALIVE_TIME_ms, POOLINACTIVITY_TIME_ms)) {
      // Restart connection
      ESP_LOGI(TAG, "Detected more than 2 min without data form stratum server. Closing socket and reopening...");
      pool_close(pool_socket);
      isMinerSuscribed = false;
      MiningJobStop(job_pool, s_submition_map);
      continue;
    }

    {
      uint32_t time_now = millis();
      if (time_now < last_job_time)  // 32bit wrap
        last_job_time = time_now;
      if (time_now >= last_job_time + 10 * 60 * 1000)  // 10 minutes without job
      {
        pool_close(pool_socket);
        isMinerSuscribed = false;
        MiningJobStop(job_pool, s_submition_map);
        continue;
      }
    }

    uint32_t hw_midstate[8];
    uint32_t diget_mid[8];
    uint32_t bake[16];
#if defined(CONFIG_IDF_TARGET_ESP32)
    uint8_t sha_buffer_swap[128];
#endif

    // Read pending messages from pool
    while (pool_socket != nullptr && pool_available(pool_socket.get())) {
      ESP_LOGD(TAG, "Received message from pool...");
      std::string line = pool_read_until(pool_socket.get(), '\n');
      stratum_method result = parse_mining_method(line);
      switch (result) {
        case MINING_NOTIFY: {
          if (parse_mining_notify(line, mJob)) {
            {
              std::lock_guard<std::mutex> lock(s_job_mutex);
              s_job_request_list_sw.clear();
#ifdef HARDWARE_SHA265
              s_job_request_list_hw.clear();
#endif
            }
            // Increse templates readed
            templates++;

            job_pool++;
            s_working_current_job_id = job_pool & 0xFF;  // Terminate current job in thread

            last_job_time = millis();
            mLastTXtoPool = last_job_time;

            uint32_t mh = hashes / 1000000;
            Mhashes += mh;
            hashes -= mh * 1000000;

            // Prepare data for new jobs
            mMiner = calculateMiningData(mWorker, mJob);

            memset(mMiner.bytearray_blockheader + 80, 0, 128 - 80);
            mMiner.bytearray_blockheader[80] = 0x80;
            mMiner.bytearray_blockheader[126] = 0x02;
            mMiner.bytearray_blockheader[127] = 0x80;

            nerd_mids(diget_mid, mMiner.bytearray_blockheader);
            nerd_sha256_bake(diget_mid, mMiner.bytearray_blockheader + 64, bake);

#ifdef HARDWARE_SHA265
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
            esp_sha_acquire_hardware();
            sha_hal_hash_block(SHA2_256, mMiner.bytearray_blockheader, 64 / 4, true);
            sha_hal_read_digest(SHA2_256, hw_midstate);
            esp_sha_release_hardware();
#endif
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
            for (int i = 0; i < 32; ++i) {
              ((uint32_t *) sha_buffer_swap)[i] =
                  __builtin_bswap32(((const uint32_t *) (mMiner.bytearray_blockheader))[i]);
            }
#endif

#ifdef RANDOM_NONCE
            nonce_pool = RandomGet() & RANDOM_NONCE_MASK;
#else
            nonce_pool = 0xDA54E700;  // nonce 0x00000000 is not possible, start from some random nonce
#endif
            {
              std::lock_guard<std::mutex> lock(s_job_mutex);
              for (int i = 0; i < 4; ++i) {
                JobPush(s_job_request_list_sw, job_pool, nonce_pool, NONCE_PER_JOB_SW, currentPoolDifficulty,
                        mMiner.bytearray_blockheader, diget_mid, bake);
#ifdef RANDOM_NONCE
                nonce_pool = RandomGet() & RANDOM_NONCE_MASK;
#else
                nonce_pool += NONCE_PER_JOB_SW;
#endif

#ifdef HARDWARE_SHA265
#if defined(CONFIG_IDF_TARGET_ESP32)
                JobPush(s_job_request_list_hw, job_pool, nonce_pool, NONCE_PER_JOB_HW, currentPoolDifficulty,
                        sha_buffer_swap, hw_midstate, bake);
#else
                JobPush(s_job_request_list_hw, job_pool, nonce_pool, NONCE_PER_JOB_HW, currentPoolDifficulty,
                        mMiner.bytearray_blockheader, hw_midstate, bake);
#endif

#ifdef RANDOM_NONCE
                nonce_pool = RandomGet() & RANDOM_NONCE_MASK;
#else
                nonce_pool += NONCE_PER_JOB_HW;
#endif
#endif
              }
            }
          } else {
            ESP_LOGD(TAG, "Parsing error, need restart");
            pool_close(pool_socket);
            isMinerSuscribed = false;
            MiningJobStop(job_pool, s_submition_map);
          }
        } break;
        case MINING_SET_DIFFICULTY: {
          ESP_LOGD(TAG, "Parsed JSON: Minning set difficulty");
          parse_mining_set_difficulty(line, currentPoolDifficulty);
        } break;
        case STRATUM_SUCCESS: {
          ESP_LOGD(TAG, "Parsed JSON: Success");
          unsigned long id = parse_extract_id(line);
          auto itt = s_submition_map.find(id);
          if (itt != s_submition_map.end()) {
            if (itt->second->diff > best_diff)
              best_diff = itt->second->diff;
            if (itt->second->is32bit)
              shares++;
            if (itt->second->isValid) {
              ESP_LOGD(TAG, "CONGRATULATIONS! Valid block found");
              valids++;
            }
            s_submition_map.erase(itt);
          }
        } break;
        case STRATUM_PARSE_ERROR: {
          ESP_LOGD(TAG, "Parsed JSON: error on JSON");
          unsigned long id = parse_extract_id(line);
          auto itt = s_submition_map.find(id);
          if (itt != s_submition_map.end()) {
            ESP_LOGD(TAG, "Refuse submition %d", id);
            s_submition_map.erase(itt);
          }
        } break;
        default:
          ESP_LOGD(TAG, "Parsed JSON: Unknown");
          break;
      }
    }

    std::list<std::shared_ptr<JobResult>> job_result_list;

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Small delay

    if (job_pool != 0xFFFFFFFF) {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      job_result_list.insert(job_result_list.end(), s_job_result_list.begin(), s_job_result_list.end());
      s_job_result_list.clear();

      while (s_job_request_list_sw.size() < JOB_QUEUE_SIZE) {
        JobPush(s_job_request_list_sw, job_pool, nonce_pool, NONCE_PER_JOB_SW, currentPoolDifficulty,
                mMiner.bytearray_blockheader, diget_mid, bake);

#ifdef RANDOM_NONCE
        nonce_pool = RandomGet() & RANDOM_NONCE_MASK;
#else
        nonce_pool += NONCE_PER_JOB_SW;
#endif
      }

#ifdef HARDWARE_SHA265
      while (s_job_request_list_hw.size() < JOB_QUEUE_SIZE) {
#if defined(CONFIG_IDF_TARGET_ESP32)
        JobPush(s_job_request_list_hw, job_pool, nonce_pool, NONCE_PER_JOB_HW, currentPoolDifficulty, sha_buffer_swap,
                hw_midstate, bake);
#else
        JobPush(s_job_request_list_hw, job_pool, nonce_pool, NONCE_PER_JOB_HW, currentPoolDifficulty,
                mMiner.bytearray_blockheader, hw_midstate, bake);
#endif

#ifdef RANDOM_NONCE
        nonce_pool = RandomGet() & RANDOM_NONCE_MASK;
#else
        nonce_pool += NONCE_PER_JOB_HW;
#endif
      }
#endif
    }

    while (!job_result_list.empty()) {
      std::shared_ptr<JobResult> res = job_result_list.front();
      job_result_list.pop_front();

      hashes += res->nonce_count;
      if (res->difficulty > currentPoolDifficulty && job_pool == res->id && res->nonce != 0xFFFFFFFF) {
        if (pool_socket == nullptr)
          break;

        unsigned long sumbit_id = 0;
        tx_mining_submit(pool_socket.get(), mWorker, mJob, res->nonce, sumbit_id);
        ESP_LOGD(TAG, "  - Current diff share: %d", res->difficulty);
        ESP_LOGD(TAG, "  - Current pool diff : %d", currentPoolDifficulty);
        ESP_LOGD(TAG, "  - TX SHARE: %s", format_hex_pretty(res->hash, 32).c_str());
        mLastTXtoPool = millis();

        std::shared_ptr<Submition> submition = std::make_shared<Submition>();
        submition->diff = res->difficulty;
        submition->is32bit = (res->hash[29] == 0 && res->hash[28] == 0);
        if (submition->is32bit) {
          submition->isValid = checkValid(res->hash, mMiner.bytearray_target);
        } else {
          submition->isValid = false;
        }

        s_submition_map.insert(std::make_pair(sumbit_id, submition));
        if (s_submition_map.size() > 32)
          s_submition_map.erase(s_submition_map.begin());
      }
    }
  }
}

////////////////// THREAD CALLS ///////////////////

void minerWorkerSw(void *task_id) {
  unsigned int miner_id = (uint32_t) task_id;
  ESP_LOGCONFIG(TAG, "[MINER][SW] %d Started...", miner_id);

  std::shared_ptr<JobRequest> job;
  std::shared_ptr<JobResult> result;

  uint8_t hash[32];
  uint32_t wdt_counter = 0;

  while (1) {
    {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      if (result) {
        if (s_job_result_list.size() < RESULT_QUEUE_SIZE)
          s_job_result_list.push_back(result);
        result.reset();
      }
      if (!s_job_request_list_sw.empty()) {
        job = s_job_request_list_sw.front();
        s_job_request_list_sw.pop_front();
      } else {
        job.reset();
      }
    }

    if (job) {
      mMonitor.NerdStatus = NM_Hashing;

      result = std::make_shared<JobResult>();
      result->difficulty = job->difficulty;
      result->nonce = 0xFFFFFFFF;
      result->id = job->id;
      result->nonce_count = job->nonce_count;
      uint8_t job_in_work = job->id & 0xFF;
      for (uint32_t n = 0; n < job->nonce_count; ++n) {
        ((uint32_t *) (job->sha_buffer + 64 + 12))[0] = job->nonce_start + n;
        if (nerd_sha256d_baked(job->midstate, job->sha_buffer + 64, job->bake, hash)) {
          double diff_hash = diff_from_target(hash);
          if (diff_hash > result->difficulty) {
            result->difficulty = diff_hash;
            result->nonce = job->nonce_start + n;
            memcpy(result->hash, hash, 32);
          }
        }

        if ((uint16_t) (n & 0xFF) == 0 && s_working_current_job_id != job_in_work) {
          result->nonce_count = n + 1;
          break;
        }
      }
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    wdt_counter++;
    if (wdt_counter >= 8) {
      wdt_counter = 0;
      esp_task_wdt_reset();
    }
  }
}

#ifdef HARDWARE_SHA265

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)

static inline IRAM_ATTR void nerd_sha_ll_fill_text_block_sha256(const void *input_text, uint32_t nonce) {
  uint32_t *data_words = (uint32_t *) input_text;
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_TEXT_BASE);

  REG_WRITE(&reg_addr_buf[0], data_words[0]);
  REG_WRITE(&reg_addr_buf[1], data_words[1]);
  REG_WRITE(&reg_addr_buf[2], data_words[2]);

  REG_WRITE(&reg_addr_buf[3], nonce);
  REG_WRITE(&reg_addr_buf[4], 0x00000080);
  REG_WRITE(&reg_addr_buf[5], 0x00000000);
  REG_WRITE(&reg_addr_buf[6], 0x00000000);
  REG_WRITE(&reg_addr_buf[7], 0x00000000);
  REG_WRITE(&reg_addr_buf[8], 0x00000000);
  REG_WRITE(&reg_addr_buf[9], 0x00000000);
  REG_WRITE(&reg_addr_buf[10], 0x00000000);
  REG_WRITE(&reg_addr_buf[11], 0x00000000);
  REG_WRITE(&reg_addr_buf[12], 0x00000000);
  REG_WRITE(&reg_addr_buf[13], 0x00000000);
  REG_WRITE(&reg_addr_buf[14], 0x00000000);
  REG_WRITE(&reg_addr_buf[15], 0x80020000);
}

static inline IRAM_ATTR void nerd_sha_ll_fill_text_block_sha256_inter() {
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_TEXT_BASE);

  DPORT_INTERRUPT_DISABLE();
  REG_WRITE(&reg_addr_buf[0], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 0 * 4));
  REG_WRITE(&reg_addr_buf[1], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 1 * 4));
  REG_WRITE(&reg_addr_buf[2], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 2 * 4));
  REG_WRITE(&reg_addr_buf[3], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 3 * 4));
  REG_WRITE(&reg_addr_buf[4], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 4 * 4));
  REG_WRITE(&reg_addr_buf[5], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 5 * 4));
  REG_WRITE(&reg_addr_buf[6], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 6 * 4));
  REG_WRITE(&reg_addr_buf[7], DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 7 * 4));
  DPORT_INTERRUPT_RESTORE();

  REG_WRITE(&reg_addr_buf[8], 0x00000080);
  REG_WRITE(&reg_addr_buf[9], 0x00000000);
  REG_WRITE(&reg_addr_buf[10], 0x00000000);
  REG_WRITE(&reg_addr_buf[11], 0x00000000);
  REG_WRITE(&reg_addr_buf[12], 0x00000000);
  REG_WRITE(&reg_addr_buf[13], 0x00000000);
  REG_WRITE(&reg_addr_buf[14], 0x00000000);
  REG_WRITE(&reg_addr_buf[15], 0x00010000);
}

static inline IRAM_ATTR void nerd_sha_ll_read_digest(void *ptr) {
  DPORT_INTERRUPT_DISABLE();
  ((uint32_t *) ptr)[0] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 0 * 4);
  ((uint32_t *) ptr)[1] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 1 * 4);
  ((uint32_t *) ptr)[2] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 2 * 4);
  ((uint32_t *) ptr)[3] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 3 * 4);
  ((uint32_t *) ptr)[4] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 4 * 4);
  ((uint32_t *) ptr)[5] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 5 * 4);
  ((uint32_t *) ptr)[6] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 6 * 4);
  ((uint32_t *) ptr)[7] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 7 * 4);
  DPORT_INTERRUPT_RESTORE();
}

static inline IRAM_ATTR bool nerd_sha_ll_read_digest_if(void *ptr) {
  DPORT_INTERRUPT_DISABLE();
  uint32_t last = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 7 * 4);

  if ((uint16_t) (last >> 16) != 0) {
    DPORT_INTERRUPT_RESTORE();
    return false;
  }

  ((uint32_t *) ptr)[7] = last;
  ((uint32_t *) ptr)[0] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 0 * 4);
  ((uint32_t *) ptr)[1] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 1 * 4);
  ((uint32_t *) ptr)[2] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 2 * 4);
  ((uint32_t *) ptr)[3] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 3 * 4);
  ((uint32_t *) ptr)[4] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 4 * 4);
  ((uint32_t *) ptr)[5] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 5 * 4);
  ((uint32_t *) ptr)[6] = DPORT_SEQUENCE_REG_READ(SHA_H_BASE + 6 * 4);

  DPORT_INTERRUPT_RESTORE();
  return true;
}

static inline IRAM_ATTR void nerd_sha_ll_write_digest(void *digest_state) {
  uint32_t *digest_state_words = (uint32_t *) digest_state;
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_H_BASE);

  REG_WRITE(&reg_addr_buf[0], digest_state_words[0]);
  REG_WRITE(&reg_addr_buf[1], digest_state_words[1]);
  REG_WRITE(&reg_addr_buf[2], digest_state_words[2]);
  REG_WRITE(&reg_addr_buf[3], digest_state_words[3]);
  REG_WRITE(&reg_addr_buf[4], digest_state_words[4]);
  REG_WRITE(&reg_addr_buf[5], digest_state_words[5]);
  REG_WRITE(&reg_addr_buf[6], digest_state_words[6]);
  REG_WRITE(&reg_addr_buf[7], digest_state_words[7]);
}

static inline IRAM_ATTR void nerd_sha_hal_wait_idle() {
  while (REG_READ(SHA_BUSY_REG)) {
  }
}

void minerWorkerHw(void *task_id) {
  unsigned int miner_id = (uint32_t) task_id;
  ESP_LOGCONFIG(TAG, "[MINER][HW] %d Started...", miner_id);

  std::shared_ptr<JobRequest> job;
  std::shared_ptr<JobResult> result;

  uint8_t interResult[64];
  uint8_t hash[32];
  uint8_t digest_mid[32];
  uint8_t sha_buffer[64];
  uint32_t wdt_counter = 0;

  while (1) {
    {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      if (result) {
        if (s_job_result_list.size() < RESULT_QUEUE_SIZE)
          s_job_result_list.push_back(result);
        result.reset();
      }
      if (!s_job_request_list_hw.empty()) {
        job = s_job_request_list_hw.front();
        s_job_request_list_hw.pop_front();
      } else {
        job.reset();
      }
    }

    if (job) {
      mMonitor.NerdStatus = NM_Hashing;

      result = std::make_shared<JobResult>();
      result->id = job->id;
      result->nonce = 0xFFFFFFFF;
      result->nonce_count = job->nonce_count;
      result->difficulty = job->difficulty;
      uint8_t job_in_work = job->id & 0xFF;
      memcpy(digest_mid, job->midstate, sizeof(digest_mid));
      memcpy(sha_buffer, job->sha_buffer + 64, sizeof(sha_buffer));

      esp_sha_acquire_hardware();
      REG_WRITE(SHA_MODE_REG, SHA2_256);
      uint32_t nend = job->nonce_start + job->nonce_count;
      for (uint32_t n = job->nonce_start; n < nend; ++n) {
        nerd_sha_ll_write_digest(digest_mid);
        nerd_sha_ll_fill_text_block_sha256(sha_buffer, n);
        REG_WRITE(SHA_CONTINUE_REG, 1);

        sha_ll_load(SHA2_256);

        nerd_sha_hal_wait_idle();
        nerd_sha_ll_fill_text_block_sha256_inter();
        REG_WRITE(SHA_START_REG, 1);
        sha_ll_load(SHA2_256);

        nerd_sha_hal_wait_idle();
        if (nerd_sha_ll_read_digest_if(hash)) {
          // ~5 per second
          double diff_hash = diff_from_target(hash);
          if (diff_hash > result->difficulty) {
            if (isSha256Valid(hash)) {
              result->difficulty = diff_hash;
              result->nonce = n;
              memcpy(result->hash, hash, sizeof(hash));
            }
          }
        }
        if ((uint8_t) (n & 0xFF) == 0 && s_working_current_job_id != job_in_work) {
          result->nonce_count = n - job->nonce_start + 1;
          break;
        }
      }
      esp_sha_release_hardware();
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    wdt_counter++;
    if (wdt_counter >= 8) {
      wdt_counter = 0;
      esp_task_wdt_reset();
    }
  }
}

#endif  // #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) ||
        // defined(CONFIG_IDF_TARGET_ESP32C3)

#if defined(CONFIG_IDF_TARGET_ESP32)

static inline IRAM_ATTR bool nerd_sha_ll_read_digest_swap_if(void *ptr) {
  DPORT_INTERRUPT_DISABLE();
  uint32_t fin = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 7 * 4);
  if ((uint32_t) (fin & 0xFFFF) != 0) {
    DPORT_INTERRUPT_RESTORE();
    return false;
  }

  ((uint32_t *) ptr)[7] = __builtin_bswap32(fin);
  ((uint32_t *) ptr)[0] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 0 * 4));
  ((uint32_t *) ptr)[1] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 1 * 4));
  ((uint32_t *) ptr)[2] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 2 * 4));
  ((uint32_t *) ptr)[3] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 3 * 4));
  ((uint32_t *) ptr)[4] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 4 * 4));
  ((uint32_t *) ptr)[5] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 5 * 4));
  ((uint32_t *) ptr)[6] = __builtin_bswap32(DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 6 * 4));

  DPORT_INTERRUPT_RESTORE();
  return true;
}

static inline IRAM_ATTR void nerd_sha_ll_read_digest(void *ptr) {
  DPORT_INTERRUPT_DISABLE();

  ((uint32_t *) ptr)[0] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 0 * 4);
  ((uint32_t *) ptr)[1] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 1 * 4);
  ((uint32_t *) ptr)[2] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 2 * 4);
  ((uint32_t *) ptr)[3] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 3 * 4);
  ((uint32_t *) ptr)[4] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 4 * 4);
  ((uint32_t *) ptr)[5] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 5 * 4);
  ((uint32_t *) ptr)[6] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 6 * 4);
  ((uint32_t *) ptr)[7] = DPORT_SEQUENCE_REG_READ(SHA_TEXT_BASE + 7 * 4);

  DPORT_INTERRUPT_RESTORE();
}

static inline IRAM_ATTR void nerd_sha_hal_wait_idle() {
  while (DPORT_REG_READ(SHA_256_BUSY_REG)) {
  }
}

static inline IRAM_ATTR void nerd_sha_ll_fill_text_block_sha256(const void *input_text) {
  uint32_t *data_words = (uint32_t *) input_text;
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_TEXT_BASE);

  reg_addr_buf[0] = data_words[0];
  reg_addr_buf[1] = data_words[1];
  reg_addr_buf[2] = data_words[2];
  reg_addr_buf[3] = data_words[3];
  reg_addr_buf[4] = data_words[4];
  reg_addr_buf[5] = data_words[5];
  reg_addr_buf[6] = data_words[6];
  reg_addr_buf[7] = data_words[7];
  reg_addr_buf[8] = data_words[8];
  reg_addr_buf[9] = data_words[9];
  reg_addr_buf[10] = data_words[10];
  reg_addr_buf[11] = data_words[11];
  reg_addr_buf[12] = data_words[12];
  reg_addr_buf[13] = data_words[13];
  reg_addr_buf[14] = data_words[14];
  reg_addr_buf[15] = data_words[15];
}

static inline IRAM_ATTR void nerd_sha_ll_fill_text_block_sha256_upper(const void *input_text, uint32_t nonce) {
  uint32_t *data_words = (uint32_t *) input_text;
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_TEXT_BASE);

  reg_addr_buf[0] = data_words[0];
  reg_addr_buf[1] = data_words[1];
  reg_addr_buf[2] = data_words[2];
  reg_addr_buf[3] = __builtin_bswap32(nonce);

  reg_addr_buf[4] = 0x80000000;
  reg_addr_buf[5] = 0x00000000;
  reg_addr_buf[6] = 0x00000000;
  reg_addr_buf[7] = 0x00000000;
  reg_addr_buf[8] = 0x00000000;
  reg_addr_buf[9] = 0x00000000;
  reg_addr_buf[10] = 0x00000000;
  reg_addr_buf[11] = 0x00000000;
  reg_addr_buf[12] = 0x00000000;
  reg_addr_buf[13] = 0x00000000;
  reg_addr_buf[14] = 0x00000000;
  reg_addr_buf[15] = 0x00000280;
}

static inline IRAM_ATTR void nerd_sha_ll_fill_text_block_sha256_double() {
  uint32_t *reg_addr_buf = (uint32_t *) (SHA_TEXT_BASE);

  reg_addr_buf[8] = 0x80000000;
  reg_addr_buf[9] = 0x00000000;
  reg_addr_buf[10] = 0x00000000;
  reg_addr_buf[11] = 0x00000000;
  reg_addr_buf[12] = 0x00000000;
  reg_addr_buf[13] = 0x00000000;
  reg_addr_buf[14] = 0x00000000;
  reg_addr_buf[15] = 0x00000100;
}

void minerWorkerHw(void *task_id) {
  unsigned int miner_id = (uint32_t) task_id;
  ESP_LOGCONFIG(TAG, "[MINER][HW32D] %d Started...", miner_id);

  std::shared_ptr<JobRequest> job;
  std::shared_ptr<JobResult> result;

  uint8_t hash[32];
  uint8_t sha_buffer[128];

  while (1) {
    {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      if (result) {
        if (s_job_result_list.size() < RESULT_QUEUE_SIZE) {
          s_job_result_list.push_back(result);
        }
        result.reset();
      }
      if (!s_job_request_list_hw.empty()) {
        job = s_job_request_list_hw.front();
        s_job_request_list_hw.pop_front();
      } else {
        job.reset();
      }
    }

    if (job) {
      mMonitor.NerdStatus = NM_Hashing;

      result = std::make_shared<JobResult>();
      result->id = job->id;
      result->nonce = 0xFFFFFFFF;
      result->nonce_count = job->nonce_count;
      result->difficulty = job->difficulty;
      uint8_t job_in_work = job->id & 0xFF;
      memcpy(sha_buffer, job->sha_buffer, 80);

      esp_sha_lock_engine(SHA2_256);
      uint32_t processed_nonces = 0;  // Track actually processed nonces
      for (uint32_t n = 0; n < job->nonce_count; ++n) {
        nerd_sha_ll_fill_text_block_sha256(sha_buffer);
        sha_ll_start_block(SHA2_256);

        nerd_sha_hal_wait_idle();
        nerd_sha_ll_fill_text_block_sha256_upper(sha_buffer + 64, job->nonce_start + n);
        sha_ll_continue_block(SHA2_256);

        nerd_sha_hal_wait_idle();
        sha_ll_load(SHA2_256);

        nerd_sha_hal_wait_idle();
        nerd_sha_ll_fill_text_block_sha256_double();
        sha_ll_start_block(SHA2_256);

        nerd_sha_hal_wait_idle();
        sha_ll_load(SHA2_256);
        if (nerd_sha_ll_read_digest_swap_if(hash)) {
          processed_nonces++;  // Only count successful hash operations

          // ~5 per second
          double diff_hash = diff_from_target(hash);
          if (diff_hash > result->difficulty) {
            if (isSha256Valid(hash)) {
              result->difficulty = diff_hash;
              result->nonce = job->nonce_start + n;
              memcpy(result->hash, hash, sizeof(hash));
            }
          }
        }
        if ((uint8_t) (n & 0xFF) == 0 && s_working_current_job_id != job_in_work) {
          result->nonce_count = processed_nonces;  // Use actual processed count
          break;
        }
      }
      // Update final count with actual processed nonces
      result->nonce_count = processed_nonces;
      esp_sha_unlock_engine(SHA2_256);
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    esp_task_wdt_reset();
  }
}

#endif  // CONFIG_IDF_TARGET_ESP32

#endif  // HARDWARE_SHA265

#define DELAY 100

void resetStat() {
  ESP_LOGD(TAG, "[MONITOR] Resetting stats");
  templates = hashes = Mhashes = totalKHashes = elapsedKHs = upTime = shares = valids = 0;
  best_diff = 0.0;
}

void runMonitor(void *name) {
  ESP_LOGCONFIG(TAG, "[MONITOR] Started...");

  unsigned long mLastCheck = 0;
  uint32_t uptime_frac = 0;

  mMonitor.Status = false;
  totalKHashes = (Mhashes * 1000) + hashes / 1000;

  while (1) {
    unsigned long now_millis = millis();

    mMonitor.Status = (pool_socket != nullptr) && isMinerSuscribed;

    unsigned long mElapsed = now_millis - mLastCheck;
    if (mElapsed >= 1000) {
      mLastCheck = now_millis;

      unsigned long currentKHashes = (Mhashes * 1000) + hashes / 1000;
      elapsedKHs = currentKHashes - totalKHashes;
      totalKHashes = currentKHashes;

      uptime_frac += mElapsed;
      while (uptime_frac >= 1000) {
        uptime_frac -= 1000;
        upTime++;
      }

      updateMiningData(mElapsed);

      // Monitor state when hashrate is 0.0
      if (elapsedKHs == 0) {
        ESP_LOGD(TAG,
                 ">>> [MONITOR] Miner: newJob>%s / inRun>%s) - Client: connected>%s / subscribed>%s / wificonnected>%s",
                 YESNO(true), YESNO(isMinerSuscribed), YESNO((pool_socket != nullptr)), YESNO(isMinerSuscribed),
                 YESNO(network::is_connected()));
      }

#ifdef DEBUG_MEMORY
      ESP_LOGD(TAG, "### [Total Heap / Free heap / Min free heap]: %d / %d / %d", ESP.getHeapSize(), ESP.getFreeHeap(),
               ESP.getMinFreeHeap());
      ESP_LOGD(TAG, "### Max stack usage: %d", uxTaskGetStackHighWaterMark(NULL));
#endif
    }

    vTaskDelay(DELAY / portTICK_PERIOD_MS);
  }
}

}  // namespace nerdminer
}  // namespace esphome
