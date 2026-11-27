#include "mining.h"
#include "monitor.h"
#include "nerdminer.h"
#include "stratum.h"
#include "timeconst.h"
#include "utils.h"

#include <atomic>
#include <cstring>
#include <deque>
#include <mutex>

#include <lwip/netdb.h>
#include <lwip/sockets.h>

#include <esp_random.h>
#include <mbedtls/sha256.h>

#if defined(CONFIG_IDF_TARGET_ESP32)
#include <sha/sha_parallel_engine.h>
#include "sha256_pipelined.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#include <hal/sha_hal.h>
#include <hal/sha_ll.h>
#include <sha/sha_dma.h>
#endif

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::nerdminer {

namespace {

#if defined(CONFIG_IDF_TARGET_ESP32)
constexpr uint32_t HW_BATCH_SIZE = 65536U;
#else
constexpr uint32_t HW_BATCH_SIZE = 16384U;
#endif
constexpr uint32_t SW_BATCH_SIZE = 4096U;
constexpr uint32_t SW_YIELD_INTERVAL = 512U;
constexpr uint32_t HW_YIELD_BATCHES = 16U;
constexpr double DEFAULT_DIFFICULTY = 0.00015;
constexpr uint32_t KEEPALIVE_TIME_MS = 30U * 60U * 1000U;

struct ActiveJob {
  uint32_t version{0};
  mining_subscribe worker{};
  mining_job job{};
  miner_data data{};
  double pool_difficulty{0.0};
};

struct ShareSubmission {
  mining_subscribe worker{};
  mining_job job{};
  uint32_t nonce{0};
  double diff{0.0};
  bool valid_block{false};
};

std::mutex s_job_mutex;
ActiveJob s_active_job{};
std::atomic<uint32_t> s_job_version{0};
std::atomic<uint32_t> s_next_nonce{0};
std::atomic<bool> s_mining_running{true};
volatile bool s_hw_mining_active{true};

std::mutex s_submit_mutex;
std::deque<ShareSubmission> s_submit_queue;

std::atomic<uint32_t> s_hashes{0};
std::atomic<uint32_t> s_shares{0};
std::atomic<uint32_t> s_valids{0};
std::atomic<double> s_best_diff{0.0};

void update_best_diff(double candidate_diff) {
  double prev = s_best_diff.load(std::memory_order_relaxed);
  while (candidate_diff > prev && !s_best_diff.compare_exchange_weak(prev, candidate_diff, std::memory_order_relaxed)) {
  }
}

void record_candidate(const ActiveJob &local_job, const uint8_t *hash) {
  double diff_hash = diff_from_target(const_cast<uint8_t *>(hash));
  update_best_diff(diff_hash);

  if (diff_hash > local_job.pool_difficulty) {
    bool is_block = checkValid(const_cast<uint8_t *>(hash), const_cast<uint8_t *>(local_job.data.bytearray_target));

    ShareSubmission sub{};
    sub.worker = local_job.worker;
    sub.job = local_job.job;
    memcpy(&sub.nonce, local_job.data.bytearray_blockheader + 76, sizeof(uint32_t));
    sub.diff = diff_hash;
    sub.valid_block = is_block;

    std::lock_guard<std::mutex> lock(s_submit_mutex);
    if (s_submit_queue.size() < 16) {
      s_submit_queue.push_back(std::move(sub));
    }
  }
}

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
inline void IRAM_ATTR nerd_sha_ll_write_digest(const void *digest_state) {
  uint32_t *digest_state_words = reinterpret_cast<uint32_t *>(const_cast<void *>(digest_state));
  uint32_t *reg_address_words = reinterpret_cast<uint32_t *>(SHA_H_BASE);
  for (int i = 0; i < 8; i++) {
    REG_WRITE(&reg_address_words[i], digest_state_words[i]);
  }
}

inline void IRAM_ATTR nerd_sha_hal_wait_idle() {
  while (REG_READ(SHA_BUSY_REG)) {
  }
}
#endif

}  // namespace

// Global counters read by monitor.cpp
uint32_t templates = 0;
uint32_t Mhashes = 0;
uint32_t totalKHashes = 0;
uint32_t elapsedKHs = 0;
uint64_t upTime = 0;

uint32_t shares = 0;
uint32_t valids = 0;
double best_diff = 0.0;

monitor_data mMonitor{};

void init_miner_sha() {
#if defined(CONFIG_IDF_TARGET_ESP32)
  static bool initialized = false;
  if (initialized) {
    return;
  }
  initialized = true;

  sha256_pipelined_init();

  // Claim all SHA engines permanently on classic ESP32 so mbedTLS uses software SHA
  // during TLS/HTTPS sessions and never collides with pipelined assembly on SHA_TEXT_BASE.
  const esp_sha_type sha_types[] = {SHA1, SHA2_256, SHA2_384, SHA2_512};
  for (esp_sha_type type : sha_types) {
    bool ok = esp_sha_try_lock_engine(type);
    ESP_LOGD(TAG, "HW SHA engine %d %s", static_cast<int>(type), ok ? "claimed for mining" : "already locked");
  }
#endif
}

void stopMining() {
  s_hw_mining_active = false;
  s_mining_running.store(false, std::memory_order_release);
}

void runMonitor(void *name) {
  ESP_LOGD(TAG, "Starting %s", static_cast<char *>(name));

  unsigned long m_last_update = millis();

  while (s_mining_running.load(std::memory_order_acquire)) {
    unsigned long now = millis();
    unsigned long m_elapsed = now - m_last_update;

    if (m_elapsed >= 1000) {
      m_last_update = now;

      uint32_t pending_hashes = s_hashes.load(std::memory_order_relaxed);
      uint32_t current_khashes = (Mhashes * 1000U) + (pending_hashes / 1000U);
      if (current_khashes >= totalKHashes) {
        elapsedKHs = current_khashes - totalKHashes;
        totalKHashes = current_khashes;
      } else {
        elapsedKHs = 0;
      }

      shares = s_shares.load(std::memory_order_relaxed);
      valids = s_valids.load(std::memory_order_relaxed);
      best_diff = s_best_diff.load(std::memory_order_relaxed);

      updateMiningData(m_elapsed);

      upTime += m_elapsed / 1000ULL;
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
  vTaskDelete(nullptr);
}

void runStratumWorker(void *name) {
  ESP_LOGD(TAG, "Starting %s", static_cast<char *>(name));

  s_hw_mining_active = true;
  s_mining_running.store(true, std::memory_order_release);

  unsigned long submit_id = 0;
  uint32_t last_job_time = millis();
  double current_pool_difficulty = DEFAULT_DIFFICULTY;

  mMonitor.NerdStatus = NM_Connecting;

  std::unique_ptr<esphome::socket::Socket> client = nullptr;
  mining_subscribe m_worker = init_mining_subscribe();
  mining_job m_job{};
  miner_data m_miner_data{};
  bool is_subscribed = false;

  while (s_mining_running.load(std::memory_order_acquire)) {
    if (!esphome::network::is_connected()) {
      mMonitor.NerdStatus = NM_Connecting;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!pool_connected(client.get())) {
      ESP_LOGD(TAG, "Client not connected, trying to connect...");
      pool_close(client);
      is_subscribed = false;
      mMonitor.NerdStatus = NM_Connecting;

      client = esphome::socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
      if (client == nullptr) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      client->setblocking(false);

      struct addrinfo hints{};
      hints.ai_family = AF_INET;
      hints.ai_socktype = SOCK_STREAM;
      struct addrinfo *res = nullptr;

      char port_str[16];
      snprintf(port_str, sizeof(port_str), "%d", NERDMINER_POOL_PORT);

      int err = getaddrinfo(NERDMINER_POOL, port_str, &hints, &res);
      if (err != 0 || res == nullptr) {
        ESP_LOGE(TAG, "DNS lookup failed for %s: %d", NERDMINER_POOL, err);
        if (res != nullptr) {
          freeaddrinfo(res);
        }
        pool_close(client);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      int conn_res = client->connect(res->ai_addr, res->ai_addrlen);
      freeaddrinfo(res);

      if (conn_res < 0 && errno != EINPROGRESS) {
        ESP_LOGE(TAG, "Unable to connect to %s:%d (errno %d)", NERDMINER_POOL, NERDMINER_POOL_PORT, errno);
        pool_close(client);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      bool connected = false;
      int fd = client->get_fd();
      if (fd >= 0) {
        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(fd, &write_fds);

        struct timeval tv{};
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        int sel = lwip_select(fd + 1, nullptr, &write_fds, nullptr, &tv);
        if (sel > 0 && FD_ISSET(fd, &write_fds)) {
          int sock_err = 0;
          socklen_t len = sizeof(sock_err);
          if (client->getsockopt(SOL_SOCKET, SO_ERROR, &sock_err, &len) == 0 && sock_err == 0) {
            connected = true;
          }
        }
      }

      if (!connected) {
        ESP_LOGE(TAG, "Connection timeout to %s:%d", NERDMINER_POOL, NERDMINER_POOL_PORT);
        pool_close(client);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      ESP_LOGD(TAG, "Connected to %s:%d", NERDMINER_POOL, NERDMINER_POOL_PORT);

      m_worker = init_mining_subscribe();
      if (!tx_mining_subscribe(client.get(), m_worker)) {
        ESP_LOGE(TAG, "Failed to subscribe");
        pool_close(client);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      strcpy(m_worker.wName, NERDMINER_POOL_WORKER);
      strcpy(m_worker.wPass, NERDMINER_POOL_PASS);

      if (!tx_mining_auth(client.get(), m_worker.wName, m_worker.wPass)) {
        ESP_LOGE(TAG, "Failed to authorize");
        pool_close(client);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }

      tx_suggest_difficulty(client.get(), DEFAULT_DIFFICULTY);
      is_subscribed = true;
      last_job_time = millis();
    }

    // Drain any pending share submissions from mining workers
    while (true) {
      ShareSubmission sub{};
      {
        std::lock_guard<std::mutex> lock(s_submit_mutex);
        if (s_submit_queue.empty()) {
          break;
        }
        sub = std::move(s_submit_queue.front());
        s_submit_queue.pop_front();
      }

      tx_mining_submit(client.get(), sub.worker, sub.job, sub.nonce, submit_id);
      ESP_LOGD(TAG, " - Current diff: %f", sub.diff);
      ESP_LOGD(TAG, " - Pool diff: %f", current_pool_difficulty);

      s_shares.fetch_add(1, std::memory_order_relaxed);
      if (sub.valid_block) {
        ESP_LOGD(TAG, "CONGRATULATIONS! Valid block found with diff: %f", sub.diff);
        s_valids.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGD(TAG, "  - Valid blocks: %lu", static_cast<unsigned long>(s_valids.load(std::memory_order_relaxed)));
      }
    }

    if ((millis() - last_job_time) > KEEPALIVE_TIME_MS) {
      ESP_LOGD(TAG, "Keepalive timeout, reconnecting...");
      pool_close(client);
      is_subscribed = false;
      mMonitor.NerdStatus = NM_Connecting;
      continue;
    }

    while (pool_available(client.get())) {
      std::string line = pool_read_until(client.get(), '\n');
      if (line.empty()) {
        continue;
      }

      ESP_LOGD(TAG, "  Received: %s", line.c_str());
      stratum_method result = parse_mining_method(line);
      switch (result) {
        case STRATUM_PARSE_ERROR:
          ESP_LOGD(TAG, "  Parsed JSON: error on JSON");
          break;

        case MINING_NOTIFY:
          if (parse_mining_notify(line, m_job)) {
            static uint32_t hash_remainder = 0;
            uint32_t total_pending = hash_remainder + s_hashes.exchange(0, std::memory_order_relaxed);
            Mhashes += total_pending / 1000000U;
            hash_remainder = total_pending % 1000000U;

            m_miner_data = calculateMiningData(m_worker, m_job);
            m_job.coinb1.clear();
            m_job.coinb2.clear();
            m_job.merkle_branch.clear();

            {
              std::lock_guard<std::mutex> lock(s_job_mutex);
              s_active_job.version++;
              s_active_job.worker = m_worker;
              s_active_job.job = m_job;
              s_active_job.data = m_miner_data;
              s_active_job.pool_difficulty = current_pool_difficulty;
              s_next_nonce.store(esp_random(), std::memory_order_relaxed);
              s_job_version.store(s_active_job.version, std::memory_order_release);
            }

            templates++;
            last_job_time = millis();
            mMonitor.NerdStatus = NM_Hashing;
          }
          break;

        case MINING_SET_DIFFICULTY:
          parse_mining_set_difficulty(line, current_pool_difficulty);
          {
            std::lock_guard<std::mutex> lock(s_job_mutex);
            s_active_job.pool_difficulty = current_pool_difficulty;
          }
          break;

        case STRATUM_SUCCESS:
          ESP_LOGD(TAG, "  Parsed JSON: Success");
          break;

        default:
          ESP_LOGD(TAG, "  Parsed JSON: unknown");
          break;
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }

  pool_close(client);
  vTaskDelete(nullptr);
}

void minerWorkerSw(void *task_id) {
  ESP_LOGD(TAG, "Starting %s on core %d", static_cast<char *>(task_id), xPortGetCoreID());

  ActiveJob local_job{};
  uint32_t cached_version = 0;
  uint8_t tail_and_nonce[16];
  uint8_t inter_hash[32];
  uint8_t hash[32];

  mbedtls_sha256_context base_ctx;
  mbedtls_sha256_context work_ctx;
  mbedtls_sha256_init(&base_ctx);
  mbedtls_sha256_init(&work_ctx);

  while (s_mining_running.load(std::memory_order_acquire)) {
    uint32_t global_ver = s_job_version.load(std::memory_order_acquire);
    if (global_ver == 0) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    if (global_ver != cached_version) {
      {
        std::lock_guard<std::mutex> lock(s_job_mutex);
        local_job = s_active_job;
        cached_version = local_job.version;
      }

      // Precompute first 64-byte block state once per job
      mbedtls_sha256_starts(&base_ctx, 0);
      mbedtls_sha256_update(&base_ctx, local_job.data.bytearray_blockheader, 64);
      memcpy(tail_and_nonce, local_job.data.bytearray_blockheader + 64, 12);
    }

    uint32_t start_nonce = s_next_nonce.fetch_add(SW_BATCH_SIZE, std::memory_order_relaxed);
    uint32_t end_nonce = start_nonce + SW_BATCH_SIZE;
    uint32_t batch_hashes = 0;

    for (uint32_t n = start_nonce; n != end_nonce; ++n) {
      if (s_job_version.load(std::memory_order_relaxed) != cached_version ||
          !s_mining_running.load(std::memory_order_relaxed)) {
        break;
      }

      memcpy(tail_and_nonce + 12, &n, sizeof(uint32_t));

      mbedtls_sha256_clone(&work_ctx, &base_ctx);
      mbedtls_sha256_update(&work_ctx, tail_and_nonce, 16);
      mbedtls_sha256_finish(&work_ctx, inter_hash);

      mbedtls_sha256_starts(&work_ctx, 0);
      mbedtls_sha256_update(&work_ctx, inter_hash, 32);
      mbedtls_sha256_finish(&work_ctx, hash);

      batch_hashes++;

      // Early 16-bit check on upper bytes of little-endian 256-bit hash
      if (hash[31] == 0 && hash[30] == 0) {
        memcpy(local_job.data.bytearray_blockheader + 76, &n, sizeof(uint32_t));
        record_candidate(local_job, hash);
      }

      if ((batch_hashes % SW_YIELD_INTERVAL) == 0) {
        s_hashes.fetch_add(SW_YIELD_INTERVAL, std::memory_order_relaxed);
        batch_hashes = 0;
        vTaskDelay(1);
      }
    }

    if (batch_hashes > 0) {
      s_hashes.fetch_add(batch_hashes, std::memory_order_relaxed);
    }
  }

  mbedtls_sha256_free(&work_ctx);
  mbedtls_sha256_free(&base_ctx);
  vTaskDelete(nullptr);
}

#if defined(CONFIG_IDF_TARGET_ESP32)

void minerWorkerHw(void *task_id) {
  ESP_LOGD(TAG, "Starting %s (Pipelined ASM) on core %d", static_cast<char *>(task_id), xPortGetCoreID());

  ActiveJob local_job{};
  uint32_t cached_version = 0;
  uint32_t header_swapped[20];
  uint8_t verify_hash[32];
  uint32_t yield_batches = 0;

  while (s_mining_running.load(std::memory_order_acquire)) {
    uint32_t global_ver = s_job_version.load(std::memory_order_acquire);
    if (global_ver == 0) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    if (global_ver != cached_version) {
      {
        std::lock_guard<std::mutex> lock(s_job_mutex);
        local_job = s_active_job;
        cached_version = local_job.version;
      }

      const uint32_t *header_words = reinterpret_cast<const uint32_t *>(local_job.data.bytearray_blockheader);
      for (int i = 0; i < 20; i++) {
        header_swapped[i] = __builtin_bswap32(header_words[i]);
      }
    }

    uint32_t start_nonce = s_next_nonce.fetch_add(HW_BATCH_SIZE, std::memory_order_relaxed);
    uint32_t nonce_swapped = __builtin_bswap32(start_nonce);
    uint32_t remaining = HW_BATCH_SIZE;

    while (remaining > 0 && s_job_version.load(std::memory_order_relaxed) == cached_version &&
           s_mining_running.load(std::memory_order_relaxed)) {
      uint32_t done = 0;
      bool candidate =
          sha256_pipelined_mine(header_swapped, &nonce_swapped, remaining, &done, verify_hash, &s_hw_mining_active);

      s_hashes.fetch_add(done, std::memory_order_relaxed);
      remaining = (done < remaining) ? (remaining - done) : 0;

      if (candidate) {
        uint32_t candidate_nonce = __builtin_bswap32(nonce_swapped - 1U);
        memcpy(local_job.data.bytearray_blockheader + 76, &candidate_nonce, sizeof(uint32_t));
        record_candidate(local_job, verify_hash);
      }

      // Feed standard ESPHome IDLE WDT periodically without reconfiguring watchdog
      if (++yield_batches >= HW_YIELD_BATCHES) {
        yield_batches = 0;
        vTaskDelay(1);
      }
    }
  }

  vTaskDelete(nullptr);
}

#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)

void minerWorkerHw(void *task_id) {
  ESP_LOGD(TAG, "Starting %s (HW SHA) on core %d", static_cast<char *>(task_id), xPortGetCoreID());

  ActiveJob local_job{};
  uint32_t cached_version = 0;
  uint32_t midstate[8];
  uint8_t sha_buffer[64];
  uint8_t hash[32];

  memset(sha_buffer, 0, sizeof(sha_buffer));
  sha_buffer[16] = 0x80;
  sha_buffer[62] = 0x02;
  sha_buffer[63] = 0x80;

  while (s_mining_running.load(std::memory_order_acquire)) {
    uint32_t global_ver = s_job_version.load(std::memory_order_acquire);
    if (global_ver == 0) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    if (global_ver != cached_version) {
      std::lock_guard<std::mutex> lock(s_job_mutex);
      local_job = s_active_job;
      cached_version = local_job.version;
      nerd_mids(midstate, local_job.data.bytearray_blockheader);
      memcpy(sha_buffer, local_job.data.bytearray_blockheader + 64, 12);
    }

    uint32_t start_nonce = s_next_nonce.fetch_add(HW_BATCH_SIZE, std::memory_order_relaxed);
    uint32_t end_nonce = start_nonce + HW_BATCH_SIZE;
    uint32_t batch_hashes = 0;

    esp_sha_acquire_hardware();
    REG_WRITE(SHA_MODE_REG, SHA2_256);

    for (uint32_t n = start_nonce; n != end_nonce; ++n) {
      if (s_job_version.load(std::memory_order_relaxed) != cached_version ||
          !s_mining_running.load(std::memory_order_relaxed)) {
        break;
      }

      memcpy(sha_buffer + 12, &n, sizeof(uint32_t));

      nerd_sha_ll_write_digest(midstate);
      sha_ll_fill_text_block(sha_buffer, 16);
      sha_ll_continue_block(SHA2_256);
      nerd_sha_hal_wait_idle();
      sha_ll_load(SHA2_256);
      nerd_sha_hal_wait_idle();

      REG_WRITE(SHA_TEXT_BASE + 8 * 4, 0x00000080);
      REG_WRITE(SHA_TEXT_BASE + 15 * 4, 0x00010000);
      for (int i = 9; i < 15; i++) {
        REG_WRITE(SHA_TEXT_BASE + i * 4, 0);
      }

      sha_ll_start_block(SHA2_256);
      nerd_sha_hal_wait_idle();
      sha_ll_load(SHA2_256);
      nerd_sha_hal_wait_idle();

      batch_hashes++;

      if (static_cast<uint16_t>(REG_READ(SHA_H_BASE + 7 * 4) >> 16) == 0) {
        sha_ll_read_digest(SHA2_256, hash, 32);
        memcpy(local_job.data.bytearray_blockheader + 76, &n, sizeof(uint32_t));
        record_candidate(local_job, hash);
      }
    }

    esp_sha_release_hardware();
    s_hashes.fetch_add(batch_hashes, std::memory_order_relaxed);
    vTaskDelay(1);
  }

  vTaskDelete(nullptr);
}

#else

void minerWorkerHw(void *task_id) { minerWorkerSw(task_id); }

#endif

}  // namespace esphome::nerdminer
