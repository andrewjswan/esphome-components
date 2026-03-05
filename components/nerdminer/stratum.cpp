/*
 * SparkMiner - Stratum Protocol Implementation
 * Stratum v1 client for pool communication
 *
 * Based on BitsyMiner by Justin Williams (GPL v3)
 */

#include <ArduinoJson.h>
#include <utility>  // For std::swap
#include <lwip/netdb.h>

#include "nerdminer.h"
#include "config.h"
#include "stratum.h"
#include "miner.h"
#include "utils.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/hal.h"

// ============================================================
// Constants
// ============================================================
#define STRATUM_MSG_BUFFER 512
#define RESPONSE_TIMEOUT_MS 3000
#define KEEPALIVE_MS 120000
#define INACTIVITY_MS 700000

namespace esphome {
namespace nerdminer {

// ============================================================
// Global State
// ============================================================
static QueueHandle_t s_submitQueue = NULL;
static submit_entry_t s_pendingResponses[MAX_PENDING_SUBMISSIONS];
static uint16_t s_pendingIndex = 0;

static pool_config_t s_primaryPool;

static volatile bool s_isConnected = false;
static volatile bool s_reconnectRequested = false;
static char s_currentPoolUrl[MAX_POOL_URL_LEN] = {0};

// Stores the fully authorized username (e.g. "wallet.worker") for use in submissions
static char s_authorizedWorkerName[MAX_WALLET_LEN + 34] = {0};

static uint32_t s_messageId = 1;
static uint32_t s_lastActivity = 0;
static uint32_t s_lastSubmit = 0;

// WiFi reconnection state (Issue #4 fix)
static uint32_t s_wifiReconnectAttempts = 0;
static uint32_t s_lastWifiReconnectAttempt = 0;

// Extra nonce from subscription
static char s_extraNonce1[32] = {0};
static int s_extraNonce2Size = 4;

// JSON document for parsing
static JsonDocument s_doc;

// ============================================================
// Socket Functions
// ============================================================

bool sockConnected(esphome::socket::Socket *sock) {
  if (sock == nullptr || sock->get_fd() < 0)
    return false;
  uint8_t dummy;

  ssize_t res = lwip_recv(sock->get_fd(), &dummy, 1, MSG_PEEK | MSG_DONTWAIT);
  if (res == 0)
    return false;
  if (res < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    return false;
  return true;
}

bool sockAvailable(esphome::socket::Socket *sock) {
  if (sock == nullptr || sock->get_fd() < 0) {
    return false;
  }

  char dummy;
  int res = lwip_recv(sock->get_fd(), &dummy, 1, MSG_PEEK | MSG_DONTWAIT);
  return res > 0;
}

void sockClose(std::unique_ptr<esphome::socket::Socket> &sock) {
  if (sock != nullptr) {
    sock->close();
    sock.reset();
    errno = 0;
  }
}

bool checkPoolConnection(std::unique_ptr<esphome::socket::Socket> &sock) {
  if (sock != nullptr) {
    if (sockConnected(sock.get())) {
      return true;
    } else {
      ESP_LOGD(TAG, "Connection lost, cleaning up...");
      sockClose(sock);
    }
  }

  ESP_LOGD(TAG, "Client not connected, trying to connect...");
  ESP_LOGD(TAG, "Connecting to %s:%d...", s_primaryPool.url, s_primaryPool.port);

  struct addrinfo hints;
  struct addrinfo *res = nullptr;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  int err = getaddrinfo(s_primaryPool.url, NULL, &hints, &res);
  if (err != 0 || res == nullptr) {
    ESP_LOGW(TAG, "DNS Resolution failed for %s, error: %d", s_primaryPool.url, err);
    if (res != nullptr) {
      freeaddrinfo(res);
    }
    return false;
  }

  // Try connecting pool IP
  sock = esphome::socket::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == nullptr) {
    ESP_LOGW(TAG, "Failed to create socket (errno: %d)", errno);
    freeaddrinfo(res);
    return false;
  }

  sock->setblocking(false);

  struct sockaddr_in s_addr{};
  s_addr.sin_family = AF_INET;
  s_addr.sin_port = htons(s_primaryPool.port);
  s_addr.sin_addr.s_addr = ((struct sockaddr_in *) res->ai_addr)->sin_addr.s_addr;

  freeaddrinfo(res);

  int conn_res = sock->connect((struct sockaddr *) &s_addr, sizeof(s_addr));
  if (conn_res == 0) {
    ESP_LOGD(TAG, "Successfully connected to %s!", s_primaryPool.url);
    s_lastActivity = millis();
    return true;
  }

  if (conn_res < 0 && errno != EINPROGRESS) {
    ESP_LOGW(TAG, "Connection to %s Failed immediately, errno: %d", s_primaryPool.url, errno);
    sockClose(sock);
    return false;
  }

  struct pollfd pfd;
  pfd.fd = sock->get_fd();
  pfd.events = POLLOUT;
  int ready = poll(&pfd, 1, 1500);
  if (ready > 0) {
    int so_error;
    socklen_t len = sizeof(so_error);
    getsockopt(pfd.fd, SOL_SOCKET, SO_ERROR, &so_error, &len);

    if (so_error == 0) {
      ESP_LOGD(TAG, "Successfully connected via Poll to %s!", s_primaryPool.url);
      s_lastActivity = millis();
      return true;
    } else {
      ESP_LOGW(TAG, "Connection to %s Error after Poll: %d", s_primaryPool.url, so_error);
    }
  } else if (ready == 0) {
    ESP_LOGW(TAG, "Connection to %s Connection timeout...", s_primaryPool.url);
  } else {
    ESP_LOGW(TAG, "Connection to %s Poll system error: %d", s_primaryPool.url, errno);
  }

  sockClose(sock);
  return false;
}

// ============================================================
// Utility Functions
// ============================================================

static uint32_t getNextId() {
  if (s_messageId == UINT32_MAX) {
    s_messageId = 1;
  }
  return s_messageId++;
}

// Safe string copy with null termination
static void safeStrCpy(char *dest, const char *src, size_t maxLen) {
  strncpy(dest, src, maxLen - 1);
  dest[maxLen - 1] = '\0';
}

// Format hex string with zero padding (big-endian - value as hex)
static void formatHex8(char *dest, uint32_t value) {
  static const char *hex = "0123456789abcdef";
  for (int i = 7; i >= 0; i--) {
    dest[i] = hex[value & 0xF];
    value >>= 4;
  }
  dest[8] = '\0';
}

// Bounded read to prevent stack overflow/OOM from malicious packets
static std::string readBoundedLine(esphome::socket::Socket *sock, char terminator, size_t maxLen = 4096) {
  std::string line;
  if (sock == nullptr || sock->get_fd() < 0) {
    return line;
  }

  int fd = sock->get_fd();
  uint32_t start = millis();
  while (millis() - start < 1500) {
    char c;
    int res = lwip_read(fd, &c, 1);

    if (res > 0) {
      if (c == terminator) {
        return line;
      }
      if (line.length() < maxLen) {
        line += c;
      }
    } else if (res < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        vTaskDelay(1);
        continue;
      }
      break;
    } else {
      break;
    }
  }

  if (line.length() >= maxLen) {
    ESP_LOGW(TAG, "[STRATUM] WARNING: Line exceeded max length, discarded.");
    return "";  // Return empty to signal error
  }
  return line;
}

ssize_t sendData(esphome::socket::Socket *sock, const std::string &data) {
  if (sock == nullptr || sock->get_fd() < 0)
    return -1;

  int fd = sock->get_fd();
  const char *ptr = data.c_str();
  size_t len = data.size();
  size_t total_sent = 0;
  uint32_t start_time = millis();

  while (total_sent < len) {
    int sent = lwip_write(fd, ptr + total_sent, len - total_sent);

    if (sent > 0) {
      total_sent += sent;
      start_time = millis();
    } else if (sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        vTaskDelay(1);
        if (millis() - start_time > 1500) {
          ESP_LOGW(TAG, "Send: Timeout (Socket buffer full)");
          return -1;
        }
        continue;
      }
      ESP_LOGW(TAG, "Send: Error: %d", errno);
      return -1;
    } else {
      return -1;
    }
  }
  return (ssize_t) total_sent;
}

// ============================================================
// Protocol Functions
// ============================================================

static bool sendMessage(esphome::socket::Socket *sock, const char *msg) {
  if (!sockConnected(sock))
    return false;

  // Send message with newline as single write (like NerdMiner)
  // This avoids TCP packet fragmentation issues
  sendData(sock, std::string(msg) + "\n");

  ESP_LOGD(TAG, "[STRATUM] TX: %s", msg);
  return true;
}

static bool waitForResponse(esphome::socket::Socket *sock, int timeoutMs) {
  int elapsed = 0;
  while (!sockAvailable(sock) && elapsed < timeoutMs) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    elapsed += 10;
  }
  return sockAvailable(sock);
}

static bool parseSubscribeResponse(const std::string &line) {
  s_doc.clear();
  DeserializationError err = deserializeJson(s_doc, line);

  if (err) {
    ESP_LOGW(TAG, "[STRATUM] JSON parse error: %s RAW: %s", err.c_str(), line.c_str());
    return false;
  }

  if (s_doc["error"].is<JsonVariant>() && s_doc["error"].size() > 0) {
    const char *errMsg = s_doc["error"][1];
    ESP_LOGW(TAG, "[STRATUM] Subscribe error: %s", errMsg ? errMsg : "unknown");
    return false;
  }

  if (!s_doc["result"].is<JsonVariant>() || !s_doc["result"].is<JsonArray>()) {
    ESP_LOGW(TAG, "[STRATUM] Invalid subscribe response (no result)");
    return false;
  }

  // Extract extra nonce
  const char *en1 = s_doc["result"][1];
  if (en1) {
    safeStrCpy(s_extraNonce1, en1, sizeof(s_extraNonce1));
  }

  s_extraNonce2Size = s_doc["result"][2] | 4;

  // Pass to miner
  miner_set_extranonce(s_extraNonce1, s_extraNonce2Size);

  ESP_LOGD(TAG, "[STRATUM] Subscribed: extraNonce1=%s, extraNonce2Size=%d", s_extraNonce1, s_extraNonce2Size);

  return true;
}

static bool parseAuthorizeResponse(const std::string &line) {
  s_doc.clear();
  DeserializationError err = deserializeJson(s_doc, line);

  if (err)
    return false;

  if (s_doc["error"].is<JsonVariant>() && s_doc["error"].size() > 0) {
    const char *errMsg = s_doc["error"][1];
    ESP_LOGW(TAG, "[STRATUM] Auth error: %s", errMsg ? errMsg : "unknown");
    return false;
  }

  bool result = s_doc["result"] | false;
  return result;
}

static void parseMiningNotify(const std::string &line) {
  if (!s_doc["params"].is<JsonVariant>())
    return;

  JsonArray params = s_doc["params"];

  // Use static job to avoid stack allocation of large struct each time
  static stratum_job_t job;
  memset(&job, 0, sizeof(job));

  // Copy strings to fixed char arrays (no heap allocation!)
  const char *p0 = params[0];
  const char *p1 = params[1];
  const char *p2 = params[2];
  const char *p3 = params[3];
  const char *p5 = params[5];
  const char *p6 = params[6];
  const char *p7 = params[7];

  if (p0)
    strncpy(job.jobId, p0, STRATUM_JOB_ID_LEN - 1);
  if (p1)
    strncpy(job.prevHash, p1, STRATUM_PREVHASH_LEN - 1);
  if (p2)
    strncpy(job.coinBase1, p2, STRATUM_COINBASE1_LEN - 1);
  if (p3)
    strncpy(job.coinBase2, p3, STRATUM_COINBASE2_LEN - 1);
  if (p5)
    strncpy(job.version, p5, STRATUM_FIELD_LEN - 1);
  if (p6)
    strncpy(job.nbits, p6, STRATUM_FIELD_LEN - 1);
  if (p7)
    strncpy(job.ntime, p7, STRATUM_FIELD_LEN - 1);

  // Copy merkle branches to fixed array
  JsonArray merkle = params[4];
  job.merkleBranchCount = 0;
  for (size_t i = 0; i < merkle.size() && i < STRATUM_MAX_MERKLE; i++) {
    const char *branch = merkle[i];
    if (branch) {
      strncpy(job.merkleBranches[i], branch, 67);
      job.merkleBranches[i][67] = '\0';
      job.merkleBranchCount++;
    }
  }

  job.cleanJobs = params[8] | false;
  strncpy(job.extraNonce1, s_extraNonce1, STRATUM_EXTRANONCE_LEN - 1);
  job.extraNonce2Size = s_extraNonce2Size;

  s_lastActivity = millis();
  miner_start_job(&job);
}

static void parseSetDifficulty(const std::string &line) {
  if (!s_doc["params"].is<JsonVariant>())
    return;

  double diff = s_doc["params"][0] | 1.0;

  if (!std::isnan(diff) && diff > 0) {
    miner_set_difficulty(diff);
    ESP_LOGD(TAG, "[STRATUM] Pool difficulty: %.4f", diff);
  }
}

static void handleServerMessage(esphome::socket::Socket *sock) {
  std::string line = readBoundedLine(sock, '\n');
  if (line.length() == 0)
    return;

  ESP_LOGD(TAG, "[STRATUM] RX: %s", line.c_str());
  s_doc.clear();
  DeserializationError err = deserializeJson(s_doc, line);
  if (err) {
    ESP_LOGW(TAG, "[STRATUM] Parse error: %s", err.c_str());
    return;
  }

  // Check for submission responses
  if (s_doc["id"].is<JsonVariant>() && s_doc["result"].is<JsonVariant>()) {
    uint32_t msgId = s_doc["id"];
    bool accepted = s_doc["result"] | false;

    // Find matching pending submission
    for (int i = 0; i < MAX_PENDING_SUBMISSIONS; i++) {
      if (s_pendingResponses[i].msgId == msgId) {
        mining_stats_t *stats = miner_get_stats();

        uint32_t latency = millis() - s_pendingResponses[i].sentTime;
        stats->lastLatency = latency;
        stats->avgLatency = (stats->avgLatency == 0) ? latency : ((stats->avgLatency * 9 + latency) / 10);

        if (accepted) {
          stats->accepted++;
          ESP_LOGD(TAG, "[STRATUM] Share accepted!");
        } else {
          stats->rejected++;
          const char *reason = s_doc["error"][1] | "unknown";
          ESP_LOGD(TAG, "[STRATUM] Share rejected: %s", reason);
          ESP_LOGD(TAG, "[STRATUM] Share rejected: %s", reason);
        }

        // Call callback if set
        if (s_pendingResponses[i].callback) {
          const char *reason = accepted ? NULL : (const char *) s_doc["error"][1];
          s_pendingResponses[i].callback(s_pendingResponses[i].sessionId, s_pendingResponses[i].msgId, accepted,
                                         reason);
        }

        s_pendingResponses[i].msgId = 0;  // Clear slot
        break;
      }
    }
  }

  // Check for method calls
  if (s_doc["method"].is<JsonVariant>()) {
    const char *method = s_doc["method"];

    if (strcmp(method, "mining.notify") == 0) {
      parseMiningNotify(line);
    } else if (strcmp(method, "mining.set_difficulty") == 0) {
      parseSetDifficulty(line);
    } else {
      ESP_LOGD(TAG, "[STRATUM] Unknown method: %s", method);
    }
  }
}

// Helper: Read lines until we get a response with matching ID (or timeout)
// Handles method calls (set_difficulty, notify) that arrive before the response
static bool waitForResponseById(esphome::socket::Socket *sock, uint32_t expectedId, std::string &outResponse,
                                int maxAttempts = 10) {
  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    std::string line = readBoundedLine(sock, '\n');  // Use bounded read to prevent OOM
    if (line.length() == 0) {
      ESP_LOGD(TAG, "[STRATUM] Empty response for id!");
      return false;
    }

    // Parse to check if this is our response or a method call
    s_doc.clear();
    DeserializationError err = deserializeJson(s_doc, line);
    if (err) {
      ESP_LOGW(TAG, "[STRATUM] JSON parse error: %s", err.c_str());
      continue;
    }

    // Check if this is a method call (id is null or missing, has "method" field)
    if (s_doc["method"].is<JsonVariant>()) {
      const char *method = s_doc["method"];

      // Handle set_difficulty immediately since it's important
      if (strcmp(method, "mining.set_difficulty") == 0) {
        double diff = s_doc["params"][0] | 1.0;
        if (!std::isnan(diff) && diff > 0) {
          miner_set_difficulty(diff);
        }
      }
      // Continue reading for our actual response
      continue;
    }

    // Check if this response matches our expected ID
    if (s_doc["id"].is<JsonVariant>()) {
      uint32_t respId = s_doc["id"] | 0;
      if (respId == expectedId) {
        outResponse = line;
        return true;
      }
      ESP_LOGD(TAG, "[STRATUM] Got response for different id: %lu (expected %lu)", respId, expectedId);
    }
  }

  ESP_LOGD(TAG, "[STRATUM] Max attempts reached waiting for response");
  return false;
}

static bool subscribe(esphome::socket::Socket *sock, const char *wallet, const char *password, const char *workerName) {
  char msg[STRATUM_MSG_BUFFER];

  // Set client timeout for blocking reads
  // client.setTimeout(5000);

  // Mining.subscribe
  uint32_t subId = getNextId();
  snprintf(msg, sizeof(msg), "{\"id\":%lu,\"method\":\"mining.subscribe\",\"params\":[\"%s/%s\"]}", subId, MINER_NAME,
           NERDMINER_VERSION);

  uint32_t startSub = millis();
  if (!sendMessage(sock, msg))
    return false;

  // Small delay to allow server to process (like NerdMiner)
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // Wait for subscribe response (handle any method calls that arrive first)
  std::string resp;
  if (!waitForResponseById(sock, subId, resp)) {
    ESP_LOGD(TAG, "[STRATUM] No subscribe response");
    return false;
  }

  // Record subscribe latency
  uint32_t subLatency = millis() - startSub;
  mining_stats_t *stats = miner_get_stats();
  stats->lastLatency = subLatency;
  stats->avgLatency = (stats->avgLatency == 0) ? subLatency : ((stats->avgLatency * 9 + subLatency) / 10);

  if (!parseSubscribeResponse(resp)) {
    ESP_LOGW(TAG, "[STRATUM] Subscribe failed");
    return false;
  }

  // Suggest difficulty
  uint32_t diffId = getNextId();
  snprintf(msg, sizeof(msg), "{\"id\":%lu,\"method\":\"mining.suggest_difficulty\",\"params\":[%.10g]}", diffId,
           DESIRED_DIFFICULTY);
  sendMessage(sock, msg);

  // Mining.authorize - append worker name if set
  char fullUsername[MAX_WALLET_LEN + 34];
  if (workerName && workerName[0]) {
    snprintf(fullUsername, sizeof(fullUsername), "%s.%s", wallet, workerName);
  } else {
    safeStrCpy(fullUsername, wallet, sizeof(fullUsername));
  }

  // Store authorized worker name for submissions
  safeStrCpy(s_authorizedWorkerName, fullUsername, sizeof(s_authorizedWorkerName));

  uint32_t authId = getNextId();
  snprintf(msg, sizeof(msg), "{\"id\":%lu,\"method\":\"mining.authorize\",\"params\":[\"%s\",\"%s\"]}", authId,
           fullUsername, password);

  uint32_t startAuth = millis();
  if (!sendMessage(sock, msg))
    return false;

  // Small delay before reading
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // Wait for authorize response (handle set_difficulty/notify that may arrive first)
  if (!waitForResponseById(sock, authId, resp)) {
    ESP_LOGW(TAG, "[STRATUM] No authorize response");
    return false;
  }

  // Record authorize latency
  uint32_t authLatency = millis() - startAuth;
  stats->lastLatency = authLatency;
  stats->avgLatency = (stats->avgLatency * 9 + authLatency) / 10;

  if (!parseAuthorizeResponse(resp)) {
    ESP_LOGW(TAG, "[STRATUM] Authorization failed");
    return false;
  }

  ESP_LOGD(TAG, "[STRATUM] Authorized as %s", fullUsername);
  return true;
}

static void submitShare(esphome::socket::Socket *sock, const submit_entry_t *entry) {
  char msg[STRATUM_MSG_BUFFER];
  char timestamp[9], nonce[9];

  // Format as 8-char hex (value as hex, zero-padded)
  formatHex8(timestamp, entry->timestamp);
  formatHex8(nonce, entry->nonce);

  uint32_t msgId = getNextId();

  // Standard Stratum v1 submit (5 params, no version rolling)
  snprintf(msg, sizeof(msg),
           "{\"id\":%lu,\"method\":\"mining.submit\",\"params\":[\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]}", msgId,
           s_authorizedWorkerName,  // Use the full worker name used during authorization
           entry->jobId, entry->extraNonce2, timestamp, nonce);

  ESP_LOGD(TAG, "[STRATUM] Submit: job=%s en2=%s time=%s nonce=%s", entry->jobId, entry->extraNonce2, timestamp, nonce);

  if (sendMessage(sock, msg)) {
    // Store in pending responses for latency tracking
    submit_entry_t pending = *entry;
    pending.msgId = msgId;
    pending.sentTime = millis();

    s_pendingResponses[s_pendingIndex] = pending;
    s_pendingIndex = (s_pendingIndex + 1) % MAX_PENDING_SUBMISSIONS;

    s_lastSubmit = millis();
    miner_get_stats()->shares++;
  }
}

// ============================================================
// Public API
// ============================================================

void stratum_init() {
  // Create submission queue
  s_submitQueue = xQueueCreate(MAX_PENDING_SUBMISSIONS, sizeof(submit_entry_t));

  // Initialize pending responses
  memset(s_pendingResponses, 0, sizeof(s_pendingResponses));

  // Set default pool
  // safeStrCpy(s_primaryPool.url, DEFAULT_POOL_URL, MAX_POOL_URL_LEN);
  // s_primaryPool.port = DEFAULT_POOL_PORT;
  // safeStrCpy(s_primaryPool.password, DEFAULT_POOL_PASS, MAX_PASSWORD_LEN);

  ESP_LOGD(TAG, "[STRATUM] Initialized");
}

void stratum_task(void *name) {
  std::unique_ptr<esphome::socket::Socket> pool_socket;
  uint32_t lastConnectAttempt = 0;

  ESP_LOGCONFIG(TAG, "[STRATUM] Started. Running %s on core %d", (char *) name, xPortGetCoreID());

  while (true) {
    // Wait for WiFi with auto-reconnect (Issue #4 fix)
    if (!network::is_connected()) {
      if (s_isConnected) {
        miner_stop();
        sockClose(pool_socket);
        s_isConnected = false;
        ESP_LOGW(TAG, "[STRATUM] Connection lost, wait...");
      }

      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }

    // Check pool configuration
    if (!s_primaryPool.url[0] || !s_primaryPool.port) {
      ESP_LOGD(TAG, "[STRATUM] No pool configured");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!s_primaryPool.wallet[0]) {
      ESP_LOGD(TAG, "[STRATUM] No wallet configured");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    // Handle reconnect request
    if (s_reconnectRequested) {
      miner_stop();
      sockClose(pool_socket);
      s_isConnected = false;
      s_reconnectRequested = false;
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    // Connect if needed
    if (!checkPoolConnection(pool_socket)) {
      if (s_isConnected) {
        miner_stop();
        sockClose(pool_socket);
        s_isConnected = false;
      }
      vTaskDelay(((1 + rand() % 60) * 1000) / portTICK_PERIOD_MS);
      continue;
    }
    // Subscribe
    if (!s_isConnected) {
      ESP_LOGD(TAG, "[STRATUM] Subscribe...");
      if (subscribe(pool_socket.get(), s_primaryPool.wallet, s_primaryPool.password, s_primaryPool.workerName)) {
        s_lastActivity = millis();
        ESP_LOGD(TAG, "[STRATUM] Subscribe complete.");
      } else {
        sockClose(pool_socket);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        continue;
      }
    }
    s_isConnected = true;

    // Handle incoming messages
    while (pool_socket != nullptr && sockAvailable(pool_socket.get())) {
      handleServerMessage(pool_socket.get());
    }

    // Process submission queue
    submit_entry_t entry;
    while (xQueueReceive(s_submitQueue, &entry, 0) == pdTRUE) {
      submitShare(pool_socket.get(), &entry);
    }

    // Send keepalive if idle
    if (millis() - s_lastSubmit > KEEPALIVE_MS) {
      char msg[STRATUM_MSG_BUFFER];
      uint32_t keepId = getNextId();
      snprintf(msg, sizeof(msg), "{\"id\":%lu,\"method\":\"mining.suggest_difficulty\",\"params\":[%.10g]}", keepId,
               DESIRED_DIFFICULTY);
      sendMessage(pool_socket.get(), msg);
      s_lastSubmit = millis();
    }

    // Check for inactivity
    if (millis() - s_lastActivity > INACTIVITY_MS) {
      ESP_LOGD(TAG, "[STRATUM] Pool inactive, disconnecting");
      miner_stop();
      sockClose(pool_socket);
      s_isConnected = false;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

bool stratum_submit_share(const submit_entry_t *entry) {
  if (!s_submitQueue)
    return false;
  return xQueueSend(s_submitQueue, entry, pdMS_TO_TICKS(100)) == pdTRUE;
}

void stratum_reconnect() { s_reconnectRequested = true; }

bool stratum_is_connected() { return s_isConnected; }

const char *stratum_get_pool() { return s_currentPoolUrl; }

void stratum_set_pool(const char *url, int port, const char *wallet, const char *password, const char *workerName) {
  safeStrCpy(s_primaryPool.url, url, MAX_POOL_URL_LEN);
  s_primaryPool.port = port;
  safeStrCpy(s_primaryPool.wallet, wallet, MAX_WALLET_LEN);
  safeStrCpy(s_primaryPool.password, password, MAX_PASSWORD_LEN);
  if (workerName) {
    safeStrCpy(s_primaryPool.workerName, workerName, 32);
  } else {
    s_primaryPool.workerName[0] = '\0';
  }
}

}  // namespace nerdminer
}  // namespace esphome
