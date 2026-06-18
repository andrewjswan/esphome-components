#include "duco.h"
#include "mining.h"

#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/json/json_util.h"

#include <algorithm>
#include <cctype>
#include <esp_task_wdt.h>
#include <soc/soc_caps.h>

#if defined(USE_ESP32)
#include <lwip/netdb.h>
#elif defined(USE_ESP8266)
#include <netdb.h>
#endif

namespace esphome::duco {

#define CHECK_INTERVAL 60000
// 15 minutes WDT for miner task
#define WDT_MINER_TIMEOUT 900000

void Duco::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Duco...");

#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif

  this->configuration = new MiningConfig(DUCO_USERNAME, DUCO_WORKER, DUCO_KEY);

  this->configuration->WALLET_ID = random_uint32() % 2811;  // Needed for miner grouping in the wallet

  this->generate_identifier();

  this->start();
}  // setup()

#ifdef USE_OTA_STATE_LISTENER
void Duco::on_ota_global_state(ota::OTAState state, float progress, uint8_t error, ota::OTAComponent *comp) {
  if (state == ota::OTA_STARTED) {
    this->stop();
  }
}
#endif

void Duco::loop() {
  if (!network::is_connected()) {
    this->configuration->is_ready = false;
    return;
  }
    
  uint32_t current_time = millis();
  if (current_time - this->last_check_time >= CHECK_INTERVAL) {
    this->last_check_time = current_time;
    check_for_problem();
  }

  if (this->configuration->is_ready)
    return;

  if (this->last_fetch_time != 0 && (current_time - this->last_fetch_time < CHECK_INTERVAL)) {
    return;
  }
  this->last_fetch_time = current_time;
  this->fetch_pool_node();
}

void Duco::start() {
//   esp_task_wdt_config_t wdt_config = {
//       .timeout_ms = WDT_MINER_TIMEOUT,
//       .idle_core_mask = (1 << SOC_CPU_CORES_NUM) - 1,  // Bitmask of all cores
//       .trigger_panic = true,
//   };
//   esp_task_wdt_init(&wdt_config);
//   esp_task_wdt_reconfigure(&wdt_config);

  this->job[0] = new MiningJob(0, this->configuration, this);
  xTaskCreatePinnedToCore(Duco::duco_thread_entry, "Miner/0", 10000, (void *) this->job[0], 1, &this->miner1_handle, 0);
//  esp_task_wdt_add(this->miner1_handle);

#if (SOC_CPU_CORES_NUM >= 2)
  this->job[1] = new MiningJob(1, this->configuration, this);
  xTaskCreatePinnedToCore(Duco::duco_thread_entry, "Miner/1", 10000, (void *) this->job[1], 1, &this->miner2_handle, 1);
//  esp_task_wdt_add(this->miner2_handle);
#endif

  ESP_LOGCONFIG(TAG, "Duco started...");
}  // start()

void Duco::stop() {
  if (this->configuration != nullptr) {
    this->configuration->is_ready = false;
  }

#if defined(ESP32)
  if (this->miner1_handle != nullptr) {
//    esp_task_wdt_delete(this->miner1_handle);
    vTaskDelete(this->miner1_handle);
    this->miner1_handle = nullptr;
  }
#if (SOC_CPU_CORES_NUM >= 2)
  if (this->miner2_handle != nullptr) {
//    esp_task_wdt_delete(this->miner2_handle);
    vTaskDelete(this->miner2_handle);
    this->miner2_handle = nullptr;
  }
#endif
#endif

  vTaskDelay(pdMS_TO_TICKS(50));

  ESP_LOGCONFIG(TAG, "Duco stopped.");
}  // stop()

void Duco::dump_config() {
  ESP_LOGCONFIG(TAG, "Duco version: %s", DUCO_VERSION);
  ESP_LOGCONFIG(TAG, "      Worker: %s", this->configuration->RIG_IDENTIFIER.c_str());
  ESP_LOGCONFIG(TAG, "       Cores: %d", SOC_CPU_CORES_NUM);
}  // dump_config()

void Duco::duco_thread_entry(void *params) {
  if (params == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  MiningJob *current_job = static_cast<MiningJob *>(params);
  ESP_LOGCONFIG(TAG, "[MINER] %d Started...", xPortGetCoreID());

  for (;;) {
    current_job->mine();
  }  // for(;;)

  vTaskDelete(NULL);
}  // duco_function()

bool Duco::fetch_pool_node() {
  ESP_LOGI(TAG, "Fetching active node from Poolpicker...");
  this->configuration->is_ready = false;

  struct addrinfo hints;
  struct addrinfo *res = nullptr;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  const char *poolpicker_url = "server.duinocoin.com";
  int err = getaddrinfo(poolpicker_url, nullptr, &hints, &res);
  if (err != 0 || res == nullptr) {
    ESP_LOGW(TAG, "DNS Resolution failed for %s, error: %d", poolpicker_url, err);
    if (res != nullptr) {
      freeaddrinfo(res);
    }
    this->configuration->is_ready = false;
    this->status_set_warning();
    return false;
  }

  std::unique_ptr<esphome::socket::Socket> sock = esphome::socket::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock == nullptr) {
    ESP_LOGW(TAG, "Failed to create socket (errno: %d)", errno);
    freeaddrinfo(res);
    this->configuration->is_ready = false;
    this->status_set_warning();
    return false;
  }

  sock->setblocking(false);

  struct sockaddr_in s_addr{};
  s_addr.sin_family = AF_INET;
  s_addr.sin_port = htons(80);
  s_addr.sin_addr.s_addr = ((struct sockaddr_in *) res->ai_addr)->sin_addr.s_addr;

  freeaddrinfo(res);

  bool connected = false;
  int conn_res = sock->connect((struct sockaddr *) &s_addr, sizeof(s_addr));

  if (conn_res == 0) {
    ESP_LOGD(TAG, "Successfully connected to %s!", poolpicker_url);
    connected = true;
  } else if (conn_res < 0 && errno == EINPROGRESS) {
    struct pollfd pfd;
    pfd.fd = sock->get_fd();
    pfd.events = POLLOUT;

    int ready = poll(&pfd, 1, 1500);
    if (ready > 0) {
      int so_error;
      socklen_t len = sizeof(so_error);
      getsockopt(pfd.fd, SOL_SOCKET, SO_ERROR, &so_error, &len);

      if (so_error == 0) {
        ESP_LOGD(TAG, "Successfully connected via Poll to %s!", poolpicker_url);
        connected = true;
      } else {
        ESP_LOGW(TAG, "Connection to %s Error after Poll: %d", poolpicker_url, so_error);
      }
    } else if (ready == 0) {
      ESP_LOGW(TAG, "Connection to %s Connection timeout...", poolpicker_url);
    } else {
      ESP_LOGW(TAG, "Connection to %s Poll system error: %d", poolpicker_url, errno);
    }
  } else {
    ESP_LOGW(TAG, "Connection to %s Failed immediately, errno: %d", poolpicker_url, errno);
  }

  if (!connected) {
    sock.reset();
    this->configuration->is_ready = false;
    this->status_set_warning();
    return false;
  }

  std::string http_request = "GET /getPool HTTP/1.1\r\n"
                             "Host: server.duinocoin.com\r\n"
                             "User-Agent: esphome/" +
                             esphome::App.get_name() + " (" + DUCO_VERSION +
                             ")\r\n"
                             "Accept: application/json\r\n"
                             "Connection: close\r\n\r\n";

  if (sock->send(http_request.c_str(), http_request.length(), 0) < 0) {
    ESP_LOGW(TAG, "Failed to send HTTP request payload");
    sock.reset();
    this->configuration->is_ready = false;
    this->status_set_warning();
    return false;
  }

  sock->setblocking(true);

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 500000;
  sock->setsockopt(SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  std::string response;
  char recv_buffer[128];
  ssize_t bytes_received = 0;

  while ((bytes_received = sock->read(recv_buffer, sizeof(recv_buffer) - 1)) > 0) {
    response.append(recv_buffer, bytes_received);

    if (response.length() >= 12 && response.find("HTTP/") == 0) {
      if (response.find("HTTP/1.1 429") == 0) {
        ESP_LOGE(TAG, "Rate limit exceeded!");
        break;
      }
    }

    if (response.length() > 3072) {
      ESP_LOGE(TAG, "HTTP Response too large. Potential memory flood protected.");
      break;
    }
  }
  sock.reset();

  size_t headers_end = response.find("\r\n\r\n");
  if (headers_end == std::string::npos) {
    headers_end = response.find("\n\n");
  }
  if (headers_end == std::string::npos) {
    headers_end = 0;
  }

  size_t json_start = response.find('{', headers_end);
  if (json_start == std::string::npos) {
    ESP_LOGE(TAG, "Invalid HTTP response: Genuine JSON payload not found.");
    this->configuration->is_ready = false;
    this->status_set_warning();
    return false;
  }

  std::string json_body = response.substr(json_start);
  ESP_LOGD(TAG, "Extracted genuine JSON: %s", json_body.c_str());

  bool parse_success = esphome::json::parse_json(json_body, [this](JsonObject root) -> bool {
    if (root["ip"].is<JsonVariant>() && root["port"].is<JsonVariant>()) {
      std::string pool_ip = root["ip"].as<std::string>();
      uint16_t pool_port = root["port"].as<uint16_t>();
      std::string pool_name = root["name"].is<JsonVariant>() ? root["name"].as<std::string>() : "DucoNode";

      this->configuration->host = pool_ip;
      this->configuration->port = pool_port;
      this->configuration->node_id = pool_name;

      ESP_LOGI(TAG, "Poolpicker successfully parsed! Target Node: %s -> %s:%d", pool_name.c_str(), pool_ip.c_str(),
               pool_port);

      this->configuration->is_ready = true;
      this->status_clear_warning();
      return true;
    }
    return false;
  });

  if (!parse_success) {
    ESP_LOGE(TAG, "JSON fields validation failed. Schema mismatch.");
    this->status_set_warning();
  }

  return parse_success;
}

void Duco::generate_identifier() {
  std::string auto_rig_name = "";

  this->configuration->chip_id = esphome::str_upper_case(esphome::get_mac_address());
  if (this->configuration->RIG_IDENTIFIER != "Auto")
    return;

#if defined(ESP8266)
  this->configuration->RIG_IDENTIFIER = "ESP8266-" + this->configuration->chip_id;
#else
  this->configuration->RIG_IDENTIFIER = "ESP32-" + this->configuration->chip_id;
#endif

  this->configuration->RIG_IDENTIFIER = auto_rig_name;
}

void Duco::on_share_found_callback() {
  this->defer([this]() {
    ESP_LOGD(TAG, "Share found event caught in the main ESPHome loop!");
    this->share_found_callback.call();
  });
}

void Duco::check_for_problem() {
  bool should_restart = false;

  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      if (this->job[i]->problem()) {
        ESP_LOGW(TAG, "Miner on Core[%d] has a problem!", i);
        should_restart = true;
      }
    }
  }

  if (should_restart) {
    esphome::delay(1000);
    esphome::App.safe_reboot();
  }
}

std::string Duco::getCoresStatus() {
  std::string status = "";
  status.reserve(SOC_CPU_CORES_NUM); 

  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] == nullptr) {
      status += "-";
    } else if (this->job[i]->problem()) {
      status += "X";
    } else {
      status += "*";
    }
  }
  return status;
}

bool Duco::getMinerState() { return this->configuration->is_ready.load(); }

std::string Duco::getPool() { return this->configuration->host; }

uint32_t Duco::getHashRate() {
  uint32_t total_hashrate = 0;
  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      total_hashrate += this->job[i]->hashrate.load();
    }
  }
  return total_hashrate / 1000;
}

uint32_t Duco::getTotalShares() {
  uint32_t total_shares = 0;
  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      total_shares += this->job[i]->share_count.load();
    }
  }
  return total_shares;
}

uint32_t Duco::getAcceptedShares() {
  uint32_t total_accepted = 0;
  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      total_accepted += this->job[i]->accepted_share_count.load();
    }
  }
  return total_accepted;
}

uint32_t Duco::getDifficulty() {
  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      return this->job[i]->difficulty.load();
    }
  }
  return 0;
}

float Duco::getShareRate() {
  uint32_t total_secs = millis() / 1000;
  if (total_secs > 0) {
    return static_cast<float>(getTotalShares()) / total_secs;
  }
  return 0.0f;
}

float Duco::getAcceptedRate() {
  uint32_t total_shares = getTotalShares();
  if (total_shares > 0) {
    return (static_cast<float>(getAcceptedShares()) * 100.0f) / total_shares;
  }
  return 0.0f;
}

uint32_t Duco::getPing() {
  for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
    if (this->job[i] != nullptr) {
      return this->job[i]->ping.load();
    }
  }
  return 0;
}

/*
void Duco::print_and_display_report() {
    uint32_t total_hashrate = 0;
    uint32_t total_accepted = 0;
    uint32_t total_shares = 0;
    uint32_t current_ping = 0;
    uint32_t current_diff = 0;

    // 1. Automatically traverse the workers array across all available CPU cores
    for (int i = 0; i < SOC_CPU_CORES_NUM; i++) {
        if (this->job[i] != nullptr) {
            // Using .load() for thread-safe atomic reading from Core 1 to Core 0
            total_hashrate += this->job[i]->hashrate.load();
            total_accepted += this->job[i]->accepted_share_count.load();
            total_shares   += this->job[i]->share_count.load();

            // Extract ping and difficulty from the first available active core
            if (current_ping == 0) current_ping = this->job[i]->ping.load();
            if (current_diff == 0) current_diff = this->job[i]->difficulty.load();
        }
    }

    // Convert raw hashrate to kH/s
    float hashrate_kH = total_hashrate / 1000.0f;

    // 2. Calculate Uptime strictly using standard types (Safe from memory fragmentation)
    uint32_t total_secs = millis() / 1000;

    // 3. Calculate efficiency (Accept Rate) with division-by-zero protection
    float accept_rate = 100.0f;
    if (total_shares > 0) {
        accept_rate = (static_cast<float>(total_accepted) * 100.0f) / total_shares;
    }

    // 4. Calculate shares found per second (Share Rate)
    float sharerate = 0.0f;
    if (total_secs > 0) {
        sharerate = static_cast<float>(total_shares) / total_secs;
    }

    // 5. Output beautiful and clean metrics into the native ESPHome logger
    ESP_LOGI("duco", "Report - Hashrate: %.1f kH/s | Accepted: %u/%u (%.1f%%) | Uptime: %s | ShareRate: %.1f/s | Ping:
%ums", hashrate_kH, total_accepted, total_shares, accept_rate, uptime_buf, sharerate, current_ping);

    // 6. Forward processed data to your display output engine using standard std::string conversion
    // If your display function expects raw 'const char*', append '.c_str()' to variables accordingly.
    display_mining_results(
        std::to_string(hashrate_kH),
        std::to_string(total_accepted),
        std::to_string(total_shares),
        std::string(uptime_buf),
        this->configuration->host, // Directly accesses the active pool IP stored in config
        std::to_string(current_diff),
        std::to_string(sharerate),
        std::to_string(current_ping),
        std::to_string(accept_rate)
    );
}
*/
}  // namespace esphome::duco
