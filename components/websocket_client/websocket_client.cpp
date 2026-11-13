#include "websocket_client.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/util.h"
#ifdef USE_WEBSOCKET_CLIENT_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif

namespace esphome::websocket_client {

static const char *const TAG = "websocket_client";

void WebSocketClient::setup() {
  this->lock_ = xSemaphoreCreateMutex();
  this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
  this->enabled_ = this->enable_on_boot_;
  // Pre-allocate assembly buffer once to avoid heap fragmentation on incoming frames
  this->rx_assembly_buffer_.reserve(this->buffer_size_);
  if (this->enabled_) {
    this->schedule_reconnect_(false);
  } else {
    this->state_ = ConnectionState::STOPPED;
  }
}

void WebSocketClient::start_client_() {
  if (!this->enabled_)
    return;

  // If Wi-Fi link is completely down, don't even let native driver try
  if (!network::is_connected()) {
    this->schedule_reconnect_(false);
    return;
  }

  if (this->client_ != nullptr) {
    this->stop_client_();
  }

  this->state_ = ConnectionState::CONNECTING;

  esp_websocket_client_config_t config{};
  config.uri = this->url_;
  config.ping_interval_sec = this->ping_interval_sec_;
  config.buffer_size = this->buffer_size_;
  config.disable_auto_reconnect = true;
  config.transport = this->ssl_ ? WEBSOCKET_TRANSPORT_OVER_SSL : WEBSOCKET_TRANSPORT_OVER_TCP;
  config.task_stack = this->ssl_ ? 8192 : 4096;

  if (this->user_agent_ != nullptr && this->user_agent_[0] != '\0') {
    config.user_agent = this->user_agent_;
  }

  if (this->ssl_) {
#ifdef USE_WEBSOCKET_CLIENT_CERT_BUNDLE
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  }

  this->client_ = esp_websocket_client_init(&config);
  if (this->client_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize WebSocket client structure");
    this->schedule_reconnect_(false);
    return;
  }

  esp_websocket_register_events(this->client_, WEBSOCKET_EVENT_ANY, &WebSocketClient::ws_event_handler, (void *) this);

  if (esp_websocket_client_start(this->client_) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to execute esp_websocket_client_start");
    this->stop_client_();
    this->schedule_reconnect_(false);
  }
}

void WebSocketClient::clear_rx_buffers_() {
  if (this->lock_ != nullptr && xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
    this->rx_assembly_buffer_.clear();
    this->rx_msg_queue_.clear();
    this->state_data_pending_ = false;
    xSemaphoreGive(this->lock_);
  }
}

void WebSocketClient::stop_client_() {
  this->clear_rx_buffers_();

  if (this->client_ != nullptr) {
    esp_websocket_client_handle_t client_to_destroy = this->client_;
    this->client_ = nullptr;

    // Allocate smaller stack for plain TCP cleanup to reduce contiguous heap pressure
    uint32_t cleanup_stack = this->ssl_ ? 4096 : 2048;

    // Destroy client in a detached FreeRTOS task to prevent blocking the main ESPHome loop
    BaseType_t res = xTaskCreate(
        [](void *param) {
          auto *handle = static_cast<esp_websocket_client_handle_t>(param);
          esp_websocket_client_stop(handle);
          esp_websocket_client_destroy(handle);
          vTaskDelete(nullptr);
        },
        "ws_destroy", cleanup_stack, client_to_destroy, 1, nullptr);

    if (res != pdPASS) {
      // Fallback to synchronous cleanup if heap fragmentation prevents task allocation
      ESP_LOGW(TAG, "Failed to spawn cleanup task, falling back to synchronous destroy");
      esp_websocket_client_stop(client_to_destroy);
      esp_websocket_client_destroy(client_to_destroy);
    }
  }
}

void WebSocketClient::loop() {
  uint32_t now = millis();

  // Defer callback execution to the main thread for thread-safety compliance
  if (this->state_connected_pending_) {
    this->state_connected_pending_ = false;
    this->connect_callbacks_.call();
  }

  if (this->state_disconnected_pending_) {
    this->state_disconnected_pending_ = false;
    this->disconnect_callbacks_.call(this->disconnect_reason_);
  }

  // Handle incoming data queue from thread context safely
  std::vector<std::string> local_queue;
  if (this->state_data_pending_ && this->lock_ != nullptr) {
    if (xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
      local_queue = std::move(this->rx_msg_queue_);
      this->rx_msg_queue_.clear();
      this->state_data_pending_ = false;
      xSemaphoreGive(this->lock_);
    }
  }

  // Dispatch all fully assembled packets sequentially without thread interruption
  for (const auto &msg : local_queue) {
    if (!msg.empty()) {
      this->receive_callbacks_.call(msg);
    }
  }

  // Freeze the reconnection state machine if the Wi-Fi network link is not active
  if (this->state_ == ConnectionState::DELAYED) {
    if (!network::is_connected()) {
      // Shift the timer forward continuously so the countdown only ticks while online
      this->reconnect_timer_start_ms_ = now;
      return;
    }

    if (now - this->reconnect_timer_start_ms_ >= this->actual_delay_ms_) {
      this->start_client_();
    }
  }
}

void WebSocketClient::dump_config() {
  ESP_LOGCONFIG(TAG,
                "WebSocket Client Profile:\n"
                "  Endpoint URI: %s\n"
                "  SSL: %s\n"
                "  Port: %u\n"
                "  Buffer Size: %u bytes\n"
                "  User-Agent: %s\n"
                "  Heartbeat Period: %u s\n"
                "  Base Delay Boundary: %u ms\n"
                "  Ceiling Delay Boundary: %u ms\n"
                "  Stability Threshold: %u ms\n"
                "  Jitter Variance Factor: %u%%\n"
                "  Enabled on Boot: %s\n"
                "  Currently Enabled: %s",
                this->url_, YESNO(this->ssl_), this->port_, static_cast<unsigned int>(this->buffer_size_),
                this->user_agent_, static_cast<unsigned int>(this->ping_interval_sec_),
                static_cast<unsigned int>(this->min_reconnect_delay_ms_),
                static_cast<unsigned int>(this->max_reconnect_delay_ms_),
                static_cast<unsigned int>(this->stable_session_duration_ms_),
                static_cast<unsigned int>(this->jitter_percent_), YESNO(this->enable_on_boot_), YESNO(this->enabled_));
}

void WebSocketClient::reconnect() {
  bool was_connected = (this->state_ == ConnectionState::CONNECTED);

  if (was_connected || this->state_ == ConnectionState::CONNECTING) {
    ESP_LOGW(TAG, "Forced manual reconnect requested...");

    this->state_connected_pending_ = false;
    this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
    this->schedule_reconnect_(false);
    this->stop_client_();

    if (was_connected) {
      // Notify listeners about manual session termination (code 1000: Normal Closure)
      this->disconnect_reason_ = 1000;
      this->state_disconnected_pending_ = true;
    }
    return;
  }

  this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
  this->schedule_reconnect_(false);
}

void WebSocketClient::schedule_reconnect_(bool policy_close) {
  if (!this->enabled_)
    return;
  this->state_ = ConnectionState::DELAYED;
  this->reconnect_timer_start_ms_ = millis();

  if (policy_close) {
    // Policy Violation code 1008 forces immediate maximum delay penalty
    this->current_reconnect_delay_ms_ = this->max_reconnect_delay_ms_;
  } else if (this->min_reconnect_delay_ms_ == this->max_reconnect_delay_ms_) {
    this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
  }

  this->actual_delay_ms_ = this->calculate_jitter_(this->current_reconnect_delay_ms_);

  ESP_LOGW(TAG, "Reconnection scheduled in %u ms (base: %u ms)", static_cast<unsigned int>(this->actual_delay_ms_),
           static_cast<unsigned int>(this->current_reconnect_delay_ms_));

  // Backoff escalation execution step
  if (this->min_reconnect_delay_ms_ != this->max_reconnect_delay_ms_ && !policy_close) {
    this->current_reconnect_delay_ms_ = std::min(this->current_reconnect_delay_ms_ * 2, this->max_reconnect_delay_ms_);
  }
}

uint32_t WebSocketClient::calculate_jitter_(uint32_t base_delay) {
  if (this->jitter_percent_ == 0) {
    return std::clamp(base_delay, this->min_reconnect_delay_ms_, this->max_reconnect_delay_ms_);
  }

  int32_t range = (base_delay * this->jitter_percent_) / 100;
  int32_t rand_val = random_uint32() % (range * 2 + 1);
  int32_t offset = rand_val - range;

  int32_t final_delay = static_cast<int32_t>(base_delay) + offset;

  // Enforce quantization to multiples of 2000 ms (2 seconds)
  final_delay = (final_delay / 2000) * 2000;

  if (final_delay < 0)
    final_delay = 0;

  // Apply [min, max] bound limit restriction
  return std::clamp(static_cast<uint32_t>(final_delay), this->min_reconnect_delay_ms_, this->max_reconnect_delay_ms_);
}

void WebSocketClient::send_message(const std::string &message) {
  // Capture local handle copy to prevent race condition if background task nulls client_
  esp_websocket_client_handle_t client = this->client_;

  if (this->state_ != ConnectionState::CONNECTED || client == nullptr || !esp_websocket_client_is_connected(client)) {
    ESP_LOGW(TAG, "Cannot dispatch payload: client is offline");
    return;
  }

  ESP_LOGD(TAG, "Payload: %.32s, length: %d bytes", message.c_str(), message.length());

  int len = esp_websocket_client_send_text(client, message.c_str(), message.length(), pdMS_TO_TICKS(500));
  if (len < 0) {
    ESP_LOGE(TAG, "Failed to send message...");
  } else {
    ESP_LOGD(TAG, "Successfully sent %d bytes", len);
  }
}

void WebSocketClient::ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *client = static_cast<WebSocketClient *>(handler_args);
  auto *data = static_cast<esp_websocket_event_data_t *>(event_data);
  client->handle_event(event_id, data);
}

void WebSocketClient::handle_event(int32_t event_id, esp_websocket_event_data_t *data) {
  if (this->state_ != ConnectionState::CONNECTING && this->state_ != ConnectionState::CONNECTED) {
    return;
  }

  uint32_t now = millis();

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "Link established successfully");
      this->state_ = ConnectionState::CONNECTED;
      this->session_start_ms_ = now;
      this->state_connected_pending_ = true;
      this->clear_rx_buffers_();
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "Link termination event detected");
      this->disconnect_reason_ = data->error_handle.esp_tls_last_esp_err;
      this->state_disconnected_pending_ = true;

      {
        bool is_stable = (this->state_ == ConnectionState::CONNECTED) &&
                         (now - this->session_start_ms_ >= this->stable_session_duration_ms_);
        bool policy_close = (data->error_handle.esp_tls_last_esp_err == 1008);

        if (is_stable && !policy_close) {
          // Reset delay base strictly on long-lived connections
          this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
        }
        this->schedule_reconnect_(policy_close);
      }

      // Transition state to DELAYED before spawning cleanup task to block trailing events
      this->stop_client_();
      break;

    case WEBSOCKET_EVENT_DATA:
      if (data->data_ptr != nullptr && data->data_len > 0 && this->lock_ != nullptr) {
        if (xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
          if (data->payload_offset == 0) {
            this->rx_assembly_buffer_.assign(static_cast<const char *>(data->data_ptr), data->data_len);
          } else {
            if (this->rx_assembly_buffer_.length() == data->payload_offset) {
              this->rx_assembly_buffer_.append(static_cast<const char *>(data->data_ptr), data->data_len);
            } else {
              ESP_LOGE(TAG, "Buffer offset mismatch! Expected %d, got %d", (int) this->rx_assembly_buffer_.length(),
                       (int) data->payload_offset);
              this->rx_assembly_buffer_.clear();
            }
          }

          // Strict boundary assertion: push to queue only when accumulated buffer size matches or exceeds reported
          // footprint
          if (!this->rx_assembly_buffer_.empty() && (this->rx_assembly_buffer_.length() >= data->payload_len) &&
              data->fin) {
            // Cap queue size to prevent heap exhaustion if main loop stalls (e.g., during OTA or flash writes)
            if (this->rx_msg_queue_.size() < 16) {
              this->rx_msg_queue_.push_back(std::move(this->rx_assembly_buffer_));
              this->state_data_pending_ = true;
            } else {
              ESP_LOGW(TAG, "RX message queue full, dropping frame");
            }
            this->rx_assembly_buffer_.clear();
            this->rx_assembly_buffer_.reserve(this->buffer_size_);
          }

          xSemaphoreGive(this->lock_);
        }
      }
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "Internal client socket error exception");
      break;
  }
}

}  // namespace esphome::websocket_client
