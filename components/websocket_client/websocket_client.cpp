#include "websocket_client.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/util.h"
#include "esp_wifi.h"
#ifdef USE_WS_BRIDGE_CERT_BUNDLE
#include "esp_crt_bundle.h"
#endif

namespace esphome::namespace websocket_client {

static const char *const TAG = "websocket_client";

void WebSocketClient::setup() {
  this->lock_ = xSemaphoreCreateMutex();
  this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
  this->start_client_();
}

void WebSocketClient::start_client_() {
  // If Wi-Fi link is completely down, don't even let native driver try
  if (!network::is_connected()) {
    this->schedule_reconnect_(false);
    return;
  }

  this->state_ = ConnectionState::CONNECTING;

  esp_websocket_client_config_t config{};
  config.uri = this->url_.c_str();
  config.ping_interval_sec = this->ping_interval_sec_;
  config.buffer_size = this->buffer_size_;
  config.disable_auto_reconnect = true;
  config.transport = this->ssl_ ? WEBSOCKET_TRANSPORT_OVER_SSL : WEBSOCKET_TRANSPORT_OVER_TCP;
  config.task_stack = this->ssl_ ? 8192 : 4096;

  if (this->ssl_) {
#ifdef USE_WS_BRIDGE_CERT_BUNDLE
    config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  }

  // if (this->ssl_) {
  //   // Completely bypass certificate bundle validation to save memory and skip Let's Encrypt issues
  //   config.skip_cert_common_name_check = true;
  //   config.use_global_ca_store = false;
  //   config.cert_pem = nullptr;
  //   config.crt_bundle_attach = nullptr;
  //   ESP_LOGW(TAG, "Secured WSS mode: SSL/TLS certificate validation is explicitly bypassed");
  // }

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

void WebSocketClient::stop_client_() {
  if (this->client_ != nullptr) {
    esp_websocket_client_destroy(this->client_);
    this->client_ = nullptr;
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

  // Handle incoming data in the main thread context to ensure thread-safety
  if (this->state_data_pending_ && this->lock_ != nullptr) {
    std::string local_buffer;
    if (xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
      local_buffer = std::move(this->rx_queue_buffer_);
      this->rx_queue_buffer_.clear();
      this->state_data_pending_ = false;
      xSemaphoreGive(this->lock_);
    }

    if (!local_buffer.empty()) {
      this->receive_callbacks_.call(local_buffer);
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
  ESP_LOGCONFIG(TAG, "WebSocket Client Profile:");
  ESP_LOGCONFIG(TAG, "  Endpoint URI: %s", this->url_.c_str());
  ESP_LOGCONFIG(TAG, "  SSL: %s", YESNO(this->ssl_));
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Heartbeat Period: %u s", static_cast<unsigned int>(this->ping_interval_sec_));
  ESP_LOGCONFIG(TAG, "  Allocation Buffer: %u bytes", static_cast<unsigned int>(this->buffer_size_));
  ESP_LOGCONFIG(TAG, "  Base Delay Boundary: %u ms", static_cast<unsigned int>(this->min_reconnect_delay_ms_));
  ESP_LOGCONFIG(TAG, "  Ceiling Delay Boundary: %u ms", static_cast<unsigned int>(this->max_reconnect_delay_ms_));
  ESP_LOGCONFIG(TAG, "  Stability Threshold: %u ms", static_cast<unsigned int>(this->stable_session_duration_ms_));
  ESP_LOGCONFIG(TAG, "  Jitter Variance Factor: %u%%", static_cast<unsigned int>(this->jitter_percent_));
}

void WebSocketClient::schedule_reconnect_(bool policy_close) {
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
    return base_delay;
  }

  int32_t range = (base_delay * this->jitter_percent_) / 100;
  int32_t rand_val = random_uint32() % (range * 2 + 1);
  int32_t offset = rand_val - range;
  int32_t final_delay = static_cast<int32_t>(base_delay) + offset;

  // Enforce quantization to multiples of 2000 ms (2 seconds)
  final_delay = (final_delay / 2000) * 2000;

  if (final_delay < 0)
    final_delay = 0;

  // Apply maximum upper bound limit restriction
  return std::min(static_cast<uint32_t>(final_delay), this->max_reconnect_delay_ms_);
}

void WebSocketClient::send_message(const std::string &message) {
  if (this->state_ != ConnectionState::CONNECTED || this->client_ == nullptr) {
    ESP_LOGW(TAG, "Cannot dispatch payload: client is offline");
    return;
  }
  esp_websocket_client_send_text(this->client_, message.c_str(), message.length(), portMAX_DELAY);
}

void WebSocketClient::ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *client = static_cast<WebSocketClient *>(handler_args);
  auto *data = static_cast<esp_websocket_event_data_t *>(event_data);
  client->handle_event(event_id, data);
}

void WebSocketClient::handle_event(int32_t event_id, esp_websocket_event_data_t *data) {
  uint32_t now = millis();

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      ESP_LOGI(TAG, "Link established successfully");
      this->state_ = ConnectionState::CONNECTED;
      this->session_start_ms_ = now;
      this->state_connected_pending_ = true;
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "Link termination event detected");
      this->stop_client_();

      this->disconnect_reason_ = data->error_handle.esp_tls_last_esp_err;
      this->state_disconnected_pending_ = true;

      if (this->lock_ != nullptr && xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
        this->rx_queue_buffer_.clear();
        this->state_data_pending_ = false;
        xSemaphoreGive(this->lock_);
      }

      {
        bool is_stable = (now - this->session_start_ms_ >= this->stable_session_duration_ms_);
        bool policy_close = (data->error_handle.esp_tls_last_esp_err == 1008);

        if (is_stable && !policy_close) {
          // Reset delay base strictly on long-lived connections
          this->current_reconnect_delay_ms_ = this->min_reconnect_delay_ms_;
        }
        this->schedule_reconnect_(policy_close);
      }
      break;

    case WEBSOCKET_EVENT_DATA:
      if (data->data_ptr != nullptr && data->data_len > 0 && this->lock_ != nullptr) {
        if (xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE) {
          if (data->payload_offset == 0) {
            this->rx_queue_buffer_.assign(static_cast<const char *>(data->data_ptr), data->data_len);
          } else {
            this->rx_queue_buffer_.append(static_cast<const char *>(data->data_ptr), data->data_len);
          }

          if (data->payload_offset + data->data_len >= data->payload_len) {
            this->state_data_pending_ = true;
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

}  // namespace esphome::namespacewebsocket_client
