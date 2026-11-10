#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esp_websocket_client.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string>

namespace esphome::websocket_client {

enum class ConnectionState { STOPPED, CONNECTING, CONNECTED, DELAYED };

class WebSocketClient : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void enable() { 
    if (!this->enabled_) {
      this->enabled_ = true;
      this->start_client_();
    }
  }
  void disable() { 
    if (this->enabled_) {
      this->enabled_ = false;
      this->stop_client_();
      this->state_ = ConnectionState::STOPPED;
    }
  }
  bool is_enabled() const { return this->enabled_; }
  bool is_connected() const { return this->state_ == ConnectionState::CONNECTED; }
  void set_enable_on_boot(bool enable_on_boot) { this->enable_on_boot_ = enable_on_boot; }

  void set_url(const std::string &url) { this->url_ = url; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_ssl(bool ssl) { this->ssl_ = ssl; }
  void set_user_agent(const std::string &user_agent) { this->user_agent_ = user_agent; }
  void set_ping_interval(uint32_t seconds) { this->ping_interval_sec_ = seconds; }
  void set_buffer_size(uint32_t bytes) { this->buffer_size_ = bytes; }
  void set_min_reconnect_delay(uint32_t seconds) { this->min_reconnect_delay_ms_ = seconds * 1000; }
  void set_max_reconnect_delay(uint32_t seconds) { this->max_reconnect_delay_ms_ = seconds * 1000; }
  void set_stable_session_duration(uint32_t seconds) { this->stable_session_duration_ms_ = seconds * 1000; }
  void set_jitter_percent(uint32_t percent) { this->jitter_percent_ = percent; }

  void add_on_connect_callback(std::function<void()> &&cb) { this->connect_callbacks_.add(std::move(cb)); }
  void add_on_disconnect_callback(std::function<void(int)> &&cb) { this->disconnect_callbacks_.add(std::move(cb)); }
  void add_on_receive_callback(std::function<void(const std::string &)> &&cb) {
    this->receive_callbacks_.add(std::move(cb));
  }

  void send_message(const std::string &message);

  uint16_t get_port() const { return this->port_; }
  bool get_ssl() const { return this->ssl_; }

 protected:
  static void ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
  void handle_event(int32_t event_id, esp_websocket_event_data_t *data);

  bool enabled_{true};
  bool enable_on_boot_{true};

  void start_client_();
  void stop_client_();
  void schedule_reconnect_(bool policy_close);
  uint32_t calculate_jitter_(uint32_t base_delay);

  std::string url_;
  uint16_t port_{80};
  bool ssl_{false};
  std::string user_agent_{""};
  uint32_t ping_interval_sec_{20};
  uint32_t buffer_size_{1024};

  uint32_t min_reconnect_delay_ms_{5000};
  uint32_t max_reconnect_delay_ms_{130000};
  uint32_t stable_session_duration_ms_{60000};
  uint32_t jitter_percent_{20};

  esp_websocket_client_handle_t client_{nullptr};
  ConnectionState state_{ConnectionState::STOPPED};

  uint32_t current_reconnect_delay_ms_{0};
  uint32_t reconnect_timer_start_ms_{0};
  uint32_t actual_delay_ms_{0};
  uint32_t session_start_ms_{0};

  bool state_connected_pending_{false};
  bool state_disconnected_pending_{false};
  bool state_data_pending_{false};
  bool state_stop_pending_{false};
  int disconnect_reason_{0};

  std::string rx_assembly_buffer_;
  std::vector<std::string> rx_msg_queue_;

  SemaphoreHandle_t lock_{nullptr};

  CallbackManager<void()> connect_callbacks_;
  CallbackManager<void(int)> disconnect_callbacks_;
  CallbackManager<void(const std::string &)> receive_callbacks_;
};

template<typename... Ts> class WebSocketEnableAction : public Action<Ts...> {
 public:
  explicit WebSocketEnableAction(WebSocketClient *parent) : parent_(parent) {}
  void play(Ts... args) override { this->parent_->enable(); }
 protected:
  WebSocketClient *parent_;
};

template<typename... Ts> class WebSocketDisableAction : public Action<Ts...> {
 public:
  explicit WebSocketDisableAction(WebSocketClient *parent) : parent_(parent) {}
  void play(Ts... args) override { this->parent_->disable(); }
 protected:
  WebSocketClient *parent_;
};

class WebSocketConnectTrigger : public Trigger<> {
 public:
  explicit WebSocketConnectTrigger(WebSocketClient *parent) {
    parent->add_on_connect_callback([this]() { this->trigger(); });
  }
};

class WebSocketDisconnectTrigger : public Trigger<int> {
 public:
  explicit WebSocketDisconnectTrigger(WebSocketClient *parent) {
    parent->add_on_disconnect_callback([this](int reason) { this->trigger(reason); });
  }
};

class WebSocketReceiveTrigger : public Trigger<std::string> {
 public:
  explicit WebSocketReceiveTrigger(WebSocketClient *parent) {
    parent->add_on_receive_callback([this](const std::string &data) { this->trigger(data); });
  }
};

template<typename... Ts> class WebSocketSendAction : public Action<Ts...> {
 public:
  explicit WebSocketSendAction(WebSocketClient *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, message)

  void play(Ts... args) override {
    auto msg = this->get_message(args...);
    this->parent_->send_message(msg);
  }

 protected:
  WebSocketClient *parent_;
};

}  // namespace esphome::websocket_client
