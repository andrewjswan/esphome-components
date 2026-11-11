## WebSocket Client Component

[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)
[![esp32_esp_idf](https://img.shields.io/badge/ESP--IDF-blue.svg)](https://esphome.io/)

The `websocket_client` component allows your ESPHome node to connect as a client to an external WebSocket server. This enables bidirectional real-time communication, allowing your device to send messages and receive commands or data streams from a remote server, Home Assistant add-on, or local service.

### Configuration Variables

!!! example annotate "Example configuration entry"

    ``` { .yaml .copy .annotate }
    websocket_client:
      id: my_websocket_client
      url: "ws://192.168.1.100:9000/stream"
      ping_interval: 10s
      buffer_size: 2048

      on_connect:
        - logger.log: "WebSocket connection established!"
        - websocket_client.send:
            id: my_websocket_client
            message: '{"event": "node_online"}'

      on_disconnect:
        - logger.log:
            format: "Connection terminated. Reason Code: %d"
            args: [ reason ]

      on_receive:
        - logger.log:
            format: "Data payload captured: %s"
            args: [ x.c_str() ]

    binary_sensor:
      - platform: gpio
        pin: GPIO0
        name: "Manual Transmission Button"
        on_press:
          then:
            - websocket_client.send:
                id: my_websocket_client
                message: "Ping event trigger"
    ```

### Main Configuration

* **id** (*Optional*, ID): Manual ID for use in lambdas or other components.
* **url** (*Required*, string): WebSocket server URL (e.g., `ws://192.168.1.50:8123/ws` or `wss://example.com/endpoint`).
* **port** (*Optional*, integer): Target port (defaults to `80` for `ws://` and `443` for `wss://`).
* **ssl** (*Optional*, boolean): Enable/disable SSL/TLS (auto-detected from URL scheme if omitted).
* **user_agent** (*Optional*, string): Custom User-Agent header (defaults to `ESPHome/WSC (<version>)`).
* **ping_interval** (*Optional*, Time): Interval for keep-alive ping frames (defaults to `20s`).
* **buffer_size** (*Optional*, integer): Internal TX/RX memory buffer in bytes (defaults to `1024`).
* **enable_on_boot** (*Optional*, boolean): Automatically connect on startup (defaults to `true`).

#### Reconnection Settings

* **min_reconnect_delay** / **max_reconnect_delay** (*Optional*, Time): Backoff delay bounds for reconnections (defaults to `5s` and `130s`).
* **stable_session_duration** (*Optional*, Time): Time required to mark a session as stable (defaults to `60s`).
* **jitter_percent** (*Optional*, integer): Randomization factor (0-100%) applied to backoff delays (defaults to `20`).

#### Automation Triggers

* **on_connect** / **on_disconnect** (*Optional*, Action): Triggered upon connection or disconnection (`reason` code available on disconnect).
* **on_receive** (*Optional*, Action): Triggered when a text data packet is received (`x` contains the raw payload string).

#### Actions

* **`websocket_client.send`**: Sends a text or JSON payload (`id`, `message`).
* **`websocket_client.enable`** / **`websocket_client.disable`**: Manually start or tear down the socket session.

!!! example annotate "Sends a text message through the active WebSocket connection to the server"

    ``` { .yaml .copy .annotate }
    on_button_press:
      - websocket_client.send:
          id: my_websocket_client
          data: "Hello from ESPHome!"
    ```
