## WebSocket Client Component

[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)
[![esp32_esp_idf](https://img.shields.io/badge/ESP--IDF-blue.svg)](https://esphome.io/)

The `websocket_client` component allows your ESPHome node to connect as a client to an external WebSocket server. This enables bidirectional real-time communication, allowing your device to send messages and receive commands or data streams from a remote server, Home Assistant add-on, or local service.

### Configuration Variables

!!! example annotate "Example configuration entry"

    ``` { .yaml .copy .annotate }
    websocket_client:
      id: my_websocket_client
      url: "ws://192.168.1.50:8123/ws"
      on_connect:
        - logger.log: "Connected to WebSocket server!"
      on_disconnect:
        - logger.log: "Disconnected from WebSocket server."
      on_message:
        - logger.log:
            format: "Received message: %s"
            args: [x]
    ```

* **id** (*Optional*, ID): Manually specify the ID for this component to use it in lambda actions or other components.
* **url** (*Required*, string): The WebSocket server URL to connect to (e.g., `ws://example.com/socket` or `wss://secure.example.com/socket`).

* **on_connect** (*Optional*, Action): An automation to perform when the WebSocket client successfully connects to the server.
* **on_disconnect** (*Optional*, Action): An automation to perform when the WebSocket connection is lost or terminated.
* **on_message** (*Optional*, Action): An automation triggered whenever a text message is received from the WebSocket server. The received message string is available in lambdas via the variable `x`.

### Actions

#### websocket_client.send

Sends a text message through the active WebSocket connection to the server.

    ``` { .yaml .copy .annotate }
    on_button_press:
      - websocket_client.send:
          id: my_websocket_client
          data: "Hello from ESPHome!"
    ```
id (Optional, ID): The ID of the websocket client component. Use this if you have multiple instances.

data (Required, string, templatable): The text message payload to send to the server.
