## Fastled Helper

[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Helper providing a set of palettes, working with palettes and additional functions.

### Configuration

!!! example annotate "Configuration"

    ``` { .yaml .copy .annotate }
    external_components:
      - source:
          type: git
          url: https://github.com/andrewjswan/esphome-components
          ref: main
        components: [fastled_helper]
        refresh: 60s

    fastled_helper:
      id: palettes
      palettes: true
      music_leds: false
    ```
