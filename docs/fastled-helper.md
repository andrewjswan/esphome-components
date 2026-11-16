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

* **id** (*Optional*, string): The unique identifier for this instance of the audio engine.
* **palettes** (*Optional*, boolean): Enables support for internal color palettes and gradient functions. It activates global variables for tracking and changing palettes dynamically, which is required for palette-based light effects. Defaults to `false`.
* **music_leds** (*Optional*, boolean): Enables support for audio-reactive palettes and musical light effects. It exposes the Fast Fourier Transform (FFT) spectrum interface to handle real-time sound data and equalizer rendering. Requires palettes to be enabled. Defaults to `false`.
