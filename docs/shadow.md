## Shadow
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)
[![esp32_esp_idf](https://img.shields.io/badge/ESP--IDF-blue.svg)](https://esphome.io/)

Allows you to run a script in a parallel thread (Task)

### Configuration

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/esphome-components
      ref: main
    components: [shadow]
    refresh: 60s

script:
  - id: some_script
    then:
      - logger.log: "Script Running..."

shadow:
  id: esp_shadow
  script_id: some_script
  interval: 60
```
