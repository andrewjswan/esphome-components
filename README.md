[![esphome_badge](https://img.shields.io/badge/ESPHome-Components-blue.svg)](https://esphome.io/)
[![Build](https://github.com/andrewjswan/esphome-components/actions/workflows/build.yaml/badge.svg)](https://github.com/andrewjswan/esphome-components/actions/workflows/build.yaml)
[![GitHub](https://img.shields.io/github/license/andrewjswan/esphome-components?color=blue)](https://github.com/andrewjswan/esphome-components/blob/master/LICENSE)
[![StandWithUkraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://github.com/vshymanskyy/StandWithUkraine/blob/main/docs/README.md)

# ESPHome Components
External components for ESPHome

## Shadow
Allows you to run a script in a parallel thread (Task)

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/esphome-components
      ref: main
    components: [ shadow ]
    refresh: 60s

script:
  - id: some_script
    then:
      - logger.log: "Script Running..."

shadow:
  id: esp_shadow
  script_id: some_script
  interval: 60

ota:
  - platform: esphome
    on_begin:
      then:
        - lambda: |-
            id(esp_shadow)->stop();
```
