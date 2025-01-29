[![esphome_badge](https://img.shields.io/static/v1?label=ESPHome&message=Components&color=blue&logo=esphome)](https://esphome.io/)
[![Build](https://github.com/andrewjswan/esphome-components/actions/workflows/build.yaml/badge.svg)](https://github.com/andrewjswan/esphome-components/actions/workflows/build.yaml)
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)
[![esp32_esp_idf](https://img.shields.io/badge/ESP--IDF-blue.svg)](https://esphome.io/)
[![GitHub](https://img.shields.io/github/license/andrewjswan/esphome-components?color=blue)](https://github.com/andrewjswan/esphome-components/blob/master/LICENSE)
[![StandWithUkraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://github.com/vshymanskyy/StandWithUkraine/blob/main/docs/README.md)

# ESPHome Components
External components for ESPHome

## Matrix Lamp
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Component assembly of firmware adaptations based on the original idea of ​​Gyver Lamp, for ESPHome.
Has a separate repository. https://github.com/andrewjswan/matrix-lamp/

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/matrix-lamp
      ref: main
    components: [matrix_lamp]

matrix_lamp:
  id: matrix
  width: 16
  height: 16
  random: true
  scale_id: matrix_scale
  speed_id: matrix_speed
```

## Shadow
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)
[![esp32_esp_idf](https://img.shields.io/badge/ESP--IDF-blue.svg)](https://esphome.io/)

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

## NerdMiner
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

This component let you try to reach a bitcoin block with a small piece of hardware.

The main aim of this component is to let you learn more about minery and to have a beautiful piece of hardware in your desktop.
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/esphome-components
      ref: main
    components: [ nerdminer ]
    refresh: 60s

nerdminer:
  id: miner
  walletid: !secret wallet
  worker: "esphomeminer"
  pool: "public-pool.io"
  port: 21496

ota:
  - platform: esphome
    on_begin:
      then:
        - lambda: |-
            id(miner)->stop();
```

> [!NOTE]
> Based on [**NerdSoloMiner**](https://github.com/BitMaker-hub/NerdMiner_v2) by [BitMaker](https://github.com/BitMaker-hub)

## Fastled Helper
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Helper providing a set of palettes, working with palettes and additional functions.

```yaml
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

## Music Leds / Sound Reactive
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Ported `Sound Reactive` from WLED to ESPHome

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/esphome-components
      ref: main
    components: [music_leds]
    refresh: 60s

i2s_audio:
  i2s_lrclk_pin: 15
  i2s_bclk_pin: 14

microphone:
  - platform: i2s_audio
    id: adc_mic
    adc_type: external
    sample_rate: 10240
    bits_per_sample: 16bit
    i2s_din_pin: 32
    channel: right

music_leds:
  id: music_light
  mic_id: adc_mic

light:
  - id: !extend neopixel_led
    effects:
      - music_leds_effect:
          name: Binmap with Music
          mode: BINMAP
```
