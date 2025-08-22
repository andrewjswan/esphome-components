## Music Leds / Sound Reactive
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Ported `Sound Reactive` from WLED to ESPHome

### Configuration

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/andrewjswan/esphome-components
      ref: main
    components: [fastled_helper, music_leds]
    refresh: 60s

fastled_helper:
  music_leds: true

i2s_audio:
  i2s_lrclk_pin: GPIO15
  i2s_bclk_pin: GPIO14

microphone:
  - platform: i2s_audio
    id: adc_mic
    adc_type: external
    sample_rate: 10240
    bits_per_sample: 32bit
    i2s_din_pin: GPIO32
    channel: right

music_leds:
  id: music_light

light:
  - id: !extend neopixel_led
    effects:
      - music_leds_effect:
          name: Grav with Music
          mode: GRAV
```

### ESPHome package

!!! note
    Full package: https://github.com/andrewjswan/esphome-config/blob/main/packages/neopixel_light_music_leds.yaml
