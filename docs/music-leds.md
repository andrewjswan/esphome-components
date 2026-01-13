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

## Local triggers

#### on_sound_loop

This trigger is called on every sound loop. In lambda's you can use local variables:

- **volume_smth** (float): Either sampleAvg or sampleAgc depending on soundAgc; smoothed sample.

- **volume_raw** (int16_t): Either sampleRaw or rawSampleAgc depending on soundAgc.

- **fft_major_peak** (float): FFT: strongest (peak) frequency.

- **sample_peak** (bool): Boolean flag for peak, responding routine may reset this flag. Auto-reset after 50ms.

!!! example annotate "On Sound Loop trigger"

    ``` { .yaml .copy .annotate }
    music_leds:
      id: music_light
      on_sound_loop:
        - logger.log:
            format: "${friendly_name} Sound loop, %f %d %f %d"
            args:
              - volume_smth
              - volume_raw
              - fft_major_peak
              - sample_peak
    ```

### ESPHome package

!!! note
    - [Music Leds](https://andrewjswan.github.io/esphome-config/music-leds/) configuration
    - Full package: [/esphome-config/packages/neopixel_light_music_leds.yaml](https://github.com/andrewjswan/esphome-config/blob/main/packages/neopixel_light_music_leds.yaml)
    - ESPNow package: [/esphome-config/packages/neopixel_light_music_leds_espnow_master.yaml](https://github.com/andrewjswan/esphome-config/blob/main/packages/neopixel_light_music_leds_espnow_master.yaml)
