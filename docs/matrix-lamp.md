## Matrix Lamp
[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

Component assembly of firmware adaptations based on the original idea of ​​Gyver Lamp, for ESPHome.
Has a [separate repository](https://github.com/andrewjswan/matrix-lamp/).

### Configuration

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

## Firmware
You can install the firmware using [ESPHome](https://esphome.io/), using one of the ready-made configuration files or use [**MatrixLamp - ESP Web Tools**](https://andrewjswan.github.io/matrix-lamp/) - a convenient tool for installing and updating the firmware of ESP32 devices in the browser.
