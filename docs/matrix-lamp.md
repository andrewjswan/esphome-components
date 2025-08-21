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
