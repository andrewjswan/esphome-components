## NerdMiner

[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

This component let you try to reach a bitcoin block with a small piece of hardware.

The main aim of this component is to let you learn more about minery and to have a beautiful piece of hardware in your desktop.

### Configuration

!!! example annotate "Configuration"

    ``` { .yaml .copy .annotate }
    external_components:
      - source:
          type: git
          url: https://github.com/andrewjswan/esphome-components
          ref: main
        components: [nerdminer]
        refresh: 60s
    
    nerdminer:
      id: miner
      walletid: !secret wallet
      worker: "esphomeminer"
      pool: "public-pool.io"
      port: 21496
    ```

### Based

!!! note
    Based on [**NerdSoloMiner**](https://github.com/BitMaker-hub/NerdMiner_v2) by [BitMaker](https://github.com/BitMaker-hub)
