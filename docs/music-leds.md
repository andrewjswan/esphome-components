## Music Leds / Sound Reactive Component

[![esp32_arduino](https://img.shields.io/badge/ESP32-Arduino-darkcyan.svg)](https://esphome.io/)

The `music_leds` component is a high-performance, real-time audio visualization engine for addressable LED strips. It captures raw hardware audio from an I2S or PDM microphone, performs real-time Fast Fourier Transform (FFT) analysis, applies psychoacoustic range scaling, filters the noise floor via a dynamic noise gate, and exposes a comprehensive set of standalone lighting effects and event triggers.

### Configuration Variables

#### Main Component

!!! example annotate "Example configuration entry"

    ``` { .yaml .copy .annotate }
    music_leds:
      id: my_music_engine
      microphone: my_i2s_mic
      sample_scale: 24
      scaling_mode: SQUARE_ROOT
      beat_sensitivity: 65
      noise_gate_floor: 0.10
    ```

* **id** (*Optional*, string): The unique identifier for this instance of the audio engine.
* **microphone** (*Optional*, [ID](https://esphome.io/guides/configuration-types/#id)): The ID of the configured hardware microphone component source.
* **sample_scale** (*Optional*, integer): Establishes an inverse division factor (`1.0f / sample_scale`) to attenuate raw hardware amplitudes, aligning framework-specific gain differences to ensure clean dynamic range processing and complete noise cutoff in quiet environments (use `24` for standard 16-bit I2S microphones like INMP441, or `4` for PDM microphones like SPM1423). Defaults to `24`.
* **scaling_mode** (*Optional*, string): Specifies the mathematical spectrum compression profile used to map linear FFT energy levels to nonlinear human psychoacoustic perception. Defaults to `SQUARE_ROOT`.
    * `LINEAR`: Direct mapping. Best used for raw instrumentation over generic visual tracking.
    * `LOGARITHMIC`: Wide dynamic range compression. Excellent for emphasizing ultra-quiet details.
    * `SQUARE_ROOT`: Balanced standard optimization. Ideal for energetic, rhythmic tracks.
* **beat_sensitivity** (*Optional*, integer): Sets the internal statistical onset trigger threshold multiplier for rhythmic tracking. High values make beat detection highly volatile, while lower values restrict triggers to massive transient baselines. Valid range is `1` to `100`. Defaults to `65`.
* **noise_gate_floor** (*Optional*, float): The base silence energy threshold before the dynamic gate engages. When macro band signals fall below this, the engine smoothly bleeds register states down to absolute zero to prevent ghost flickering. Valid range is `0.001` to `0.5`. Defaults to `0.10`.
* **pre_amp_gain** (*Optional*, float): Manual global pre-amplification multiplier applied directly to the macro energy pools. Valid range is `1.0` to `20.0`. Defaults to `1.0`.
* **sample_gain** (*Optional*, integer): Sets the primary linear pre-equalizer preamp level factor used to shift the raw spectrum curves inside the DSP viewport. Valid range is `0` to `255`. Defaults to `60`.
* **task_core** (*Optional*, integer): Explicitly pin the real-time audio computation task loop to an isolated hardware processor core (`0` or `1`) on dual-core microcontrollers to protect against Wi-Fi loop stuttering. Defaults to `1`.
* **task_priority** (*Optional*, integer): Defines the execution task thread scheduling priority. Higher numbers guarantee execution lock-step precision during dense network routing routines. Valid range is `1` to `24`. Defaults to `10`.

#### Light Effects

To map the engine datasets straight onto your addressable LED panels, attach the `music_leds_effect` register block to any addressable light configuration.

!!! example annotate "Example configuration"

    ``` { .yaml .copy .annotate }
    light:
      - platform: neopixel
        pin: GPIO16
        num_leds: 60
        name: "Sound Reactive Strip"
        effects:
          - music_leds_effect:
              name: "Grav with Music"
              mode: GRAV
    ```

* **name** (**Required**, string): The display name of the effect exposed inside the Home Assistant frontend interface.
* **mode** (*Optional*, string): Selects the specific real-time rendering logic applied to the array. Defaults to `PIXELS`.

#### Available Visual Modes

The component provides a large list of precompiled, reactive animation profiles:

* `GRAV`, `GRAVICENTER`, `GRAVICENTRIC`, `GRAVIMETER`: Gravity-physics-based peak drops, centering blocks, and multi-directional kinetic accelerators reacting primarily to low-frequency hits.
* `PIXELS`, `MIDNOISE`, `MATRIPIX`, `NOISEFIRE`: Noise-mapped, responsive fields optimized to transform rapid transitions into structural color matrices.
* `JUNGLES`, `PLASMOID`: Fluid organic ambient spectrum clouds that speed up or cycle hue maps based on multi-band energy density.
* `RIPPLEPEAK`, `PUDDLEPEAK`, `PUDDLES`: Emits ripple waves originating from anchor canvas centers triggered dynamically by transient musical peaks.
* `PIXELWAVE`, `WATERFALL`, `DJLIGHT`: Traditional linear color charts, falling spectrum graphs, and high-tempo virtual stage lighting mappings.

#### On Sound Loop Trigger

An automation trigger that fires on every calculated audio task iteration frame, exporting instant fixed-point and floating-point parameters for custom automation scripts.

!!! example annotate "Example configuration"

    ``` { .yaml .copy .annotate }
    music_leds:
      ...
      on_sound_loop:
        then:
          - if:
              condition:
                lambda: return sample_peak;
              then:
                - logger.log: "Audio hardware threshold saturation point clip hit!"
    ```

The trigger yields four explicit local context variables back into the automation block scope:
* `volume_smth` (`float`): Perceptually smoothed current frame volume baseline envelope.
* `volume_raw` (`int16`): The direct, raw quantitative instant volume frame level.
* `fft_major_peak` (`float`): Dominant evaluated major frequency coordinate bin peak in Hz.
* `sample_peak` (`bool`): Active Boolean clipping monitor flag. Evaluates to `true` if the analog-to-digital input pipeline hits full scale hardware saturation limits.

### Complete Example YAML Implementation

Below is a complete implementation blueprint pairing an I2S hardware mic setup with an addressable WS2812B lighting array running sound-reactive matrices:

!!! example annotate "Configuration"

    ``` { .yaml .copy .annotate }
    i2s_audio:
      i2s_lrclk_pin: GPIO33
      i2s_bclk_pin: GPIO27

    microphone:
      - platform: i2s_audio
        id: inmp441_mic
        i2s_din_pin: GPIO32
        adc_type: external
        pdm: false
        channel: left
        bits_per_sample: 16bit
        sample_rate: 10240 Hz

    music_leds:
      id: sound_engine
      microphone: inmp441_mic
      sample_scale: 24
      scaling_mode: SQUARE_ROOT
      beat_sensitivity: 70
      noise_gate_floor: 0.08
      task_core: 1
      task_priority: 12

    light:
      - platform: neopixelbus
        name: "Living Room Reactive Light"
        pin: GPIO3
        num_leds: 300
        variant: 800KBPS
        effects:
          - music_leds_effect:
              name: "Audio Gravity Beat"
              mode: GRAV
          - music_leds_effect:
              name: "Audio Plasma Clouds"
              mode: PLASMOID
    ```

### ESPHome package

!!! note
    - [Music Leds](https://andrewjswan.github.io/esphome-config/music-leds/) configuration
    - Full package: [/esphome-config/packages/neopixel_light_music_leds.yaml](https://github.com/andrewjswan/esphome-config/blob/main/packages/neopixel_light_music_leds.yaml)
    - ESPNow package: [/esphome-config/packages/neopixel_light_music_leds_espnow_master.yaml](https://github.com/andrewjswan/esphome-config/blob/main/packages/neopixel_light_music_leds_espnow_master.yaml)
