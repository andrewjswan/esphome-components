#include "music_leds.h"

#include "esphome/components/fastled_helper/colors.h"
#include "esphome/components/fastled_helper/utils.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#define DEBUG

#ifdef DEBUG
#include "debug.h"
#endif

namespace esphome::music_leds {

// *****************************************************************************
// Data
// *****************************************************************************

// allocates effect data buffer on heap and initialises (erases) it
bool MusicLeds::allocateData(size_t len) {
  if (len == 0) {
    return false;  // nothing to do
  }
  if (this->data && this->_dataLen >= len) {  // already allocated enough (reduce fragmentation)
    if (this->start_effect_) {
      memset(data, 0, len);  // erase buffer if called during effect initialisation
    }
    return true;
  }

  this->deallocateData();  // if the old buffer was smaller release it first
  // Do not use SPI RAM on ESP32 since it is slow
  this->data = (byte *) calloc(len, sizeof(byte));
  if (!this->data) {
    this->status_momentary_warning("Effect data, allocation failed!");
    return false;
  }  // allocation failed

  this->_dataLen = len;
  return true;
}

void MusicLeds::deallocateData() {
  if (!this->data) {
    this->_dataLen = 0;
    return;
  }
  // check that we don't have a dangling / inconsistent data pointer
  if (this->_dataLen > 0) {
    free(this->data);
  }
  this->data = nullptr;
  _dataLen = 0;
}

// *****************************************************************************
// Effects
// *****************************************************************************
void MusicLeds::show_frame(PLAYMODE CurrentMode, esphome::Color current_color, light::AddressableLight *p_it) {
  if (!this->is_running() && !this->microphone_is_running()) {
    return;
  }

  fastled_helper::init_leds(p_it->size());
  fastled_helper::register_fft_spectrum(this->features_.fft_result);

  this->leds_num = p_it->size();

  this->main_color = CRGB(current_color.r, current_color.g, current_color.b);
  if ((int) fastled_helper::current_palette == 0) {
    // 5% from main color
    this->back_color = CRGB(current_color.r / 100 * 5, current_color.g / 100 * 5, current_color.b / 100 * 5);
  } else {
    this->back_color = CRGB::Black;
  }
  if (this->start_effect_) {
    this->store = UINT8_MAX;
  }

  asm volatile("memw" ::: "memory");

  switch (CurrentMode) {
#ifdef DEF_BLURZ
    case MODE_BLURZ:
      this->visualize_blurz(fastled_helper::leds);
      break;
#endif
#ifdef DEF_FREQWAVE
    case MODE_FREQWAVE:
      this->visualize_freqwave(fastled_helper::leds);
      break;
#endif
#ifdef DEF_GRAV
    case MODE_GRAV:
      this->visualize_gravfreq(fastled_helper::leds);
      break;
#endif
#ifdef DEF_GRAVICENTER
    case MODE_GRAVICENTER:
      this->visualize_gravcenter(fastled_helper::leds);
      break;
#endif
#ifdef DEF_GRAVICENTRIC
    case MODE_GRAVICENTRIC:
      this->visualize_gravcentric(fastled_helper::leds);
      break;
#endif
#ifdef DEF_GRAVIMETER
    case MODE_GRAVIMETER:
      this->visualize_gravmeter(fastled_helper::leds);
      break;
#endif
#ifdef DEF_PIXELS
    case MODE_PIXELS:
      this->visualize_pixels(fastled_helper::leds);
      break;
#endif
#ifdef DEF_JUNGLES
    case MODE_JUNGLES:
      this->visualize_juggles(fastled_helper::leds);
      break;
#endif
#ifdef DEF_MIDNOISE
    case MODE_MIDNOISE:
      this->visualize_midnoise(fastled_helper::leds);
      break;
#endif
#ifdef DEF_RIPPLEPEAK
    case MODE_RIPPLEPEAK:
      this->visualize_ripplepeak(fastled_helper::leds);
      break;
#endif
#ifdef DEF_MATRIPIX
    case MODE_MATRIPIX:
      this->visualize_matripix(fastled_helper::leds);
      break;
#endif
#ifdef DEF_NOISEFIRE
    case MODE_NOISEFIRE:
      this->visualize_noisefire(fastled_helper::leds);
      break;
#endif
#ifdef DEF_NOISEMETER
    case MODE_NOISEMETER:
      this->visualize_noisemeter(fastled_helper::leds);
      break;
#endif
#ifdef DEF_PIXELWAVE
    case MODE_PIXELWAVE:
      this->visualize_pixelwave(fastled_helper::leds);
      break;
#endif
#ifdef DEF_PLASMOID
    case MODE_PLASMOID:
      this->visualize_plasmoid(fastled_helper::leds);
      break;
#endif
#ifdef DEF_PUDDLEPEAK
    case MODE_PUDDLEPEAK:
      this->visualize_puddlepeak(fastled_helper::leds);
      break;
#endif
#ifdef DEF_PUDDLES
    case MODE_PUDDLES:
      this->visualize_puddles(fastled_helper::leds);
      break;
#endif
#ifdef DEF_DJLIGHT
    case MODE_DJLIGHT:
      this->visualize_DJLight(fastled_helper::leds);
      break;
#endif
#ifdef DEF_WATERFALL
    case MODE_WATERFALL:
      this->visualize_waterfall(fastled_helper::leds);
      break;
#endif
  }

  for (int i = 0; i < p_it->size(); i++) {
    (*p_it)[i] = Color(fastled_helper::leds[i].r, fastled_helper::leds[i].g, fastled_helper::leds[i].b);
  }

  this->start_effect_ = false;
  delay_microseconds_safe(1);
}

// *****************************************************************************************************************************************************************
#ifdef DEF_BLURZ
typedef struct Blurz {
  uint32_t counter;
  uint32_t last_execution;
  uint16_t index;
} blurz;

void MusicLeds::visualize_blurz(CRGB *physic_leds)  // Blurz. By Andrew Tuline.
{
  const unsigned dataSize = sizeof(Blurz);
  if (!this->allocateData(dataSize)) {
    return;  // Safe exit if heap allocation fails
  }

  Blurz *blurz = reinterpret_cast<Blurz *>(this->data);
  uint32_t current_time = millis();

  // High-Performance Frame Rate Decay Management
  uint8_t fadeoutDelay = (256 - this->speed) >> 5;  // Replaced division by 32

  blurz->counter++;

  if ((fadeoutDelay <= 1) || ((blurz->counter % fadeoutDelay) == 0)) {
    fastled_helper::fade_out(physic_leds, this->leds_num, this->speed, this->back_color);
  }

  // Time Interval Step Engine (Emulating 60 FPS)
  // Calculate original speed_formula threshold in milliseconds
  uint16_t divisor = std::max<uint16_t>(1U, this->leds_num);
  uint32_t speed_formula = 5U + ((50U * (255U - this->speed)) / divisor);

  // Guard initialization for the first frame execution step
  if (blurz->last_execution == 0) {
    blurz->last_execution = current_time;
  }

  // Check if enough absolute physical milliseconds have elapsed
  if (current_time - blurz->last_execution >= speed_formula) {
    // Increment step mark by calculated delay interval to prevent timing drift
    blurz->last_execution = current_time;

    // Ultra-Fast Hardware Random Spark Placement
    uint16_t segLoc = fastled_helper::hw_random16(this->leds_num);

    // Fetch the 8-bit equalizer array using your context index wrapped via quick bitmask
    uint8_t aux_idx = blurz->index & 0x0F;  // Securely limits value to 0..15 range without modulo math
    uint8_t raw_fft_val = this->features_.fft_result[aux_idx];  // Direct 8-bit GEQ spectrum reading

    uint16_t scaled_fft = static_cast<uint16_t>(raw_fft_val) << 1;  // Double the amplitude via left-shift
    uint8_t blend_weight = (scaled_fft > 255) ? 255 : static_cast<uint8_t>(scaled_fft);

    // Optimized Color Spectrum Interpolation
    uint32_t raw_palette_idx = (static_cast<uint32_t>(scaled_fft) * 240) / divisor;
    uint8_t palette_index = static_cast<uint8_t>(raw_palette_idx & 0xFF);

    CRGB target_palette_color = fastled_helper::color_from_palette(palette_index, this->main_color);

    // Inject Newly Blended Audio Spark using your non-linear 8-bit blender
    physic_leds[segLoc] = fastled_helper::color_blend(this->back_color, target_palette_color, blend_weight);

    // Increment index counter and wrap around safely using bitwise AND instead of slow modulo operator
    blurz->index = (blurz->index + 1) & 0x0F;

    // Directional Temporal Blur Filtering
    fastled_helper::blur(physic_leds, this->leds_num, this->variant);
  }
}  // visualize_blurz()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_FREQWAVE
// Assign a color to the central (starting pixels) based on the predominant frequencies and the volume.
// The color is being determined by mapping the MajorPeak from the FFT and then mapping this to the HSV color circle.
void MusicLeds::visualize_freqwave(CRGB *physic_leds)  // Freqwave. By Andreas Pleschung.
{
  if (this->leds_num == 0 || physic_leds == nullptr)
    return;

  // Compile-time static constants
  static constexpr float FREQ_LOW_LIMIT = 140.0f;    // Bypasses low-end I2S artifacts and room rumble
  static constexpr float FREQ_HIGH_LIMIT = 5120.0f;  // Perfect alignment with your active Nyquist ceiling
  static constexpr float AUDIO_PREAMP = 1.5f;        // Clean, uniform gain coefficient for smoothed volume

  // Pre-calculated delta denominator to eliminate division steps at runtime
  static constexpr float FREQ_DELTA_INV = 1.0f / (FREQ_HIGH_LIMIT - FREQ_LOW_LIMIT);

  uint16_t center_idx = this->leds_num / 2;

  // Safe Hardware Pacing with explicit priority brackets
  uint8_t current_step = (micros() / (256 - this->speed) / 500) % 16;

  // Execute rendering and array shifting strictly on hardware clock ticks
  if (this->store != current_step) {
    this->store = current_step;

    CRGB injection_color = this->back_color;  // Default to background color state on off-beats

    float hz = this->features_.dominant_frequency_hz;

    // Ultra-fast Log-less Frequency Linear Interpolation
    if (hz >= FREQ_LOW_LIMIT && hz <= FREQ_HIGH_LIMIT) {
      // High-performance Brightness Mapping using local constexpr preamp
      float pixVal = static_cast<float>(this->features_.volume_smth()) * (static_cast<float>(this->variant) / 256.0f) *
                     AUDIO_PREAMP;
      uint8_t raw_brightness = static_cast<uint8_t>(std::min(255.0f, pixVal));

      // Apply inverse gamma correction from fastled_helper to bypass ESPHome's active 2.80 squash
      uint8_t brightness = fastled_helper::gamma8inv(raw_brightness);

      // Direct multiplication via pre-calculated inverse delta instead of expensive runtime division
      float mapped_hue = (hz - FREQ_LOW_LIMIT) * FREQ_DELTA_INV * 255.0f;
      uint8_t hue = static_cast<uint8_t>(std::clamp(mapped_hue, 0.0f, 255.0f));

      // Construct native FastLED CRGB from HSV parameters securely
      injection_color = CHSV(hue, 240, brightness);
    }

    // Inject color into the center pixel first
    physic_leds[center_idx] = injection_color;

    // Shift the pixels one pixel outwards
    // Right Side Shift: Move to the left (towards the end of the strip)
    for (uint16_t i = this->leds_num - 1; i > center_idx; i--) {
      physic_leds[i] = physic_leds[i - 1];
    }
    // Left Side Shift: Move to the right (towards the start of the strip)
    for (uint16_t i = 0; i < center_idx; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }
  }
}  // visualize_freqwave()
#endif

// *****************************************************************************************************************************************************************
#if defined(DEF_GRAV) || defined(DEF_GRAVICENTER) || defined(DEF_GRAVICENTRIC) || defined(DEF_GRAVIMETER)

// Gravity struct requited for GRAV* effects
typedef struct Gravity {
  int topLED;
  int gravityCounter;
} gravity;

// Gravcenter effects By Andrew Tuline.
// Gravcenter base function for Gravcenter (0), Gravcentric (1), Gravimeter (2), Gravfreq (3)
void MusicLeds::mode_gravcenter_base(unsigned mode, CRGB *physic_leds) {
  const unsigned dataSize = sizeof(Gravity);
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Gravity *gravcen = reinterpret_cast<Gravity *>(this->data);

  // Dynamic frame fading (peak tails)
  uint8_t fade_val = 251;  // Default for mode 0 (Gravcenter)
  if (mode == 1)
    fade_val = 253;  // Gravcentric
  else if (mode == 2)
    fade_val = 249;  // Gravimeter
  else if (mode == 3)
    fade_val = 250;  // Gravfreq

  fastled_helper::fade_out(physic_leds, this->leds_num, fade_val, this->back_color);

  // Calculate smoothed volume via new DSP features API
  float mySampleAvg;
  uint16_t tempsamp;
  float segmentSampleAvg = (float) this->features_.volume_smth() * (float) this->variant / 255.0f;

  if (mode == 2) {              // Gravimeter
    segmentSampleAvg *= 0.25f;  // Compensate for later sensitivity upscaling
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 64.0f, 0.0f, (float) (this->leds_num - 1));
    tempsamp = constrain((uint16_t) mySampleAvg, 0, this->leds_num - 1);
  } else {                       // Gravcenter, Gravcentric, Gravfreq
    segmentSampleAvg *= 0.125f;  // Compensate for later sensitivity upscaling
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 32.0f, 0.0f, (float) this->leds_num / 2.0f);
    tempsamp = constrain((uint16_t) mySampleAvg, 0, this->leds_num / 2);
  }

  // Gravity tracker calculation
  uint8_t gravity = 8 - this->speed / 32;
  if (gravity == 0)
    gravity = 1;  // Division by zero protection

  uint8_t offset = (mode == 2) ? 0 : 1;
  if (tempsamp >= gravcen->topLED) {
    gravcen->topLED = tempsamp - offset;
  } else if (gravcen->gravityCounter % gravity == 0) {
    gravcen->topLED--;
  }

  uint32_t ms = millis();

  // Render modes
  if (mode == 1) {  // Gravcentric
    uint8_t index = (uint8_t) (segmentSampleAvg * 24.0f) + (uint8_t) (ms / 200);
    CRGB color = fastled_helper::color_from_palette(index, this->main_color);

    for (uint16_t i = 0; i < tempsamp; i++) {
      physic_leds[i + this->leds_num / 2] = color;
      physic_leds[this->leds_num / 2 - 1 - i] = color;
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }

  } else if (mode == 2) {  // Gravimeter
    uint8_t blend_weight = constrain((int) (segmentSampleAvg * 8.0f), 0, 255);
    for (uint16_t i = 0; i < tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i * segmentSampleAvg + ms, 5000 + i * segmentSampleAvg);
      CRGB target_color = fastled_helper::color_from_palette(index, this->main_color);
      physic_leds[i] = fastled_helper::color_blend(this->back_color, target_color, blend_weight);
    }
    if (gravcen->topLED > 0 && gravcen->topLED < this->leds_num) {
      physic_leds[gravcen->topLED] = fastled_helper::color_from_palette(ms, this->main_color);
    }

  } else if (mode == 3) {  // Gravfreq (Frequency mapping zone)
    float hz = this->features_.dominant_frequency_hz;
    if (hz < 20.0f)
      hz = 20.0f;  // Cut off infrasound/silence to protect log10f

    // Calculate Nyquist limit based on active sample rate
    float max_frequency = this->sample_rate_ / 2.0f;
    float max_freq_log10 = std::log10(max_frequency);

    // Align the logarithmic window dynamically
    float raw_index = (std::log10(hz) - (max_freq_log10 - 1.78f)) * 255.0f;

    // Securely clamp to 8-bit color space bounds to prevent overflow
    uint8_t index = static_cast<uint8_t>(constrain(static_cast<int>(raw_index), 0, 255));
    CRGB color = fastled_helper::color_from_palette(index, this->main_color);

    for (uint16_t i = 0; i < tempsamp; i++) {
      physic_leds[i + this->leds_num / 2] = color;
      physic_leds[this->leds_num / 2 - i - 1] = color;
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }

  } else {  // Gravcenter (mode == 0)
    uint8_t blend_weight = constrain((int) (segmentSampleAvg * 8.0f), 0, 255);
    for (uint16_t i = 0; i < tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i * segmentSampleAvg + ms, 5000 + i * segmentSampleAvg);
      CRGB target_color = fastled_helper::color_from_palette(index, this->main_color);
      CRGB blended = fastled_helper::color_blend(this->back_color, target_color, blend_weight);
      physic_leds[i + this->leds_num / 2] = blended;
      physic_leds[this->leds_num / 2 - i - 1] = blended;
    }
    if (gravcen->topLED >= 0) {
      CRGB peak_color = fastled_helper::color_from_palette(ms, this->main_color);
      physic_leds[gravcen->topLED + this->leds_num / 2] = peak_color;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = peak_color;
    }
  }

  gravcen->gravityCounter = (gravcen->gravityCounter + 1) % gravity;
}
#endif

#ifdef DEF_GRAVICENTER
void MusicLeds::visualize_gravcenter(CRGB *physic_leds)  // Gravcenter. By Andrew Tuline.
{
  mode_gravcenter_base(0, physic_leds);
}  // visualize_gravcenter()
#endif

#ifdef DEF_GRAVICENTRIC
void MusicLeds::visualize_gravcentric(CRGB *physic_leds)  // Gravcentric. By Andrew Tuline.
{
  mode_gravcenter_base(1, physic_leds);
}  // visualize_gravcentric
#endif

#ifdef DEF_GRAVIMETER
void MusicLeds::visualize_gravmeter(CRGB *physic_leds)  // Gravmeter. By Andrew Tuline.
{
  mode_gravcenter_base(2, physic_leds);
}  // visualize_gravcentric
#endif

#ifdef DEF_GRAV
void MusicLeds::visualize_gravfreq(CRGB *physic_leds)  // Gravfreq. By Andrew Tuline.
{
  return mode_gravcenter_base(3, physic_leds);
}  // visualize_gravfreq
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_PIXELS
void MusicLeds::visualize_pixels(CRGB *physic_leds)  // Pixels. By Andrew Tuline.
{
  const unsigned dataSize = 32 * sizeof(uint8_t);
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  uint8_t *myVals = reinterpret_cast<uint8_t *>(this->data);

  // Get current synchronized volume features
  uint8_t current_volume = this->features_.volume_smth();

  // Pseudo-random circular buffer filing based on time metrics
  uint32_t ms = millis();
  myVals[ms % 32] = current_volume;

  // Apply responsive fade-out based on speed parameter
  fastled_helper::fade_out(physic_leds, this->leds_num, 64 + (this->speed >> 1), this->back_color);

  // Render loop using variant parameter as density control
  int loop_limit = (int) this->variant / 8;
  for (int i = 0; i < loop_limit; i++) {
    uint16_t segLoc = fastled_helper::hw_random16(this->leds_num);

    // Pick color index from circular history buffer
    uint8_t palette_index = myVals[i % 32] + (i * 4);
    CRGB target_color = fastled_helper::color_from_palette(palette_index, this->main_color);

    // Blend background and palette color using current volume as intensity weight
    physic_leds[segLoc] = fastled_helper::color_blend(this->back_color, target_color, current_volume);
  }
}  // visualize_pixels()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_JUNGLES
void MusicLeds::visualize_juggles(CRGB *physic_leds)  // Juggles. By Andrew Tuline.
{
  // Apply uniform tail fading for the juggling points
  fastled_helper::fade_out(physic_leds, this->leds_num, 224, this->back_color);

  // Extract clamped audio volume intensity from DSP engine features
  uint8_t dynamic_volume = this->features_.volume_smth();

  // Calculate total number of synchronized moving dots based on variant slider
  int dots_count = ((int) this->variant / 32) + 1;
  uint32_t ms = millis();

  for (int i = 0; i < dots_count; i++) {
    // Generate sine-wave coordinate projection for current pixel index
    uint16_t speed_param = (int) this->speed / 4 + (i * 2);
    uint16_t led_index = fastled_helper::beatsin16(speed_param, 0, this->leds_num - 1);

    // Pick shifting color index from selected palette mapping
    uint8_t palette_index = (uint8_t) (ms / 4) + (i * 2);
    CRGB target_color = fastled_helper::color_from_palette(palette_index, this->main_color);

    // Render modulated point with reactive scaling by current sound volume
    physic_leds[led_index] = fastled_helper::color_blend(this->back_color, target_color, dynamic_volume);
  }
}  // visualize_juggles()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_MIDNOISE
void MusicLeds::visualize_midnoise(CRGB *physic_leds)  // Midnoise. By Andrew Tuline.
{
  static int noise_x = 0;
  static int noise_y = 0;

  // Calculate safe responsive frame decay factor
  int fade_factor = ((int) this->speed * (int) this->speed) / 255;
  if (fade_factor < 1)
    fade_factor = 1;  // Zero protection to prevent freeze bugs in fade_out

  fastled_helper::fade_out(physic_leds, this->leds_num, fade_factor, this->back_color);

  // Process math scaling using modern float-based features API
  float audio_vol = (float) this->features_.volume_smth();

  // Re-scale sensitivity chains based on original variant slider intent
  float dynamic_scale = audio_vol * ((float) this->variant / 256.0f);
  dynamic_scale *= ((float) this->variant / 128.0f);

  // Remap audio response directly into LED center-out boundaries
  float half_leds = (float) this->leds_num / 2.0f;
  int max_len = (int) remap(dynamic_scale, 0.0f, 127.0f, 0.0f, half_leds);
  if (max_len > (int) half_leds) {
    max_len = (int) half_leds;
  }

  // Render 1D Perlin noise projection mirrored around physical center
  int start_bound = (this->leds_num / 2) - max_len;
  int end_bound = (this->leds_num / 2) + max_len;

  for (int i = start_bound; i < end_bound; i++) {
    // Generate Perlin coordinates modulated by position and immediate signal amplitude
    uint8_t noise_idx = fastled_helper::perlin8(i * audio_vol + noise_x, noise_y + i * audio_vol);
    physic_leds[i] = fastled_helper::color_from_palette(noise_idx, this->main_color);
  }

  // Update coordinate shifts using wrapped fastled helper timers
  noise_x += fastled_helper::beatsin8(5, 0, 10);
  noise_y += fastled_helper::beatsin8(4, 0, 10);
}  // visualize_midnoise()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_RIPPLEPEAK
typedef struct Ripple {
  uint8_t state;
  uint8_t color;
  uint16_t pos;
} ripple;

void MusicLeds::visualize_ripplepeak(CRGB *physic_leds)  // Ripple peak. By Andrew Tuline.
{
  constexpr uint8_t max_steps = 16;
  constexpr unsigned max_ripples = 16;
  const unsigned dataSize = sizeof(Ripple) * max_ripples;

  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Ripple *ripples = reinterpret_cast<Ripple *>(this->data);

  // Apply uniform responsive tail fading
  uint8_t dynamic_fade = map(this->features_.volume_raw(), 0, 255, 230, 200);
  fastled_helper::fade_out(physic_leds, this->leds_num, dynamic_fade, this->back_color);

  // Read the 50ms latched peak token to prevent missing fast triggers on Core 0
  bool is_peak_triggered = this->features_.sample_peak;

  // Variant slider controls the density of concurrent ripples allowed
  int max_active_ripples = map(this->variant, 0, 255, 1, max_ripples);

  for (int i = 0; i < max_active_ripples; i++) {
    if (is_peak_triggered) {
      if (ripples[i].state == 254) {
        ripples[i].state = 255;
      }
    }

    switch (ripples[i].state) {
      case 254:  // Idle mode
        break;

      case 255:  // Initialize ripple configuration on peak onset
        ripples[i].pos = fastled_helper::hw_random16(this->leds_num);

        // Translate absolute frequency [20Hz .. 11025Hz] to palette index [0 .. 255]
        {
          float hz = this->features_.dominant_frequency_hz;
          if (hz > 1.0f) {
            ripples[i].color = static_cast<uint8_t>(log10f(hz) * 128.0f);
          } else {
            ripples[i].color = 0;
          }
        }
        ripples[i].state = 0;
        break;

      case 0:  // Render origin point
        physic_leds[ripples[i].pos] = fastled_helper::color_from_palette(ripples[i].color, this->main_color);
        ripples[i].state++;
        break;

      case max_steps:  // Terminal execution state reached
        ripples[i].state = 254;
        break;

      default:  // Propagate wave outwards in both directions
      {
        CRGB wave_color = fastled_helper::color_from_palette(ripples[i].color, this->main_color);
        uint8_t blend_weight = (2 * 255) / ripples[i].state;

        int forward_idx = (ripples[i].pos + ripples[i].state + this->leds_num) % this->leds_num;
        int backward_idx = (ripples[i].pos - ripples[i].state + this->leds_num) % this->leds_num;

        physic_leds[forward_idx] = fastled_helper::color_blend(this->back_color, wave_color, blend_weight);
        physic_leds[backward_idx] = fastled_helper::color_blend(this->back_color, wave_color, blend_weight);

        ripples[i].state++;  // Advance propagation step
      } break;
    }  // switch step
  }  // for i
}  // visualize_ripplepeak()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_MATRIPIX
void MusicLeds::visualize_matripix(CRGB *physic_leds)  // Matripix. By Andrew Tuline.
{
  uint8_t secondHand = (micros() / (256 - this->speed) / 500) % 16;

#ifdef DEBUG
  if (esphome::music_leds::debug::should_log()) {
    uint8_t current_volume = this->features_.volume_raw();
    bool has_peak = this->features_.sample_peak;

    ESP_LOGD("Matripix", "SecondHand: %d, Store: %d, Speed: %d, Variant: %d, VolRaw: %d, Peak: %s", secondHand,
             this->store, (int) this->speed, (int) this->variant, current_volume, has_peak ? "YES" : "NO");
  }
#endif

  if (this->store != secondHand) {
    this->store = secondHand;

    uint16_t pixBri = 0;

    // Hybrid onset matrix injection
    if (this->features_.sample_peak && fastled_helper::hw_random8() > 192) {
      // Peak Event: Inject absolute maximum 8-bit brightness to forcefully stamp
      // a solid high-contrast matrix block directly onto the musical transient
      pixBri = 255;
    } else {
      // Off-Beat State: Scale native 8-bit integer volume safely across the palette space.
      uint16_t calculated_weight = ((uint16_t) this->features_.volume_raw() * (uint16_t) this->variant) >> 6;
      pixBri = (calculated_weight > 255) ? 255 : calculated_weight;
    }

    size_t k = this->leds_num - 1;
    for (size_t i = 0; i < k; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }

    CRGB new_color = fastled_helper::color_from_palette(millis(), this->main_color);
    physic_leds[k] = fastled_helper::color_blend(this->back_color, new_color, pixBri);

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("Matripix", "New Color: %d, %d, %d Pixels Color: %d, %d, %d", new_color.r, new_color.g, new_color.b,
               physic_leds[k].r, physic_leds[k].g, physic_leds[k].b);
    }
#endif
  }
}  // visualize_matripix()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_NOISEFIRE
// I am the god of hellfire. . . Volume (only) reactive fire routine. Oh, look how short this is.
void MusicLeds::visualize_noisefire(CRGB *physic_leds)  // Noisefire. By Andrew Tuline.
{
  // Fire palette definition. Lower value = darker.
  CRGBPalette16 myPal =
      CRGBPalette16(CHSV(0, 255, 2), CHSV(0, 255, 4), CHSV(0, 255, 8), CHSV(0, 255, 8), CHSV(0, 255, 16), CRGB::Red,
                    CRGB::Red, CRGB::Red, CRGB::DarkOrange, CRGB::DarkOrange, CRGB::Orange, CRGB::Orange, CRGB::Yellow,
                    CRGB::Orange, CRGB::Yellow, CRGB::Yellow);

  // Extract modern synchronized volume and apply scaling for maximum brightness driving
  uint16_t dynamic_brightness = (uint16_t) this->features_.volume_smth() * 2;
  if (dynamic_brightness > 255) {
    dynamic_brightness = 255;
  }

  uint32_t ms = millis();

  // Prevent arithmetic overflow during high-speed/long-strand timeline multiplications
  uint32_t variant_divisor = 256 - this->variant;
  uint32_t speed_factor = (this->speed < 1) ? 1 : this->speed;
  uint32_t y_time_axis = (ms * speed_factor / 64) * this->leds_num / 255;

  for (unsigned i = 0; i < this->leds_num; i++) {
    // X location is constant, but we move along the Y at the rate of millis(). By Andrew Tuline.
    uint8_t index = fastled_helper::perlin8(i * this->speed / 64, y_time_axis);

    // Now we need to scale index so that it gets blacker as we get close to one of the ends.
    // This is a simple y=mx+b equation that's been scaled. index/128 is another scaling.
    uint32_t spatial_scale = 255 - (i * 256 / this->leds_num);
    index = (spatial_scale * index) / variant_divisor;

    // Render using abstract helper wrapper to bridge FastLED and ESPHome environment
    physic_leds[i] = fastled_helper::color_from_palette(myPal, index, (uint8_t) dynamic_brightness, LINEARBLEND);
  }
}  // visualize_noisefire()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_NOISEMETER
typedef struct Meters {
  uint16_t noise_x;
  uint16_t noise_y;
} meters;

void MusicLeds::visualize_noisemeter(CRGB *physic_leds)  // Noisemeter. By Andrew Tuline.
{
  const unsigned dataSize = sizeof(Meters);
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Meters *meters = reinterpret_cast<Meters *>(this->data);

  // 1. Dynamic Tail Fading using mapped speed parameter
  // Scales fade rates from 200 (fast fade) to 254 (long trails) via fixed-point math
  uint8_t fadeRate = 200 + ((static_cast<uint16_t>(this->speed) * 54) >> 8);
  fastled_helper::fade_out(physic_leds, this->leds_num, fadeRate, this->back_color);

  // 2. Soundbar Length Calculation (Pure integer pipeline, no float math)
  uint8_t volumeRaw = this->features_.volume_raw();    // Range: 0..255
  uint8_t volumeSmth = this->features_.volume_smth();  // Range: 0..255

  // Calculate high-gain volume scale: volumeRaw * 2 * intensity
  uint32_t tmpSound2 = (static_cast<uint32_t>(volumeRaw) * 2 * this->variant) >> 8;

  // Directly map the sound wave height to physical pixel length limits [0 .. leds_num]
  uint32_t calculated_len = (tmpSound2 * this->leds_num) >> 8;
  uint16_t maxLen = static_cast<uint16_t>(std::min(calculated_len, static_cast<uint32_t>(this->leds_num)));

  // Fill Soundbar length with Perlin Noise mapped from palette
  for (uint16_t i = 0; i < maxLen; i++) {
    // Generate organic 2D vector coordinate mapping using integer scaling instead of floats
    // volumeSmth (0..255) acts as a dynamic spatial scaling factor for noise density
    uint16_t dynamic_scale = (i * volumeSmth) >> 4;  // Shifted right to prevent noise from becoming too chaotic
    uint16_t noise_x = dynamic_scale + meters->noise_x;
    uint16_t noise_y = dynamic_scale + meters->noise_y;

    // Call FastLED native inlined 8-bit Perlin noise generator
    uint8_t noise_index = fastled_helper::perlin8(noise_x, noise_y);

    // Write mapped pixel color vector directly onto the physical strip array
    physic_leds[i] = fastled_helper::color_from_palette(noise_index, this->main_color);
  }

  // Increment internal registers to evolve noise coordinates over time
  meters->noise_x += fastled_helper::beatsin8(5, 0, 10);
  meters->noise_y += fastled_helper::beatsin8(4, 0, 10);
}  // visualize_noisemeter()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_PIXELWAVE
void MusicLeds::visualize_pixelwave(CRGB *physic_leds)  // Pixelwave. By Andrew Tuline.
{
  uint8_t secondHand = (micros() / (256 - this->speed) / 500) % 16;

  // Perform wave propagation when execution time-frame steps advance
  if (this->store != secondHand) {
    this->store = secondHand;

    uint16_t pixBri = 0;

    // Dual-mode center injection
    if (this->features_.sample_peak && fastled_helper::hw_random8() > 192) {
      // Peak Event: Enforce absolute maximum brightness to forcefully inject
      // a crisp high-contrast shockwave directly on the musical transient
      pixBri = 255;
    } else {
      // Off-Beat State: Scale native 8-bit integer volume safely across the palette space.
      uint16_t calculated_weight = ((uint16_t) this->features_.volume_raw() * (uint16_t) this->variant) >> 6;
      pixBri = (calculated_weight > 255) ? 255 : calculated_weight;
    }

    // Propagate pixels outward from the physical center to both ends
    unsigned center_idx = this->leds_num / 2;

    // Shift right half outwards (from center toward the end of the strand)
    for (unsigned i = this->leds_num - 1; i > center_idx; i--) {
      physic_leds[i] = physic_leds[i - 1];
    }

    // Shift left half outwards (from center toward the start of the strand)
    // Fixed iteration order to prevent pixel data destruction/cloning loops
    for (unsigned i = 0; i < center_idx; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }

    // Inject new audio-modulated pixel at the center boundary anchor
    uint32_t ms = millis();
    CRGB new_color = fastled_helper::color_from_palette((uint8_t) ms, this->main_color);

    physic_leds[center_idx] = fastled_helper::color_blend(this->back_color, new_color, (uint8_t) pixBri);
  }
}  // visualize_pixelwave()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_PLASMOID
typedef struct Plasphase {
  int16_t thisphase;
  int16_t thatphase;
} plasphase;

void MusicLeds::visualize_plasmoid(CRGB *physic_leds)  // Plasmoid. By Andrew Tuline.
{
  const unsigned dataSize = sizeof(Plasphase);
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Plasphase *plasmoip = reinterpret_cast<Plasphase *>(this->data);

  // Apply heavy uniform frame decay for the plasma fluid effect
  fastled_helper::fade_out(physic_leds, this->leds_num, 32, this->back_color);

  // Update synchronized phase offsets via abstracted fastled timers
  plasmoip->thisphase += fastled_helper::beatsin8(6, -4, 4);
  plasmoip->thatphase += fastled_helper::beatsin8(7, -4, 4);

  // Extract modern audio volume parameter from the DSP features container
  uint16_t audio_threshold = ((uint16_t) this->features_.volume_smth() * (uint16_t) this->variant) >> 6;

  for (unsigned i = 0; i < this->leds_num; i++) {
    // Generate complex wave projection pattern using mathematical helper wraps
    uint16_t speed_factor_1 = 1 + (3 * (int) this->speed / 32);
    uint8_t wave_arg_1 = ((i * speed_factor_1) + plasmoip->thisphase) & 0xFF;
    uint8_t thisbright = cubicwave8(wave_arg_1) / 2;

    uint16_t speed_factor_2 = 97 + (5 * (int) this->speed / 32);
    uint8_t wave_arg_2 = ((i * speed_factor_2) + plasmoip->thatphase) & 0xFF;
    thisbright += fastled_helper::cos8_t(wave_arg_2) / 2;

    uint8_t colorIndex = thisbright;

    // Dynamic noise gate cutoff: clamp pixel brightness if sound energy is insufficient
    if (audio_threshold < (uint16_t) thisbright) {
      thisbright = 0;
    }

    // Render resulting plasmoid stream with blended color states
    physic_leds[i] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(colorIndex, this->main_color), thisbright);
  }
}  // visualize_plasmoid()
#endif

// *****************************************************************************************************************************************************************
#if defined(DEF_PUDDLES) || defined(DEF_PUDDLEPEAK)
// Puddles / Puddlepeak By Andrew Tuline.
void MusicLeds::puddles_base(CRGB *physic_leds, bool peakdetect) {
  unsigned size = 0;

  // Calculate safe tail fading factor using explicit remap wrapper
  uint8_t fadeVal = (uint8_t) remap((float) this->speed, 0.0f, 255.0f, 224.0f, 254.0f);
  fastled_helper::fade_out(physic_leds, this->leds_num, fadeVal, this->back_color);

  // Process responsive audio triggers and calculate dynamic pool sizes
  if (peakdetect) {  // Puddles peak mode
    // Utilize the latched peak
    if (this->features_.sample_peak) {
      uint16_t smoothed_vol = (uint16_t) this->features_.volume_smth();
      size = (smoothed_vol * (uint16_t) this->variant) / 256 / 4 + 1;
    }
  } else {  // Standard Puddles mode
    // Utilize raw immediate byte volume from the modern DSP features container
    uint8_t raw_vol = this->features_.volume_raw();
    if (raw_vol > 1) {
      size = ((uint16_t) raw_vol * (uint16_t) this->variant) / 256 / 8 + 1;
    }
  }

  // Render and clip flash block execution layout if sound energy is present
  if (size > 0) {
    // Lazy initialization of position bounds to preserve hardware RNG entropy
    unsigned pos = fastled_helper::hw_random16(this->leds_num);

    // Hard boundary clipping constraint to prevent out-of-bounds array overflows
    if (pos + size >= this->leds_num) {
      size = this->leds_num - pos;
    }

    uint32_t ms = millis();
    for (unsigned i = 0; i < size; i++) {
      physic_leds[pos + i] = fastled_helper::color_from_palette((uint8_t) ms, this->main_color);
    }
  }
}  // puddles_base()
#endif

#ifdef DEF_PUDDLEPEAK
void MusicLeds::visualize_puddlepeak(CRGB *physic_leds)  // Puddlepeak. By Andrew Tuline.
{
  puddles_base(physic_leds, true);
}  // visualize_puddlepeak()
#endif

#ifdef DEF_PUDDLES
void MusicLeds::visualize_puddles(CRGB *physic_leds)  // Puddles. By Andrew Tuline.
{
  puddles_base(physic_leds, false);
}  // visualize_puddles()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_DJLIGHT
void MusicLeds::visualize_DJLight(CRGB *physic_leds)  // DJLight. Written by ??? Adapted by Will Tatam.
{
  // No need to prevent from executing on single led strips, only mid will be set (mid = 0)
  const uint16_t mid = this->leds_num / 2;

  // Enforce rigid bounded wrapping steps matching the original modulo 64 cycle clock
  uint8_t secondHand = (micros() / (256 - this->speed) / 500) % 64;

#ifdef DEBUG
  if (esphome::music_leds::debug::should_log()) {
    // Capture state variables exactly as they are evaluated in the processing loop
    uint8_t bin_0 = this->features_.fft_result[0];
    uint8_t bin_5 = this->features_.fft_result[5];
    uint8_t bin_15 = this->features_.fft_result[15];
    uint8_t bin_4 = this->features_.fft_result[4];
    bool has_peak = this->features_.sample_peak;

    // Explicitly mirror the mapping math to track fade suppression behavior
    uint8_t current_fade_weight = (uint8_t) remap((float) bin_4, 0.0f, 255.0f, 255.0f, 5.0f);

    ESP_LOGD("DJLight", "SecondHand: %d Store: %d Speed: %d Bins[0,5,15]: (%d, %d, %d) Bin4: %d FadeW: %d Peak: %s",
             secondHand, this->store, (int) this->speed, bin_0, bin_5, bin_15, bin_4, current_fade_weight,
             has_peak ? "YES" : "NO");
  }
#endif

  // Perform frequency wave propagation when execution time-frame steps advance
  if (this->store != secondHand) {
    this->store = secondHand;

    // Map 16-band graphic equalizer internal registers to RGB components with Gamma Inversion
    // Technical description: High frequencies (band 15) -> Red, Mid frequencies (band 5) -> Green, Bass (band 0) ->
    // Blue Replicates exact new layout behavior by applying inverse gamma evaluation to raw bytes via global helper
    uint8_t r_comp = fastled_helper::gamma8inv(this->features_.fft_result[15] / 2);
    uint8_t g_comp = fastled_helper::gamma8inv(this->features_.fft_result[5] / 2);
    uint8_t b_comp = fastled_helper::gamma8inv(this->features_.fft_result[0] / 2);
    CRGB color = CRGB(r_comp, g_comp, b_comp);

    // Hybrid transient override mode
    if (this->features_.sample_peak && fastled_helper::hw_random8() > 192) {
      // Peak Event: Enforce absolute maximum brightness to eliminate early track start delays
      // and forcefully inject a crisp high-contrast wave directly on the music beat
      physic_leds[mid] = color;
    } else {
      // Off-Beat State: Gracefully fall back to the native analog behavior where
      // the fade intensity is driven dynamically by the low-mid band energy (band 4)
      uint8_t fade_control = this->features_.fft_result[4];
      uint8_t fade_weight = (uint8_t) remap((float) fade_control, 0.0f, 255.0f, 255.0f, 4.0f);
      physic_leds[mid] = color.fadeToBlackBy(fade_weight);
    }

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("DJLight", "Color: %d, %d, %d Pixels Color(mid): %d, %d, %d", color.r, color.g, color.b,
               physic_leds[mid].r, physic_leds[mid].g, physic_leds[mid].b);
    }
#endif

    // Shift right half outwards (from center toward the end of the strand)
    for (uint16_t i = this->leds_num - 1; i > mid; i--) {
      physic_leds[i] = physic_leds[i - 1];
    }

    // Shift left half outwards (from center toward the start of the strand)
    for (uint16_t i = 0; i < mid; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }
  }
}  // visualize_DJLight()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_WATERFALL
// Combines peak detection with dominant frequency tracking and normalized magnitude.
void MusicLeds::visualize_waterfall(CRGB *physic_leds)  // Waterfall. By: Andrew Tuline
{
  uint8_t secondHand = (micros() / (256 - this->speed) / 500) % 16;

#ifdef DEBUG
  if (esphome::music_leds::debug::should_log()) {
    float current_hz = this->features_.dominant_frequency_hz;
    float current_mag = this->features_.magnitude / 8;
    bool has_peak = this->features_.sample_peak;

    ESP_LOGD("Waterfall", "SecondHand: %d, Store: %d, Speed: %d, Variant: %d, Hz: %.1f, mag: %.2f, peak: %s",
             secondHand, this->store, (int) this->speed, (int) this->variant, current_hz, current_mag,
             has_peak ? "YES" : "NO");
  }
#endif

  if (this->store != secondHand) {
    this->store = secondHand;

    float hz = this->features_.dominant_frequency_hz;

    // Protect FPU from log10f(0) math exception if pitch detection drops to zero
    if (hz < 1.0f)
      hz = 1.0f;

    // Calculate Nyquist limit based on active sample rate
    float max_nyquist_hz = this->sample_rate_ / 2.0f;

    // Calculate the maximum possible log scale delta for the current hardware configuration
    float max_log_delta = log10f(max_nyquist_hz) - 2.26f;

    // Guard to prevent division by zero or negative scales on ultra-low sample rates
    if (max_log_delta < 0.01f)
      max_log_delta = 0.01f;

    // Dynamically scale the multiplier so the current hardware ceiling maps perfectly to 255
    float dynamic_multiplier = 255.0f / max_log_delta;

    // Calculate palette index using the dynamically calibrated multiplier
    int32_t calculated_col = (log10f(hz) - 2.26f) * dynamic_multiplier;

    // Safely clamp the final index within strict 8-bit unsigned boundaries [0..255]
    uint8_t pixCol = std::clamp<int32_t>(calculated_col, 0, 255);

    // Handle bass frequency underflow exactly like the original acoustic design specifies
    if (hz < 182.0f) {
      pixCol = 0;  // Sub-bass notes anchor strictly to the first palette token (pure red)
    }

    unsigned k = this->leds_num - 1;

    if (this->features_.sample_peak) {
      physic_leds[k] = CHSV(92, 92, fastled_helper::gamma8inv(92));
    } else {
      float mag = this->features_.magnitude / 8.0f;
      uint16_t blend_weight = (mag > 255.0f) ? 255 : static_cast<uint16_t>(mag);

      CRGB target_color = fastled_helper::color_from_palette(pixCol + this->variant, this->main_color);
      physic_leds[k] = fastled_helper::color_blend(this->back_color, target_color, blend_weight);
    }

#ifdef DEBUG
    if (esphome::music_leds::debug::should_log()) {
      ESP_LOGD("Waterfall", "Pix Color: %d Pixels Color: %d, %d, %d", pixCol, physic_leds[k].r, physic_leds[k].g,
               physic_leds[k].b);
    }
#endif

    for (size_t i = 0; i < k; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }
  }
}  // visualize_waterfall()
#endif

}  // namespace esphome::music_leds
