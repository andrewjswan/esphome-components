#include "music_leds.h"

#include "esphome/components/fastled_helper/utils.h"
#include "esphome/core/defines.h"

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
void MusicLeds::ShowFrame(PLAYMODE CurrentMode, esphome::Color current_color, light::AddressableLight *p_it) {
  if (!this->is_running() && !this->microphone_is_running()) {
    return;
  }

  fastled_helper::InitLeds(p_it->size());

  this->leds_num = p_it->size();

  this->main_color = CRGB(current_color.r, current_color.g, current_color.b);
  if ((int) fastled_helper::current_palette == 0) {
    // 5% from main color
    this->back_color = CRGB(current_color.r / 100 * 5, current_color.g / 100 * 5, current_color.b / 100 * 5);
  } else {
    this->back_color = CRGB::Black;
  }

  asm volatile("memw" ::: "memory");

  switch (CurrentMode) {
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
#if defined(DEF_GRAV) || defined(DEF_GRAVICENTER) || defined(DEF_GRAVICENTRIC) || defined(DEF_GRAVIMETER)

#define MAX_FREQUENCY 11025      // sample frequency / 2 (as per Nyquist criterion)
#define MAX_FREQ_LOG10 4.04238f  // log10(MAX_FREQUENCY)

// Gravity struct requited for GRAV* effects
typedef struct Gravity {
  int topLED;
  int gravityCounter;
} gravity;

// Gravcenter effects By Andrew Tuline.
// Gravcenter base function for Gravcenter (0), Gravcentric (1), Gravimeter (2), Gravfreq (3)
void MusicLeds::mode_gravcenter_base(unsigned mode, CRGB *physic_leds) {
  const unsigned dataSize = sizeof(gravity);
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Gravity *gravcen = reinterpret_cast<Gravity *>(this->data);

  if (mode == 1) {  // Gravcentric
    fastled_helper::fade_out(physic_leds, this->leds_num, 253, this->back_color);
  } else if (mode == 2) {  // Gravimeter
    fastled_helper::fade_out(physic_leds, this->leds_num, 249, this->back_color);
  } else if (mode == 3) {  // Gravfreq
    fastled_helper::fade_out(physic_leds, this->leds_num, 250, this->back_color);
  } else {  // Gravcenter
    fastled_helper::fade_out(physic_leds, this->leds_num, 251, this->back_color);
  }

  float mySampleAvg;
  int tempsamp;
  float segmentSampleAvg = this->features_.smoothed_volume * static_cast<float>(this->variant);

  if (mode == 2) {             // Gravimeter
    segmentSampleAvg *= 0.25;  // divide by 4, to compensate for later "sensitivity" upscaling
    // map to pixels availeable in current segment
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 64.0f, 0.0f, (float) (this->leds_num - 1));
    tempsamp = constrain(mySampleAvg, 0, this->leds_num - 1);  // Keep the sample from overflowing.
  } else {                                                     // Gravcenter or Gravcentric or Gravfreq
    segmentSampleAvg *= 0.125f;  // divide by 8, to compensate for later "sensitivity" upscaling
    // map to pixels availeable in current segment
    mySampleAvg = remap(segmentSampleAvg * 2.0f, 0.0f, 32.0f, 0.0f, (float) this->leds_num / 2.0f);
    tempsamp = constrain(mySampleAvg, 0, this->leds_num / 2);  // Keep the sample from overflowing.
  }

  uint8_t gravity = 8 - this->speed / 32;
  int offset = (mode == 2) ? 0 : 1;
  if (tempsamp >= gravcen->topLED)
    gravcen->topLED = tempsamp - offset;
  else if (gravcen->gravityCounter % gravity == 0)
    gravcen->topLED--;

  if (mode == 1) {  // Gravcentric
    for (int i = 0; i < tempsamp; i++) {
      uint8_t index = segmentSampleAvg * 24 + millis() / 200;
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_from_palette(index, this->main_color);
      physic_leds[this->leds_num / 2 - 1 - i] = fastled_helper::color_from_palette(index, this->main_color);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }
  } else if (mode == 2) {  // Gravimeter
    for (int i = 0; i < tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i * segmentSampleAvg + millis(), 5000 + i * segmentSampleAvg);
      physic_leds[i] = fastled_helper::color_blend(
          this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
    }
    if (gravcen->topLED > 0) {
      physic_leds[gravcen->topLED] = fastled_helper::color_from_palette(millis(), this->main_color);
    }
  } else if (mode == 3) {  // Gravfreq
    for (int i = 0; i < tempsamp; i++) {
      float fft_MajorPeak = this->features_.dominant_frequency_hz;  // used in mode 3: Gravfreq
      if (fft_MajorPeak < 1.0f) {
        fft_MajorPeak = 1.0f;
      }
      uint8_t index = (log10f(fft_MajorPeak) - (MAX_FREQ_LOG10 - 1.78f)) * 255;
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_from_palette(index, this->main_color);
      physic_leds[this->leds_num / 2 - i - 1] = fastled_helper::color_from_palette(index, this->main_color);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] = CRGB::Gray;
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] = CRGB::Gray;
    }
  } else {  // Gravcenter
    for (int i = 0; i < tempsamp; i++) {
      uint8_t index = fastled_helper::perlin8(i * segmentSampleAvg + millis(), 5000 + i * segmentSampleAvg);
      physic_leds[i + this->leds_num / 2] = fastled_helper::color_blend(
          this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
      physic_leds[this->leds_num / 2 - i - 1] = fastled_helper::color_blend(
          this->back_color, fastled_helper::color_from_palette(index, this->main_color), segmentSampleAvg * 8);
    }
    if (gravcen->topLED >= 0) {
      physic_leds[gravcen->topLED + this->leds_num / 2] =
          fastled_helper::color_from_palette(millis(), this->main_color);
      physic_leds[this->leds_num / 2 - 1 - gravcen->topLED] =
          fastled_helper::color_from_palette(millis(), this->main_color);
    }
  }
  gravcen->gravityCounter = (gravcen->gravityCounter + 1) % gravity;
}
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTER
void MusicLeds::visualize_gravcenter(CRGB *physic_leds)  // Gravcenter. By Andrew Tuline.
{
  mode_gravcenter_base(0, physic_leds);
}  // visualize_gravcenter()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVICENTRIC
void MusicLeds::visualize_gravcentric(CRGB *physic_leds)  // Gravcentric. By Andrew Tuline.
{
  mode_gravcenter_base(1, physic_leds);
}  // visualize_gravcentric
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_GRAVIMETER
void MusicLeds::visualize_gravmeter(CRGB *physic_leds)  // Gravmeter. By Andrew Tuline.
{
  mode_gravcenter_base(2, physic_leds);
}  // visualize_gravcentric
#endif

// *****************************************************************************************************************************************************************
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
  if (!this->allocateData(32 * sizeof(uint8_t))) {
    return;  // allocation failed
  }
  uint8_t *myVals = reinterpret_cast<uint8_t *>(this->data);

  myVals[millis() % 32] = this->features_.volume_smth();  // filling values semi randomly

  fastled_helper::fade_out(physic_leds, this->leds_num, 64 + (this->speed >> 1), this->back_color);

  for (int i = 0; i < (int) this->variant / 8; i++) {
    uint16_t segLoc = fastled_helper::hw_random16(this->leds_num);  // 16 bit for larger strands of LED's.
    physic_leds[segLoc] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(myVals[i % 32] + i * 4, this->main_color),
        this->features_.volume_smth());
  }
}  // visualize_pixels()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_JUNGLES
void MusicLeds::visualize_juggles(CRGB *physic_leds)  // Juggles. By Andrew Tuline.
{
  fastled_helper::fade_out(physic_leds, this->leds_num, 224, this->back_color);

  int my_sampleAgc = static_cast<int>(this->features_.smoothed_volume * 255.0f);

  for (int i = 0; i < (int) this->variant / 32 + 1; i++) {
    physic_leds[beatsin16((int) this->speed / 4 + i * 2, 0, this->leds_num - 1)] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(millis() / 4 + i * 2, this->main_color), my_sampleAgc);
  }
}  // visualize_juggles()
#endif

// *****************************************************************************************************************************************************************
#ifdef DEF_MIDNOISE
void MusicLeds::visualize_midnoise(CRGB *physic_leds) {  // Midnoise. By Andrew Tuline.
  static int x = 0;
  static int y = 0;

  // Smoothly fade out old pixels based on speed setting
  fastled_helper::fade_out(physic_leds, this->leds_num, 
                          (static_cast<int>(this->speed) * static_cast<int>(this->speed)) / 255, 
                          this->back_color);

  // --- OPTIMIZED AUDIO PIPELINE INTEGRATION ---
  // Using maximum 32-bit floating point precision for sub-pixel noise smoothness
  float tmpSound = this->features_.smoothed_volume * 255.0f;
  
  // Condense complex division scale chains into a single high-performance math expression
  float variant_factor = static_cast<float>(this->variant);
  float tmpSound2 = tmpSound * (variant_factor * variant_factor) / 32768.0f;

  // Map volume power into expanding center-bleed bounds array indexes
  unsigned maxLen = remap(tmpSound2, 0.0f, 127.0f, 0.0f, static_cast<float>(this->leds_num) / 2.0f);
  if (maxLen > this->leds_num / 2) {
    maxLen = this->leds_num / 2;
  }

  // Generate dynamic Perlin Noise texture expanding symmetrically from the center
  for (int i = (this->leds_num / 2 - maxLen); i < (this->leds_num / 2 + maxLen); i++) {
    // Inject the raw audio volume directly into the noise coordinates to warp the texture with audio waves
    uint8_t index = fastled_helper::perlin8(static_cast<uint16_t>(i * tmpSound + x), 
                                            static_cast<uint16_t>(y + i * tmpSound));
    physic_leds[i] = fastled_helper::color_from_palette(index, this->main_color);
  }

  // Slow LFO-based offset progression to keep the noise canvas scrolling smoothly over time
  x = x + beatsin8(5, 0, 10);
  y = y + beatsin8(4, 0, 10);
}  // visualize_midnoise()
#endif

#ifdef DEF_RIPPLEPEAK
typedef struct Ripple {
  uint8_t state;
  uint8_t color;
  uint16_t pos;
} ripple;

void MusicLeds::visualize_ripplepeak(CRGB *physic_leds)  // Ripple peak. By Andrew Tuline.
{                                                        // This currently has no controls.
#define MAXSTEPS 16                                      // Case statement wouldn't allow a variable.

  unsigned maxRipples = 16;
  unsigned dataSize = sizeof(Ripple) * maxRipples;
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  Ripple *ripples = reinterpret_cast<Ripple *>(this->data);

  // Lower frame rate means less effective fading than FastLED
  // 225 should be the same as 240 applied twice
  fastled_helper::fade_out(physic_leds, this->leds_num, 225, this->back_color);

  for (int i = 0; i < this->variant / 16; i++) {  // Limit the number of ripples.
    if (this->features_.sample_peak) {
      ripples[i].state = 255;
    }

    switch (ripples[i].state) {
      case 254:  // Inactive mode
        break;

      case 255:  // Initialize ripple variables.
        ripples[i].pos = fastled_helper::hw_random16(this->leds_num);
        if (this->features_.dominant_frequency_hz > 1.0f)
          ripples[i].color = static_cast<int>(log10f(this->features_.dominant_frequency_hz) * 128.0f);
        else
          ripples[i].color = 0;
        ripples[i].state = 0;
        break;

      case 0:
        physic_leds[ripples[i].pos] = fastled_helper::color_from_palette(ripples[i].color, this->main_color);
        ripples[i].state++;
        break;

      case MAXSTEPS:  // At the end of the ripples. 254 is an inactive mode.
        ripples[i].state = 254;
        break;

      default:  // Middle of the ripples.
        physic_leds[(ripples[i].pos + ripples[i].state + this->leds_num) % this->leds_num] =
            fastled_helper::color_blend(this->back_color,
                                        fastled_helper::color_from_palette(ripples[i].color, this->main_color),
                                        uint8_t(2 * 255 / ripples[i].state));
        physic_leds[(ripples[i].pos - ripples[i].state + this->leds_num) % this->leds_num] =
            fastled_helper::color_blend(this->back_color,
                                        fastled_helper::color_from_palette(ripples[i].color, this->main_color),
                                        uint8_t(2 * 255 / ripples[i].state));
        ripples[i].state++;  // Next step.
        break;
    }  // switch step
  }  // for i
}  // visualize_ripplepeak()
#endif

#ifdef DEF_MATRIPIX
void MusicLeds::visualize_matripix(CRGB *physic_leds)  // Matripix. By Andrew Tuline.
{
  // effect can work on single pixels, we just lose the shifting effect
  unsigned dataSize = sizeof(CRGB) * this->leds_num;
  if (!this->allocateData(dataSize)) {
    return;  // allocation failed
  }
  CRGB *pixels = reinterpret_cast<CRGB *>(this->data);

  uint8_t secondHand = micros() / (256 - this->speed) / 500 % 16;
  if (this->store != secondHand) {
    this->store = secondHand;

    uint16_t pixBri = (this->features_.volume_raw() * this->variant) / 64;

    unsigned k = this->leds_num - 1;
    // loop will not execute if SEGLEN equals 1
    for (unsigned i = 0; i < k; i++) {
      pixels[i] = pixels[i + 1];  // shift left
      physic_leds[i] = pixels[i];
    }
    pixels[k] = fastled_helper::color_blend(this->back_color,
                                            fastled_helper::color_from_palette(millis(), this->main_color), pixBri);
    physic_leds[k] = pixels[k];
  }
}  // visualize_matripix()
#endif

#ifdef DEF_NOISEFIRE
// I am the god of hellfire. . . Volume (only) reactive fire routine. Oh, look how short this is.
void MusicLeds::visualize_noisefire(CRGB *physic_leds)  // Noisefire. By Andrew Tuline.
{
  // Fire palette definition. Lower value = darker.
  CRGBPalette16 myPal =
      CRGBPalette16(CHSV(0, 255, 2), CHSV(0, 255, 4), CHSV(0, 255, 8), CHSV(0, 255, 8), CHSV(0, 255, 16), CRGB::Red,
                    CRGB::Red, CRGB::Red, CRGB::DarkOrange, CRGB::DarkOrange, CRGB::Orange, CRGB::Orange, CRGB::Yellow,
                    CRGB::Orange, CRGB::Yellow, CRGB::Yellow);

  for (unsigned i = 0; i < this->leds_num; i++) {
    // X location is constant, but we move along the Y at the rate of millis(). By Andrew Tuline.
    unsigned index = fastled_helper::perlin8(i * this->speed / 64, millis() * this->speed / 64 * this->leds_num / 255);
    // Now we need to scale index so that it gets blacker as we get close to one of the ends.
    // This is a simple y=mx+b equation that's been scaled. index/128 is another scaling.
    index = (255 - i * 256 / this->leds_num) * index / (256 - this->variant);

    uint8_t dynamic_brightness = static_cast<uint8_t>(std::min(this->features_.smoothed_volume * 255.0f * 2.0f, 255.0f));
    physic_leds[i] = ColorFromPalette(myPal, index, dynamic_brightness, LINEARBLEND);
  }
}  // visualize_noisefire()
#endif

#ifdef DEF_PIXELWAVE
void MusicLeds::visualize_pixelwave(CRGB *physic_leds)  // Pixelwave. By Andrew Tuline.
{
  uint8_t secondHand = micros() / (256 - this->speed) / 500 + 1 % 16;
  if (this->store != secondHand) {
    this->store = secondHand;

    uint16_t pixBri = (this->features_.volume_raw() * this->variant) / 64;

    physic_leds[this->leds_num / 2] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(millis(), this->main_color), pixBri);
    for (unsigned i = this->leds_num - 1; i > this->leds_num / 2; i--) {
      physic_leds[i] = physic_leds[i - 1];  // move to the left
    }
    for (unsigned i = 0; i < this->leds_num / 2; i++) {
      physic_leds[i] = physic_leds[i + 1];  // move to the right
    }
  }

}  // visualize_pixelwave()
#endif

#ifdef DEF_PLASMOID
typedef struct Plasphase {
  int16_t thisphase;
  int16_t thatphase;
} plasphase;

void MusicLeds::visualize_plasmoid(CRGB *physic_leds)  // Plasmoid. By Andrew Tuline.
{
  if (!this->allocateData(sizeof(plasphase))) {
    return;  // allocation failed
  }
  Plasphase *plasmoip = reinterpret_cast<Plasphase *>(this->data);

  fastled_helper::fade_out(physic_leds, this->leds_num, 32, this->back_color);

  plasmoip->thisphase += beatsin8(6, -4, 4);  // You can change direction and speed individually.
  plasmoip->thatphase += beatsin8(7, -4, 4);  // Two phase values to make a complex pattern. By Andrew Tuline.

  for (unsigned i = 0; i < this->leds_num;
       i++) {  // For each of the LED's in the strand, set a brightness based on a wave as follows.
    // updated, similar to "plasma" effect - softhack007
    uint8_t thisbright = cubicwave8(((i * (1 + (3 * this->speed / 32))) + plasmoip->thisphase) & 0xFF) / 2;
    // Let's munge the brightness a bit and animate it all with the phases.
    thisbright += cos8(((i * (97 + (5 * this->speed / 32))) + plasmoip->thatphase) & 0xFF) / 2;

    uint8_t colorIndex = thisbright;
    float volume_scaled = this->features_.smoothed_volume * 255.0f;
    float threshold_calc = (volume_scaled * static_cast<float>(this->variant)) / 64.0f;

    if (static_cast<uint8_t>(std::clamp(threshold_calc, 0.0f, 255.0f)) < thisbright) {
      thisbright = 0;
    }

    physic_leds[i] = fastled_helper::color_blend(
        this->back_color, fastled_helper::color_from_palette(colorIndex, this->main_color), thisbright);
  }
}  // visualize_plasmoid()
#endif

#if defined(DEF_PUDDLES) || defined(DEF_PUDDLEPEAK)
// Puddles / Puddlepeak By Andrew Tuline.
void MusicLeds::puddles_base(CRGB *physic_leds, bool peakdetect) {
  unsigned size = 0;

  // Fade out older pixels smoothly based on the animation speed setting
  uint8_t fadeVal = map(this->speed, 0, 255, 224, 254);
  unsigned pos = fastled_helper::hw_random16(this->leds_num);
  fastled_helper::fade_out(physic_leds, this->leds_num, fadeVal, this->back_color);

  if (peakdetect) {  // --- PUDDLES PEAK MODE ---
    // Link straight to our new statistical Cross-Core Beat Detector
    if (this->features_.sample_peak) {
      // Calculate flash length inside safe float boundaries to eliminate wrap-around bugs
      float volume_scaled = this->features_.smoothed_volume * 255.0f;
      float variant_factor = static_cast<float>(this->variant);
      
      // Replicating original scaling logic but optimized: /256/4 is mathematically close to /1024
      size = static_cast<unsigned>((volume_scaled * variant_factor) / 1024.0f) + 1;
      
      if (pos + size >= this->leds_num) {
        size = this->leds_num - pos;
      }
    }
  } else {  // --- PUDDLES STANDARD MODE ---
    // Link straight to our fresh frame-local raw volume transient tracker
    if (this->features_.raw_volume > 0.01f) {
      
      float raw_volume_scaled = this->features_.raw_volume * 255.0f;
      float variant_factor = static_cast<float>(this->variant);
      
      // Replicating original scaling logic but optimized: /256/8 is mathematically close to /2048
      size = static_cast<unsigned>((raw_volume_scaled * variant_factor) / 2048.0f) + 1;
      
      if (pos + size >= this->leds_num) {
        size = this->leds_num - pos;
      }
    }
  }

  // Render the freshly generated puddle splash onto the LED grid array
  for (unsigned i = 0; i < size; i++) {
    physic_leds[pos + i] = fastled_helper::color_from_palette(millis(), this->main_color);
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

#ifdef DEF_DJLIGHT
void MusicLeds::visualize_DJLight(CRGB *physic_leds) {  // DJLight. Written by ??? Adapted by Will Tatam.
  // No need to prevent from executing on single led strips, only mid will be set (mid = 0)
  const int mid = this->leds_num / 2;

  // Control execution pacing based on effect speed setting using micros()
  uint8_t secondHand = (micros() / (256 - this->speed) / 500 + 1) % 64;
  if (this->store != secondHand) {
    this->store = secondHand;

    // R = Bass (reds), G = Mids (greens), B = Highs (blues)
    uint8_t r = static_cast<uint8_t>(this->features_.bass_energy * 127.5f); // Half amplitude like legacy / 2
    uint8_t g = static_cast<uint8_t>(this->features_.mid_energy * 127.5f);
    uint8_t b = static_cast<uint8_t>(this->features_.high_energy * 127.5f);

    CRGB color = CRGB(r, g, b);
    
    // Scale central pixel dynamic dampening using midrange instrumentation energy
    // Map float [0.0f .. 1.0f] into inverted fade mask [255 .. 4]
    uint8_t fade_amount = static_cast<uint8_t>(4.0f + (251.0f * (1.0f - this->features_.mid_energy)));
    physic_leds[mid] = color.fadeToBlackBy(fade_amount);

    // Replicate the center-outward bidirectional shift loop (Waterfall movement)
    // Shift left side outward from center to index 0
    for (int i = 0; i < mid; i++) {
      physic_leds[i] = physic_leds[i + 1];
    }
    // Shift right side outward from center to the very end
    for (int i = this->leds_num - 1; i > mid; i--) {
      physic_leds[i] = physic_leds[i - 1];
    }
  }
}  // visualize_DJLight()
#endif

#ifdef DEF_WATERFALL
// Combines peak detection with FFT_MajorPeak and FFT_Magnitude.
void MusicLeds::visualize_waterfall(CRGB *physic_leds) { // Waterfall. By: Andrew Tuline
  unsigned dataSize = sizeof(CRGB) * this->leds_num;
  if (!this->allocateData(dataSize)) {
    return;  // Allocation failed
  }
  CRGB *pixels = reinterpret_cast<CRGB *>(this->data);

  // Control execution pacing based on the effect speed setting using micros()
  uint8_t secondHand = (micros() / (256 - this->speed) / 500 + 1) % 16;
  if (this->store != secondHand) {
    this->store = secondHand;

    float current_pitch = this->features_.dominant_frequency_hz;
    
    // Andrew Tuline's native base-10 logarithmic pitch-to-palette index mapping
    uint8_t pixCol = 0;
    if (current_pitch >= 182.0f) {
      pixCol = static_cast<uint8_t>((log10f(current_pitch) - 2.26f) * 150.0f);
    } // Underflow case is implicitly handled by initializing pixCol to 0

    unsigned k = this->leds_num - 1;

    if (this->features_.sample_peak) {
      // Inject a solid bright dynamic burst line on every clean rhythmic drum hit
      pixels[k] = CRGB(CHSV(92, 92, 92));
    } else {
      uint8_t blend_amount = this->features_.volume_smth();

      pixels[k] = fastled_helper::color_blend(
          this->back_color, 
          fastled_helper::color_from_palette(pixCol + this->variant, this->main_color),
          blend_amount
      );
    }
    physic_leds[k] = pixels[k];

    // Shift the pixel trail memory line from right to left (Waterfall movement sweep)
    for (unsigned i = 0; i < k; i++) {
      pixels[i] = pixels[i + 1];
      physic_leds[i] = pixels[i];
    }
  }
}  // visualize_waterfall()
#endif

}  // namespace esphome::music_leds
