#pragma once

#define FASTLED_INTERNAL  // remove annoying pragma messages

#include "FastLED.h"
#include "palettes.h"
#include "esphome/core/color.h"

namespace esphome {
namespace fastled_helper {

#define ARRAY_SIZE(array) ((sizeof(array)) / (sizeof(array[0])))

// *****************************************************************************************************************************************************************
static CRGB *leds;

#ifdef PALETTES

static uint8_t current_palette = 4;

static CRGBPalette16 randomPalette;

static uint32_t randomPaletteChange = 0;

static CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness);
static CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness);

#ifdef MUSIC_LEDS
static CRGBPalette16 getAudioPalette(int pal);
static CRGB getCRGBForBand(int x, int pal);
#endif

#endif

// *****************************************************************************************************************************************************************
static void InitLeds(int size) {
  if (leds == NULL) {
    leds = new CRGB[size];
  }
}

// *****************************************************************************************************************************************************************
static void FreeLeds() {
  delete[] leds;
  leds = nullptr;
}

// *****************************************************************************************************************************************************************
static CRGB IRAM_ATTR color_blend(CRGB color1, CRGB color2, uint16_t blend, bool b16 = false) {
  if (blend == 0) {
    return color1;
  }

  uint16_t blendmax = b16 ? 0xFFFF : 0xFF;

  if (blend == blendmax)
    return color2;

  uint8_t shift = b16 ? 16 : 8;

  uint32_t r1 = color1.r;
  uint32_t g1 = color1.g;
  uint32_t b1 = color1.b;

  uint32_t r2 = color2.r;
  uint32_t g2 = color2.g;
  uint32_t b2 = color2.b;

  uint32_t r3 = ((r2 * blend) + (r1 * (blendmax - blend))) >> shift;
  uint32_t g3 = ((g2 * blend) + (g1 * (blendmax - blend))) >> shift;
  uint32_t b3 = ((b2 * blend) + (b1 * (blendmax - blend))) >> shift;

  return CRGB(r3, g3, b3);
}

// *****************************************************************************************************************************************************************
static void fade_out(CRGB *physic_leds, uint16_t _leds_num, uint8_t rate, CRGB back_color) {
  rate = (255 - rate) >> 1;
  float mappedRate = float(rate) + 1.1;

  // uint32_t color = SEGCOLOR(1); // target color (SEGCOLOR(1) - Second Color in WLED (Background))
  int r2 = back_color.r;
  int g2 = back_color.g;
  int b2 = back_color.b;

  for (uint_fast16_t i = 0; i < _leds_num; i++) {
    int r1 = physic_leds[i].r;
    int g1 = physic_leds[i].g;
    int b1 = physic_leds[i].b;

    int rdelta = (r2 - r1) / mappedRate;
    int gdelta = (g2 - g1) / mappedRate;
    int bdelta = (b2 - b1) / mappedRate;

    // if fade isn't complete, make sure delta is at least 1 (fixes rounding issues)
    rdelta += (r2 == r1) ? 0 : (r2 > r1) ? 1 : -1;
    gdelta += (g2 == g1) ? 0 : (g2 > g1) ? 1 : -1;
    bdelta += (b2 == b1) ? 0 : (b2 > b1) ? 1 : -1;

    physic_leds[i] = CRGB(r1 + rdelta, g1 + gdelta, b1 + bdelta);
  }
}

#ifdef PALETTES

// *****************************************************************************************************************************************************************
static CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness = 255) {
  return color_from_palette(index, CRGB(current_color.r, current_color.g, current_color.b), brightness);
}

// *****************************************************************************************************************************************************************
static CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness = 255) {
  if (current_palette == 0)  // Current led color
  {
    return ColorFromPalette(CRGBPalette16(current_color), index, brightness, LINEARBLEND);
  }

  if (current_palette == 1)  // Periodically replace palette with a random one
  {
    if (millis() - randomPaletteChange > 30000) {
      randomPalette = CRGBPalette16(CHSV(random8(), 255, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)),
                                    CHSV(random8(), 192, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)));
      randomPaletteChange = millis();
    }
    return ColorFromPalette(randomPalette, index, brightness, LINEARBLEND);
  }

  if (current_palette == 2) {
#ifdef MUSIC_LEDS  // MUSIC_LEDS
    return ColorFromPalette(getAudioPalette(0), index, brightness, LINEARBLEND);
#else
    return ColorFromPalette(paletteArr[random8(ARRAY_SIZE(paletteArr))], index, brightness, LINEARBLEND);
#endif
  }

  if (current_palette == 3) {
#ifdef MUSIC_LEDS  // MUSIC_LEDS
    return ColorFromPalette(getAudioPalette(1), index, brightness, LINEARBLEND);
#else
    return ColorFromPalette(paletteArr[random8(ARRAY_SIZE(paletteArr))], index, brightness, LINEARBLEND);
#endif
  }

  return ColorFromPalette(paletteArr[current_palette - 4], index, brightness, LINEARBLEND);
}

#ifdef MUSIC_LEDS  // MUSIC_LEDS
// *****************************************************************************************************************************************************************
static CRGBPalette16 getAudioPalette(int pal) {
  // https://forum.makerforums.info/t/hi-is-it-possible-to-define-a-gradient-palette-at-runtime-the-define-gradient-palette-uses-the/63339

  uint8_t xyz[16];  // Needs to be 4 times however many colors are being used.
                    // 3 colors = 12, 4 colors = 16, etc.

  xyz[0] = 0;  // Anchor of first color - must be zero
  xyz[1] = 0;
  xyz[2] = 0;
  xyz[3] = 0;

  CRGB rgb = getCRGBForBand(1, pal);
  xyz[4] = 1;  // Anchor of first color
  xyz[5] = rgb.r;
  xyz[6] = rgb.g;
  xyz[7] = rgb.b;

  rgb = getCRGBForBand(128, pal);
  xyz[8] = 128;
  xyz[9] = rgb.r;
  xyz[10] = rgb.g;
  xyz[11] = rgb.b;

  rgb = getCRGBForBand(255, pal);
  xyz[12] = 255;  // Anchor of last color - must be 255
  xyz[13] = rgb.r;
  xyz[14] = rgb.g;
  xyz[15] = rgb.b;

  return CRGBPalette16(xyz);
}

// *****************************************************************************************************************************************************************
static CRGB getCRGBForBand(int x, int pal) {
  extern int fftResult[];  // Summary of bins Array. 16 summary bins.

  CRGB value;
  CHSV hsv;

  if (pal == 0) {
    if (x == 1) {
      value = CRGB(uint8_t(fftResult[10] / 2), uint8_t(fftResult[4] / 2), uint8_t(fftResult[0] / 2));
    } else if (x == 255) {
      value = CRGB(uint8_t(fftResult[10] / 2), uint8_t(fftResult[0] / 2), uint8_t(fftResult[4] / 2));
    } else {
      value = CRGB(uint8_t(fftResult[0] / 2), uint8_t(fftResult[4] / 2), uint8_t(fftResult[10] / 2));
    }
  } else if (pal == 1) {
    int b = map(x, 1, 255, 0, 8);  // Convert palette position to lower half of freq band
    hsv = CHSV(uint8_t(fftResult[b]), 255, uint8_t(map(fftResult[b], 0, 255, 30, 255)));  // Pick hue
    hsv2rgb_rainbow(hsv, value);                                                          // Convert to R,G,B
  }
  return value;
}
#endif  // MUSIC_LEDS

#endif  // PALETTES

}  // namespace fastled_helper
}  // namespace esphome
