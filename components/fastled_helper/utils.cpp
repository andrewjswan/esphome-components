#include "colors.h"
#include "palettes.h"
#include "utils.h"
#include "fastled_slim.h"

#include "esphome/core/color.h"
#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

// *****************************************************************************************************************************************************************
void IRAM_ATTR fade_out(CRGB *physic_leds, uint16_t _leds_num, uint8_t rate, CRGB back_color) {
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

// blur: one-dimensional blur filter. Spreads light to 2 line neighbors.
//
//    0 = no spread at all
//   64 = moderate spreading
//  172 = maximum smooth, even spreading
//
//  173..255 = wider spreading, but increasing flicker
//
//  Total light is NOT entirely conserved, so many repeated
//  calls to 'blur' will also result in the light fading,
//  eventually all the way to black; this is by design so that
//  it can be used to (slowly) clear the LEDs to black.
void IRAM_ATTR blur(CRGB *physic_leds, uint16_t numLeds, uint8_t blur_amount) {
    uint8_t keep = 255 - blur_amount;
    uint8_t seep = blur_amount >> 1;
    CRGB carryover = CRGB::Black;

    for(uint16_t i = 0; i < numLeds; ++i) {
        CRGB cur = physic_leds[i];
        CRGB part = cur;
        part.nscale8(seep);
        cur.nscale8(keep);

        cur += carryover;
        if(i > 0) physic_leds[i-1] += part;

        physic_leds[i] = cur;
        carryover = part;
    }
}

void IRAM_ATTR fadeLightBy(CRGB *leds, uint16_t num_leds, uint8_t fadeBy) {
    nscale8_video(leds, num_leds, 255 - fadeBy);
}
 
void IRAM_ATTR fadeToBlackBy(CRGB *leds, uint16_t num_leds, uint8_t fadeBy) {
    nscale8(leds, num_leds, 255 - fadeBy);
}

// *****************************************************************************************************************************************************************

// FastLED Reference
// -----------------
// The following beat functions derived from FastLED @ 3.6.0 (https://github.com/FastLED/FastLED) are licensed under the MIT license
// See /src/dependencies/fastled_slim/LICENSE.txt for details

// 16-bit, integer based Bhaskara I's sine approximation: 16*x*(pi - x) / (5*pi^2 - 4*x*(pi - x))
// input is 16bit unsigned (0-65535), output is 16bit signed (-32767 to +32767)
// optimized integer implementation by @dedehai
int16_t sin16_t(uint16_t theta) {
  int scale = 1;
  if (theta > 0x7FFF) {
    theta = 0xFFFF - theta;
    scale = -1; // second half of the sine function is negative (pi - 2*pi)
  }
  uint32_t precal = theta * (0x7FFF - theta);
  uint64_t numerator = (uint64_t)precal * (4 * 0x7FFF); // 64bit required
  int32_t denominator = 1342095361 - precal; // 1342095361 is 5 * 0x7FFF^2 / 4
  int16_t result = numerator / denominator;
  return result * scale;
}

int16_t cos16_t(uint16_t theta) {
  return sin16_t(theta + 0x4000); // cos(x) = sin(x+pi/2)
}

uint8_t sin8_t(uint8_t theta) {
  int32_t sin16 = sin16_t((uint16_t)theta * 257); // 255 * 257 = 0xFFFF
  sin16 += 0x7FFF + 128; // shift result to range 0-0xFFFF, +128 for rounding
  return min(sin16, int32_t(0xFFFF)) >> 8; // min performs saturation, and prevents overflow
}

uint8_t cos8_t(uint8_t theta) {
  return sin8_t(theta + 64); // cos(x) = sin(x+pi/2)
}

uint8_t sqrt16_t(uint16_t x) {
    if (x <= 1) {
        return x;
    }

    uint8_t low = 1; // lower bound
    uint8_t hi, mid;

    if (x > 7904) {
        hi = 255;
    } else {
        hi = (x >> 5) + 8; // initial estimate for upper bound
    }

    do {
        mid = (low + hi) >> 1;
        if ((uint16_t)(mid * mid) > x) {
            hi = mid - 1;
        } else {
            if (mid == 255) {
                return 255;
            }
            low = mid + 1;
        }
    } while (hi >= low);

    return low - 1;
}

uint8_t sqrt8_t(uint8_t x) {
    return sqrt16_t(map8_to_16(x));
}

// Generates a 16-bit "sawtooth" wave at a given BPM, with BPM specified in Q8.8 fixed-point format:
// for 120 BPM it would be 120*256 = 30720. If you just want to specify "120", use beat16() or beat8().
// timebase is the time offset of the wave from the millis() timer
uint16_t beat88(uint16_t beats_per_minute_88, uint32_t timebase) {
  return ((millis() - timebase) * beats_per_minute_88 * 280) >> 16;
}

// Generates a 16-bit "sawtooth" wave at a given BPM
uint16_t beat16(uint16_t beats_per_minute, uint32_t timebase) {
  if (beats_per_minute < 256) beats_per_minute <<= 8;
  return beat88(beats_per_minute, timebase);
}

/// Generates an 8-bit "sawtooth" wave at a given BPM
uint8_t beat8(uint16_t beats_per_minute, uint32_t timebase) {
  return beat16(beats_per_minute, timebase) >> 8;
}

// Generates a 16-bit sine wave at a given BPM that oscillates within a given range. see fastled for details.
uint16_t beatsin88_t(uint16_t beats_per_minute_88, uint16_t lowest, uint16_t highest, uint32_t timebase, uint16_t phase_offset) {
  uint16_t beat = beat88(beats_per_minute_88, timebase);
  uint16_t beatsin = (sin16_t( beat + phase_offset) + 32768);
  uint16_t rangewidth = highest - lowest;
  uint16_t scaledbeat = scale16(beatsin, rangewidth);
  uint16_t result = lowest + scaledbeat;
  return result;
}

// Generates a 16-bit sine wave at a given BPM that oscillates within a given range. see fastled for details.
uint16_t beatsin16_t(uint16_t beats_per_minute, uint16_t lowest, uint16_t highest, uint32_t timebase, uint16_t phase_offset) {
  uint16_t beat = beat16(beats_per_minute, timebase);
  uint16_t beatsin = sin16_t(beat + phase_offset) + 32768;
  uint16_t rangewidth = highest - lowest;
  uint16_t scaledbeat = scale16(beatsin, rangewidth);
  uint16_t result = lowest + scaledbeat;
  return result;
}

// Generates an 8-bit sine wave at a given BPM that oscillates within a given range. see fastled for details.
uint8_t beatsin8_t(uint16_t beats_per_minute, uint8_t lowest, uint8_t highest, uint32_t timebase, uint8_t phase_offset) {
  uint8_t beat = beat8(beats_per_minute, timebase);
  uint8_t beatsin = sin8_t(beat + phase_offset);
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatsin, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}

// Generates an 8-bit cosine wave at a given BPM that oscillates within a given range. see fastled for details.
uint8_t beatcos8_t(uint16_t beats_per_minute, uint8_t lowest, uint8_t highest, uint32_t timebase, uint8_t phase_offset) {
  uint8_t beat = beat8(beats_per_minute, timebase);
  uint8_t beatcos = cos8_t(beat + phase_offset);
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatcos, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}

// end of FastLED functions

// *****************************************************************************************************************************************************************
/*
 * Fixed point integer based Perlin noise functions by @dedehai
 * Note: optimized for speed and to mimic fastled inoise functions, not for accuracy or best randomness
 */

#define PERLIN_SHIFT 1

// calculate gradient for corner from hash value
static inline __attribute__((always_inline)) int32_t hashToGradient(uint32_t h) {
  // using more steps yields more "detailed" perlin noise but looks less like the original fastled version (adjust
  // PERLIN_SHIFT to compensate, also changes range and needs proper adustment) return (h & 0xFF) - 128; // use
  // PERLIN_SHIFT 7 return (h & 0x0F) - 8; // use PERLIN_SHIFT 3 return (h & 0x07) - 4; // use PERLIN_SHIFT 2
  return (h & 0x03) - 2;  // use PERLIN_SHIFT 1 -> closest to original fastled version
}

// Gradient functions for 1D, 2D and 3D Perlin noise  note: forcing inline produces smaller code and makes it 3x faster!
static inline __attribute__((always_inline)) int32_t gradient1D(uint32_t x0, int32_t dx) {
  uint32_t h = x0 * 0x27D4EB2D;
  h ^= h >> 15;
  h *= 0x92C3412B;
  h ^= h >> 13;
  h ^= h >> 7;
  return (hashToGradient(h) * dx) >> PERLIN_SHIFT;
}

static inline __attribute__((always_inline)) int32_t gradient2D(uint32_t x0, int32_t dx, uint32_t y0, int32_t dy) {
  uint32_t h = (x0 * 0x27D4EB2D) ^ (y0 * 0xB5297A4D);
  h ^= h >> 15;
  h *= 0x92C3412B;
  h ^= h >> 13;
  return (hashToGradient(h) * dx + hashToGradient(h >> PERLIN_SHIFT) * dy) >> (1 + PERLIN_SHIFT);
}

static inline __attribute__((always_inline)) int32_t gradient3D(uint32_t x0, int32_t dx, uint32_t y0, int32_t dy,
                                                                uint32_t z0, int32_t dz) {
  // fast and good entropy hash from corner coordinates
  uint32_t h = (x0 * 0x27D4EB2D) ^ (y0 * 0xB5297A4D) ^ (z0 * 0x1B56C4E9);
  h ^= h >> 15;
  h *= 0x92C3412B;
  h ^= h >> 13;
  return ((hashToGradient(h) * dx + hashToGradient(h >> (1 + PERLIN_SHIFT)) * dy +
           hashToGradient(h >> (1 + 2 * PERLIN_SHIFT)) * dz) *
          85) >>
         (8 + PERLIN_SHIFT);  // scale to 16bit, x*85 >> 8 = x/3
}

// fast cubic smoothstep: t*(3 - 2t²), optimized for fixed point, scaled to avoid overflows
static uint32_t smoothstep(const uint32_t t) {
  uint32_t t_squared = (t * t) >> 16;
  uint32_t factor = (3 << 16) - ((t << 1));
  return (t_squared * factor) >> 18;  // scale to avoid overflows and give best resolution
}

// simple linear interpolation for fixed-point values, scaled for perlin noise use
static inline int32_t lerpPerlin(int32_t a, int32_t b, int32_t t) {
  return a + (((b - a) * t) >> 14);  // match scaling with smoothstep to yield 16.16bit values
}

// 1D Perlin noise function that returns a value in range of -24691 to 24689
int32_t perlin1D_raw(uint32_t x, bool is16bit) {
  // integer and fractional part coordinates
  int32_t x0 = x >> 16;
  int32_t x1 = x0 + 1;
  if (is16bit)
    x1 = x1 & 0xFF;  // wrap back to zero at 0xFF instead of 0xFFFF

  int32_t dx0 = x & 0xFFFF;
  int32_t dx1 = dx0 - 0x10000;
  // gradient values for the two corners
  int32_t g0 = gradient1D(x0, dx0);
  int32_t g1 = gradient1D(x1, dx1);
  // interpolate and smooth function
  int32_t tx = smoothstep(dx0);
  int32_t noise = lerpPerlin(g0, g1, tx);
  return noise;
}

// 2D Perlin noise function that returns a value in range of -20633 to 20629
int32_t perlin2D_raw(uint32_t x, uint32_t y, bool is16bit) {
  int32_t x0 = x >> 16;
  int32_t y0 = y >> 16;
  int32_t x1 = x0 + 1;
  int32_t y1 = y0 + 1;

  if (is16bit) {
    x1 = x1 & 0xFF;  // wrap back to zero at 0xFF instead of 0xFFFF
    y1 = y1 & 0xFF;
  }

  int32_t dx0 = x & 0xFFFF;
  int32_t dy0 = y & 0xFFFF;
  int32_t dx1 = dx0 - 0x10000;
  int32_t dy1 = dy0 - 0x10000;

  int32_t g00 = gradient2D(x0, dx0, y0, dy0);
  int32_t g10 = gradient2D(x1, dx1, y0, dy0);
  int32_t g01 = gradient2D(x0, dx0, y1, dy1);
  int32_t g11 = gradient2D(x1, dx1, y1, dy1);

  uint32_t tx = smoothstep(dx0);
  uint32_t ty = smoothstep(dy0);

  int32_t nx0 = lerpPerlin(g00, g10, tx);
  int32_t nx1 = lerpPerlin(g01, g11, tx);

  int32_t noise = lerpPerlin(nx0, nx1, ty);
  return noise;
}

// 3D Perlin noise function that returns a value in range of -16788 to 16381
int32_t perlin3D_raw(uint32_t x, uint32_t y, uint32_t z, bool is16bit) {
  int32_t x0 = x >> 16;
  int32_t y0 = y >> 16;
  int32_t z0 = z >> 16;
  int32_t x1 = x0 + 1;
  int32_t y1 = y0 + 1;
  int32_t z1 = z0 + 1;

  if (is16bit) {
    x1 = x1 & 0xFF;  // wrap back to zero at 0xFF instead of 0xFFFF
    y1 = y1 & 0xFF;
    z1 = z1 & 0xFF;
  }

  int32_t dx0 = x & 0xFFFF;
  int32_t dy0 = y & 0xFFFF;
  int32_t dz0 = z & 0xFFFF;
  int32_t dx1 = dx0 - 0x10000;
  int32_t dy1 = dy0 - 0x10000;
  int32_t dz1 = dz0 - 0x10000;

  int32_t g000 = gradient3D(x0, dx0, y0, dy0, z0, dz0);
  int32_t g001 = gradient3D(x0, dx0, y0, dy0, z1, dz1);
  int32_t g010 = gradient3D(x0, dx0, y1, dy1, z0, dz0);
  int32_t g011 = gradient3D(x0, dx0, y1, dy1, z1, dz1);
  int32_t g100 = gradient3D(x1, dx1, y0, dy0, z0, dz0);
  int32_t g101 = gradient3D(x1, dx1, y0, dy0, z1, dz1);
  int32_t g110 = gradient3D(x1, dx1, y1, dy1, z0, dz0);
  int32_t g111 = gradient3D(x1, dx1, y1, dy1, z1, dz1);

  uint32_t tx = smoothstep(dx0);
  uint32_t ty = smoothstep(dy0);
  uint32_t tz = smoothstep(dz0);

  int32_t nx0 = lerpPerlin(g000, g100, tx);
  int32_t nx1 = lerpPerlin(g010, g110, tx);
  int32_t nx2 = lerpPerlin(g001, g101, tx);
  int32_t nx3 = lerpPerlin(g011, g111, tx);
  int32_t ny0 = lerpPerlin(nx0, nx1, ty);
  int32_t ny1 = lerpPerlin(nx2, nx3, ty);

  int32_t noise = lerpPerlin(ny0, ny1, tz);
  return noise;
}

// scaling functions for fastled replacement
uint16_t perlin16(uint32_t x) {
  return ((perlin1D_raw(x) * 1159) >> 10) + 32803;  // scale to 16bit and offset (fastled range: about 4838 to 60766)
}

uint16_t perlin16(uint32_t x, uint32_t y) {
  return ((perlin2D_raw(x, y) * 1537) >> 10) + 32725;  // scale to 16bit and offset (fastled range: about 1748 to 63697)
}

uint16_t perlin16(uint32_t x, uint32_t y, uint32_t z) {
  return ((perlin3D_raw(x, y, z) * 1731) >> 10) +
         33147;  // scale to 16bit and offset (fastled range: about 4766 to 60840)
}

uint8_t perlin8(uint16_t x) {
  return (((perlin1D_raw((uint32_t) x << 8, true) * 1353) >> 10) + 32769) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

uint8_t perlin8(uint16_t x, uint16_t y) {
  return (((perlin2D_raw((uint32_t) x << 8, (uint32_t) y << 8, true) * 1620) >> 10) + 32771) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

uint8_t perlin8(uint16_t x, uint16_t y, uint16_t z) {
  return (((perlin3D_raw((uint32_t) x << 8, (uint32_t) y << 8, (uint32_t) z << 8, true) * 2015) >> 10) + 33168) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

#ifdef USE_PALETTES

// *****************************************************************************************************************************************************************
CRGB color_from_palette(const CRGBPalette16& pal, unsigned index, uint8_t brightness, TBlendType blendType) {
  if (blendType == LINEARBLEND_NOWRAP) {
    index = (index * 0xF0) >> 8; // Blend range is affected by lo4 blend of values, remap to avoid wrapping
  }
  unsigned hi4 = byte(index) >> 4;
  unsigned lo4 = (index & 0x0F);
  const CRGB* entry = (CRGB*)&(pal[0]) + hi4;
  unsigned red1   = entry->r;
  unsigned green1 = entry->g;
  unsigned blue1  = entry->b;
  if (lo4 && blendType != NOBLEND) {
    if (hi4 == 15) entry = &(pal[0]);
    else ++entry;
    unsigned f2 = (lo4 << 4);
    unsigned f1 = 256 - f2;
    red1   = (red1   * f1 + (unsigned)entry->r * f2) >> 8; // note: using color_blend() is slower
    green1 = (green1 * f1 + (unsigned)entry->g * f2) >> 8;
    blue1  = (blue1  * f1 + (unsigned)entry->b * f2) >> 8;
  }
  if (brightness < 255) { // note: zero checking could be done to return black but that is hardly ever used so it is omitted
    // actually same as color_fade(), using color_fade() is slower
    uint32_t scale = brightness + 1; // adjust for rounding (bitshift)
    red1   = (red1   * scale) >> 8;
    green1 = (green1 * scale) >> 8;
    blue1  = (blue1  * scale) >> 8;
  }
  return CRGB(red1, green1, blue1);
}

CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness) {
  if (current_palette == 0) {  // Current led color
    return color_from_palette(CRGBPalette16(current_color), index, brightness, LINEARBLEND);
  }

  if (current_palette == 1) {  // Periodically replace palette with a random one
    if (millis() - randomPaletteChange > 30000) {
      randomPalette = CRGBPalette16(CHSV(random8(), 255, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)),
                                    CHSV(random8(), 192, random8(128, 255)), CHSV(random8(), 255, random8(128, 255)));
      randomPaletteChange = millis();
    }
    return color_from_palette(randomPalette, index, brightness, LINEARBLEND);
  }

  if (current_palette == 2) {
#ifdef USE_MUSIC_LEDS  // MUSIC_LEDS
    return color_from_palette(get_audio_palette(0), index, brightness, LINEARBLEND);
#else
    return color_from_palette(paletteArr[random8(array_size(paletteArr))], index, brightness, LINEARBLEND);
#endif
  }

  if (current_palette == 3) {
#ifdef USE_MUSIC_LEDS  // MUSIC_LEDS
    return color_from_palette(get_audio_palette(1), index, brightness, LINEARBLEND);
#else
    return color_from_palette(paletteArr[random8(array_size(paletteArr))], index, brightness, LINEARBLEND);
#endif
  }

  return color_from_palette(paletteArr[current_palette - 4], index, brightness, LINEARBLEND);
}

CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness) {
  return color_from_palette(index, CRGB(current_color.r, current_color.g, current_color.b), brightness);
}

#ifdef USE_MUSIC_LEDS  // MUSIC_LEDS
// *****************************************************************************************************************************************************************
CRGB get_CRGB_for_band(int x, int pal) {
  CRGB value = CRGB::Black;  // Safe default fallback color

  // Intercept uninitialized or missing data pipelines safely
  if (g_fft_result_ptr == nullptr) {
    return value;
  }

  if (pal == 0) {
    // Read directly from the registered modern pipeline buffer memory layout
    uint8_t ch10 = g_fft_result_ptr[10] / 2;
    uint8_t ch4 = g_fft_result_ptr[4] / 2;
    uint8_t ch0 = g_fft_result_ptr[0] / 2;

    if (x == 1) {
      value = CRGB(ch10, ch4, ch0);
    } else if (x == 255) {
      value = CRGB(ch10, ch0, ch4);
    } else {
      value = CRGB(ch0, ch4, ch10);
    }
  } else if (pal == 1) {
    int b = map(x, 1, 255, 0, 8);
    uint8_t band_val = g_fft_result_ptr[b];

    // Maintain genuine integer constraints while picking the dynamic hue
    value = CHSV(band_val, 255, uint8_t(map(band_val, 0, 255, 30, 255)));
  }

  return value;
}

CRGBPalette16 get_audio_palette(int pal) {
  // https://forum.makerforums.info/t/hi-is-it-possible-to-define-a-gradient-palette-at-runtime-the-define-gradient-palette-uses-the/63339

  uint8_t xyz[16];  // Needs to be 4 times however many colors are being used.
                    // 3 colors = 12, 4 colors = 16, etc.

  xyz[0] = 0;  // Anchor of first color - must be zero
  xyz[1] = 0;
  xyz[2] = 0;
  xyz[3] = 0;

  CRGB rgb = get_CRGB_for_band(1, pal);
  xyz[4] = 1;  // Anchor of first color
  xyz[5] = rgb.r;
  xyz[6] = rgb.g;
  xyz[7] = rgb.b;

  rgb = get_CRGB_for_band(128, pal);
  xyz[8] = 128;
  xyz[9] = rgb.r;
  xyz[10] = rgb.g;
  xyz[11] = rgb.b;

  rgb = get_CRGB_for_band(255, pal);
  xyz[12] = 255;  // Anchor of last color - must be 255
  xyz[13] = rgb.r;
  xyz[14] = rgb.g;
  xyz[15] = rgb.b;

  return CRGBPalette16(xyz);
}
#endif  // MUSIC_LEDS

#endif  // PALETTES

}  // namespace esphome::fastled_helper
