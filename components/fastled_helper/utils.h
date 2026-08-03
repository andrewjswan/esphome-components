#pragma once

#define FASTLED_INTERNAL  // remove annoying pragma messages

#include "FastLED.h"

#include "palettes.h"

#include "esphome/core/color.h"
#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

#define ARRAY_SIZE(array) ((sizeof(array)) / (sizeof(array[0])))

// *****************************************************************************************************************************************************************
static CRGB *leds;

#ifdef PALETTES

static uint8_t current_palette = 4;

static CRGBPalette16 randomPalette;

static uint32_t randomPaletteChange = 0;

inline CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness);
inline CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness);

#ifdef MUSIC_LEDS
static CRGBPalette16 getAudioPalette(int pal);
static CRGB getCRGBForBand(int x, int pal);
#endif

#endif

// *****************************************************************************************************************************************************************
inline void InitLeds(int size) {
  if (leds == NULL) {
    leds = new CRGB[size];
  }
}

// *****************************************************************************************************************************************************************
inline void FreeLeds() {
  delete[] leds;
  leds = nullptr;
}

// *****************************************************************************************************************************************************************
/**
 * @brief Ultra-high performance SIMD color blending.
 * @note Parameter 'blend' accepts uint16_t to explicitly prevent external caller casting bugs,
 *       but internally clamps to 255 to maintain bitwise integrity of the 0x00FF00FF masks.
 */
inline CRGB color_blend(CRGB color1, CRGB color2, uint16_t blend) {
  // Force safe 8-bit saturation threshold to completely protect poor-man's SIMD masks
  uint32_t b = (blend > 255) ? 255 : blend;

  // Convert explicit CRGB structures directly into packed 32-bit uint32_t
  uint32_t c1 = (static_cast<uint32_t>(color1.r) << 16) | (static_cast<uint32_t>(color1.g) << 8) | color1.b;
  uint32_t c2 = (static_cast<uint32_t>(color2.r) << 16) | (static_cast<uint32_t>(color2.g) << 8) | color2.b;

  // Replicated poor-man's SIMD parallel processing matrix layout
  constexpr uint32_t TWO_CHANNEL_MASK = 0x00FF00FF; 
  
  uint32_t rb1 =  c1       & TWO_CHANNEL_MASK;  // Extract Red & Blue channels from color1
  uint32_t wg1 = (c1 >> 8) & TWO_CHANNEL_MASK;  // Extract White & Green channels from color1
  uint32_t rb2 =  c2       & TWO_CHANNEL_MASK;  // Extract Red & Blue channels from color2
  uint32_t wg2 = (c2 >> 8) & TWO_CHANNEL_MASK;  // Extract White & Green channels from color2

  // Parallel execution of alpha blending channels inside unified 32-bit registers
  uint32_t rb3 = ((((rb1 << 8) | rb2) + (rb2 * b) - (rb1 * b)) >> 8) &  TWO_CHANNEL_MASK;
  uint32_t wg3 = ((((wg1 << 8) | wg2) + (wg2 * b) - (wg1 * b)))      & ~TWO_CHANNEL_MASK; // Hardcoded bitwise inversion (~0x00FF00FF)

  uint32_t packed_result = rb3 | wg3;

  // Unpack back into native FastLED CRGB components seamlessly
  return CRGB(
    static_cast<uint8_t>((packed_result >> 16) & 0xFF), // Red channel
    static_cast<uint8_t>((packed_result >> 8)  & 0xFF), // Green channel
    static_cast<uint8_t>(packed_result         & 0xFF)  // Blue channel
  );
}

// *****************************************************************************************************************************************************************
inline void fade_out(CRGB *physic_leds, uint16_t _leds_num, uint8_t rate, CRGB back_color) {
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

// *****************************************************************************************************************************************************************
// Generates an 8-bit cosine wave at a given BPM that oscillates within a given range. see fastled for details.
inline uint8_t beatcos8(accum88 beats_per_minute, uint8_t lowest = 0, uint8_t highest = 255, uint32_t timebase = 0,
                        uint8_t phase_offset = 0) {
  uint8_t beat = beat8(beats_per_minute, timebase);
  uint8_t beatcos = cos8(beat + phase_offset);
  uint8_t rangewidth = highest - lowest;
  uint8_t scaledbeat = scale8(beatcos, rangewidth);
  uint8_t result = lowest + scaledbeat;
  return result;
}

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
static int32_t perlin1D_raw(uint32_t x, bool is16bit = false) {
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
static int32_t perlin2D_raw(uint32_t x, uint32_t y, bool is16bit = false) {
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
static int32_t perlin3D_raw(uint32_t x, uint32_t y, uint32_t z, bool is16bit = false) {
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
inline uint16_t perlin16(uint32_t x) {
  return ((perlin1D_raw(x) * 1159) >> 10) + 32803;  // scale to 16bit and offset (fastled range: about 4838 to 60766)
}

inline uint16_t perlin16(uint32_t x, uint32_t y) {
  return ((perlin2D_raw(x, y) * 1537) >> 10) + 32725;  // scale to 16bit and offset (fastled range: about 1748 to 63697)
}

inline uint16_t perlin16(uint32_t x, uint32_t y, uint32_t z) {
  return ((perlin3D_raw(x, y, z) * 1731) >> 10) +
         33147;  // scale to 16bit and offset (fastled range: about 4766 to 60840)
}

inline uint8_t perlin8(uint16_t x) {
  return (((perlin1D_raw((uint32_t) x << 8, true) * 1353) >> 10) + 32769) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

inline uint8_t perlin8(uint16_t x, uint16_t y) {
  return (((perlin2D_raw((uint32_t) x << 8, (uint32_t) y << 8, true) * 1620) >> 10) + 32771) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

inline uint8_t perlin8(uint16_t x, uint16_t y, uint16_t z) {
  return (((perlin3D_raw((uint32_t) x << 8, (uint32_t) y << 8, (uint32_t) z << 8, true) * 2015) >> 10) + 33168) >>
         8;  // scale to 16 bit, offset, then scale to 8bit
}

/**
 * @brief Computes the inverse gamma correction for a single 8-bit color channel value.
 * @details Replicates the original mathematical transformation curve using a standard 2.8 gamma scale factor.
 * @param val The incoming linear 8-bit channel intensity value [0 .. 255].
 * @return The inverse gamma corrected 8-bit byte value [0 .. 255].
 */
inline uint8_t gamma8inv(uint8_t val) {
  if (val == 0)
    return 0;
  // Standard internal inverse calculation: 1.0f / 2.8f ≈ 0.35714287f
  constexpr float gamma_inv = 0.35714287f;
  float normalized = (static_cast<float>(val) - 0.5f) / 255.0f;
  return static_cast<uint8_t>(powf(normalized, gamma_inv) * 255.0f + 0.5f);
}

// fast (true) random numbers using hardware RNG, all functions return values in the range lowerlimit to upperlimit-1
// note: for true random numbers with high entropy, do not call faster than every 200ns (5MHz)
// tests show it is still highly random reading it quickly in a loop (better than fastled PRNG)
// for 8bit and 16bit random functions: no limit check is done for best speed
// 32bit inputs are used for speed and code size, limits don't work if inverted or out of range
// inlining does save code size except for random(a,b) and 32bit random with limits
#ifdef ESP8266
#define HW_RND_REGISTER RANDOM_REG32
#else  // ESP32 family
#include "soc/wdev_reg.h"
#define HW_RND_REGISTER REG_READ(WDEV_RND_REG)
#endif
inline uint32_t hw_random() { return HW_RND_REGISTER; };
uint32_t hw_random(uint32_t upperlimit);  // not inlined for code size
int32_t hw_random(int32_t lowerlimit, int32_t upperlimit);
inline uint16_t hw_random16() { return HW_RND_REGISTER; };
inline uint16_t hw_random16(uint32_t upperlimit) {
  return (hw_random16() * upperlimit) >> 16;
};  // input range 0-65535 (uint16_t)
inline int16_t hw_random16(int32_t lowerlimit, int32_t upperlimit) {
  int32_t range = upperlimit - lowerlimit;
  return lowerlimit + hw_random16(range);
};  // signed limits, use int16_t ranges
inline uint8_t hw_random8() { return HW_RND_REGISTER; };
inline uint8_t hw_random8(uint32_t upperlimit) { return (hw_random8() * upperlimit) >> 8; };  // input range 0-255
inline uint8_t hw_random8(uint32_t lowerlimit, uint32_t upperlimit) {
  uint32_t range = upperlimit - lowerlimit;
  return lowerlimit + hw_random8(range);
};  // input range 0-255

#ifdef PALETTES

// *****************************************************************************************************************************************************************
inline CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness = 255) {
  return color_from_palette(index, CRGB(current_color.r, current_color.g, current_color.b), brightness);
}

// *****************************************************************************************************************************************************************
inline CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness = 255) {
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
// Protected static pointer to the modern container equalizer buffer
inline const uint8_t *g_fft_result_ptr = nullptr;

/**
 * @brief Registers the active pipeline equalizer array interface for downstream rendering routines.
 * @param array_ptr Pointer to the continuous 16-channel 8-bit array within the features structure.
 */
inline void register_fft_spectrum(const uint8_t *array_ptr) { g_fft_result_ptr = array_ptr; }

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
  CRGB value = CRGB::Black;  // Safe default fallback color
  CHSV hsv;

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
    hsv = CHSV(band_val, 255, uint8_t(map(band_val, 0, 255, 30, 255)));
    hsv2rgb_rainbow(hsv, value);
  }

  return value;
}
#endif  // MUSIC_LEDS

#endif  // PALETTES

}  // namespace esphome::fastled_helper
