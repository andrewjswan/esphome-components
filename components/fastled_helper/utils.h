#pragma once

#include "fastled_slim.h"
#include "palettes.h"

#include "esphome/core/color.h"
#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

template<typename T, size_t N>
__attribute__((always_inline)) inline constexpr size_t array_size(const T (&)[N]) noexcept {
  return N;
}

// Analog Unsigned subtraction macro. if result < 0, then => 0
template<typename T> __attribute__((always_inline)) inline T qsuba(T i, T j) { return i > j ? i - j : 0; }

// Digital unsigned subtraction macro. if result <0, then => 0. Otherwise, take on fixed value.
template<typename T> __attribute__((always_inline)) inline T qsubd(T i, T j) { return i > j ? j : 0; }

// *****************************************************************************************************************************************************************
// Global pointer to the LED array
inline CRGB *leds{nullptr};

inline void init_leds(int size) {
  if (leds == nullptr) {
    leds = new CRGB[size];
  }
}

inline void free_leds() {
  if (leds != nullptr) {
    delete[] leds;
    leds = nullptr;
  }
}

// *****************************************************************************************************************************************************************
void fade_out(CRGB *physic_leds, uint16_t _leds_num, uint8_t rate, CRGB back_color);
void blur(CRGB *physic_leds, uint16_t numLeds, uint8_t blur_amount);

void fadeLightBy(CRGB *leds, uint16_t num_leds, uint8_t fadeBy);
void fadeToBlackBy(CRGB *leds, uint16_t num_leds, uint8_t fadeBy);

inline uint16_t map8_to_16(uint8_t x) { return (uint16_t) ((x << 8) | x); }

int16_t sin16_t(uint16_t theta);
int16_t cos16_t(uint16_t theta);
uint8_t sin8_t(uint8_t theta);
uint8_t cos8_t(uint8_t theta);

uint8_t sqrt8_t(uint8_t x);
uint8_t sqrt16_t(uint16_t x);

uint16_t beat88(uint16_t beats_per_minute_88, uint32_t timebase = 0);
uint16_t beat16(uint16_t beats_per_minute, uint32_t timebase = 0);
uint8_t beat8(uint16_t beats_per_minute, uint32_t timebase = 0);

uint16_t beatsin88_t(uint16_t beats_per_minute_88, uint16_t lowest = 0, uint16_t highest = 65535, uint32_t timebase = 0,
                     uint16_t phase_offset = 0);
uint16_t beatsin16_t(uint16_t beats_per_minute, uint16_t lowest = 0, uint16_t highest = 65535, uint32_t timebase = 0,
                     uint16_t phase_offset = 0);
uint8_t beatsin8_t(uint16_t beats_per_minute, uint8_t lowest = 0, uint8_t highest = 255, uint32_t timebase = 0,
                   uint8_t phase_offset = 0);
uint8_t beatcos8_t(uint16_t beats_per_minute, uint8_t lowest = 0, uint8_t highest = 255, uint32_t timebase = 0,
                   uint8_t phase_offset = 0);
#define beatsin88 beatsin88_t
#define beatsin16 beatsin16_t
#define beatsin8 beatsin8_t
#define beatcos8 beatcos8_t

int32_t perlin1D_raw(uint32_t x, bool is16bit = false);
int32_t perlin2D_raw(uint32_t x, uint32_t y, bool is16bit = false);
int32_t perlin3D_raw(uint32_t x, uint32_t y, uint32_t z, bool is16bit = false);
uint16_t perlin16(uint32_t x);
uint16_t perlin16(uint32_t x, uint32_t y);
uint16_t perlin16(uint32_t x, uint32_t y, uint32_t z);
uint8_t perlin8(uint16_t x);
uint8_t perlin8(uint16_t x, uint16_t y);
uint8_t perlin8(uint16_t x, uint16_t y, uint16_t z);
#define inoise8 perlin8    // fastled legacy alias
#define inoise16 perlin16  // fastled legacy alias

#ifdef ESP8266
#define HW_RND_REGISTER RANDOM_REG32
#else  // ESP32 family
#include "soc/wdev_reg.h"
#define HW_RND_REGISTER REG_READ(WDEV_RND_REG)
#endif

// fast (true) random numbers using hardware RNG, all functions return values in the range lowerlimit to upperlimit-1
// note: for true random numbers with high entropy, do not call faster than every 200ns (5MHz)
// tests show it is still highly random reading it quickly in a loop (better than fastled PRNG)
// for 8bit and 16bit random functions: no limit check is done for best speed
// 32bit inputs are used for speed and code size, limits don't work if inverted or out of range
// inlining does save code size except for random(a,b) and 32bit random with limits

inline uint32_t hw_random() { return HW_RND_REGISTER; };
uint32_t hw_random(uint32_t upperlimit);
int32_t hw_random(int32_t lowerlimit, int32_t upperlimit);
inline uint32_t random32() { return hw_random(); }
inline uint32_t random32(uint32_t upperlimit) { return hw_random(upperlimit); }
inline int32_t random32(uint32_t lowerlimit, uint32_t upperlimit) { return hw_random(lowerlimit, upperlimit); }

inline uint16_t hw_random16() { return HW_RND_REGISTER; };
inline uint16_t hw_random16(uint32_t upperlimit) {
  return (hw_random16() * upperlimit) >> 16;
};  // input range 0-65535 (uint16_t)
inline int16_t hw_random16(int32_t lowerlimit, int32_t upperlimit) {
  int32_t range = upperlimit - lowerlimit;
  return lowerlimit + hw_random16(range);
};  // signed limits, use int16_t ranges
inline uint16_t random16() { return hw_random16(); }
inline uint16_t random16(uint32_t upperlimit) { return hw_random16(upperlimit); }
inline int16_t random16(uint32_t lowerlimit, uint32_t upperlimit) { return hw_random16(lowerlimit, upperlimit); }

inline uint8_t hw_random8() { return HW_RND_REGISTER; };
inline uint8_t hw_random8(uint32_t upperlimit) { return (hw_random8() * upperlimit) >> 8; };  // input range 0-255
inline uint8_t hw_random8(uint32_t lowerlimit, uint32_t upperlimit) {
  uint32_t range = upperlimit - lowerlimit;
  return lowerlimit + hw_random8(range);
};  // input range 0-255
inline uint8_t random8() { return hw_random8(); }
inline uint8_t random8(uint32_t upperlimit) { return hw_random8(upperlimit); }
inline uint8_t random8(uint32_t lowerlimit, uint32_t upperlimit) { return hw_random8(lowerlimit, upperlimit); }

inline __attribute__((always_inline)) static uint8_t dim8_raw(uint8_t x) { return scale8(x, x); }

/// Map from one full-range 8-bit value into a narrower
/// range of 8-bit values, possibly a range of hues.
///
/// E.g. map `myValue` into a hue in the range blue..purple..pink..red
///   @code
///   hue = map8( myValue, HUE_BLUE, HUE_RED);
///   @endcode
///
/// Combines nicely with the waveform functions (like sin8(), etc)
/// to produce continuous hue gradients back and forth:
///   @code
///   hue = map8( sin8( myValue), HUE_BLUE, HUE_RED);
///   @endcode
///
/// Mathematically simiar to lerp8by8(), but arguments are more
/// like Arduino's "map"; this function is similar to
///   @code
///   map( in, 0, 255, rangeStart, rangeEnd)
///   @endcode
///
/// but faster and specifically designed for 8-bit values.
inline __attribute__((always_inline)) static uint8_t map8(uint8_t in, uint8_t rangeStart, uint8_t rangeEnd) {
  uint8_t rangeWidth = rangeEnd - rangeStart;
  uint8_t out = scale8(in, rangeWidth);
  out += rangeStart;
  return out;
}

/// Fast, rough 8-bit ease-in/ease-out function.
/// Shaped approximately like ease8InOutCubic(),
/// it's never off by more than a couple of percent
/// from the actual cubic S-curve, and it executes
/// more than twice as fast.  Use when the cycles
/// are more important than visual smoothness.
/// Asm version takes around 7 cycles on AVR.
inline __attribute__((always_inline)) static uint8_t ease8InOutApprox(uint8_t i) {
  if (i < 64) {
    // start with slope 0.5
    i /= 2;
  } else if (i > (255 - 64)) {
    // end with slope 0.5
    i = 255 - i;
    i /= 2;
    i = 255 - i;
  } else {
    // in the middle, use slope 192/128 = 1.5
    i -= 64;
    i += (i / 2);
    i += 32;
  }

  return i;
}

CRGB color_from_palette(const CRGBPalette16 &pal, unsigned index, uint8_t brightness = 255,
                        TBlendType blendType = LINEARBLEND);
#ifdef USE_PALETTES

inline uint8_t current_palette = 4;
inline CRGBPalette16 randomPalette;
inline uint32_t randomPaletteChange = 0;

CRGB color_from_palette(int index, CRGB current_color, uint8_t brightness = 255);
CRGB color_from_palette(int index, esphome::Color current_color, uint8_t brightness = 255);

#ifdef USE_MUSIC_LEDS  // MUSIC_LEDS

// Protected static pointer to the modern container equalizer buffer
inline const uint8_t *g_fft_result_ptr = nullptr;

/**
 * @brief Registers the active pipeline equalizer array interface for downstream rendering routines.
 * @param array_ptr Pointer to the continuous 16-channel 8-bit array within the features structure.
 */
inline void register_fft_spectrum(const uint8_t *array_ptr) { g_fft_result_ptr = array_ptr; }

// *****************************************************************************************************************************************************************
CRGBPalette16 get_audio_palette(int pal);
CRGB get_CRGB_for_band(int x, int pal);

#endif  // MUSIC_LEDS

#endif  // PALETTES

}  // namespace esphome::fastled_helper
