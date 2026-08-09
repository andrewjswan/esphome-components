#include "colors.h"
#include "utils.h"

#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

// 8-bit standard fast alpha-blender
CRGB IRAM_ATTR color_blend(CRGB color1, CRGB color2, uint8_t blend) {
  if (blend == 0)
    return color1;
  if (blend == 255)
    return color2;

  uint32_t r_raw = (static_cast<uint32_t>(color2.r) * blend) + (static_cast<uint32_t>(color1.r) * (255 - blend));
  uint32_t g_raw = (static_cast<uint32_t>(color2.g) * blend) + (static_cast<uint32_t>(color1.g) * (255 - blend));
  uint32_t b_raw = (static_cast<uint32_t>(color2.b) * blend) + (static_cast<uint32_t>(color1.b) * (255 - blend));

  // Fast math approximation for clean division by 255
  return CRGB(static_cast<uint8_t>((r_raw + 1 + (r_raw >> 8)) >> 8),
              static_cast<uint8_t>((g_raw + 1 + (g_raw >> 8)) >> 8),
              static_cast<uint8_t>((b_raw + 1 + (b_raw >> 8)) >> 8));
}

// 16-bit high-resolution alpha-blender
CRGB IRAM_ATTR color_blend16(CRGB color1, CRGB color2, uint16_t blend) {
  if (blend == 0)
    return color1;
  if (blend == 0xFFFF)
    return color2;

  uint32_t r_raw = (static_cast<uint32_t>(color2.r) * blend) + (static_cast<uint32_t>(color1.r) * (0xFFFF - blend));
  uint32_t g_raw = (static_cast<uint32_t>(color2.g) * blend) + (static_cast<uint32_t>(color1.g) * (0xFFFF - blend));
  uint32_t b_raw = (static_cast<uint32_t>(color2.b) * blend) + (static_cast<uint32_t>(color1.b) * (0xFFFF - blend));

  // Fast math approximation for clean division by 65535
  return CRGB(static_cast<uint8_t>((r_raw + 1 + (r_raw >> 16)) >> 16),
              static_cast<uint8_t>((g_raw + 1 + (g_raw >> 16)) >> 16),
              static_cast<uint8_t>((b_raw + 1 + (b_raw >> 16)) >> 16));
}

CRGB IRAM_ATTR blend(const CRGB &p1, const CRGB &p2, uint8_t amountOfP2) { return color_blend(p1, p2, amountOfP2); }

void IRAM_ATTR nblend(CRGB &existing, const CRGB &overlay, uint8_t amountOfOverlay) {
  existing = color_blend(existing, overlay, amountOfOverlay);
}

void IRAM_ATTR nscale8_video(CRGB *leds, uint16_t num_leds, uint8_t scale) {
  for (uint16_t i = 0; i < num_leds; ++i) {
    leds[i].nscale8_video(scale);
  }
}

void IRAM_ATTR nscale8(CRGB *leds, uint16_t num_leds, uint8_t scale) {
  for (uint16_t i = 0; i < num_leds; ++i) {
    leds[i].nscale8(scale);
  }
}

/**
 * @brief Computes the inverse gamma correction for a single 8-bit color channel value.
 * @details Replicates the original mathematical transformation curve using a standard 2.8 gamma scale factor.
 * @param val The incoming linear 8-bit channel intensity value [0 .. 255].
 * @return The inverse gamma corrected 8-bit byte value [0 .. 255].
 */
uint8_t IRAM_ATTR gamma8inv(uint8_t val) {
  if (val == 0)
    return 0;

  constexpr float gamma_correct_val = (GAMMA_CORRECT > 0.0f) ? static_cast<float>(GAMMA_CORRECT) : 2.8f;
  constexpr float gamma_inv = 1.0f / gamma_correct_val;

  float normalized = (static_cast<float>(val) - 0.5f) / 255.0f;
  return static_cast<uint8_t>(powf(normalized, gamma_inv) * 255.0f + 0.5f);
}

/// Inline version of hsv2rgb_spectrum which returns a CRGB object.
CRGB hsv2rgb_spectrum(const CHSV &hsv) {
  CRGB rgb;
  hsv2rgb_spectrum(hsv, rgb);
  return rgb;
}

void hsv2rgb_spectrum(const CHSV &hsv, CRGB &rgb) {
  CHSV hsv2(hsv);
  hsv2.hue = scale8(hsv2.hue, 191);
  hsv2rgb_raw(hsv2, rgb);
}

void hsv2rgb_spectrum(const CHSV *phsv, CRGB *prgb, int numLeds) {
  for (int i = 0; i < numLeds; ++i) {
    hsv2rgb_spectrum(phsv[i], prgb[i]);
  }
}

void hsv2rgb_raw(const CHSV *phsv, CRGB *prgb, int numLeds) {
  for (int i = 0; i < numLeds; ++i) {
    hsv2rgb_raw(phsv[i], prgb[i]);
  }
}

/// Apply dimming compensation to values
#define APPLY_DIMMING(X) (X)

/// Divide the color wheel into eight sections, 32 elements each
/// @todo Unused. Remove?
#define HSV_SECTION_6 (0x20)

/// Divide the color wheel into four sections, 64 elements each
/// @todo I believe this is mis-named, and should be HSV_SECTION_4
#define HSV_SECTION_3 (0x40)

void hsv2rgb_raw(const CHSV &hsv, CRGB &rgb) {
  // Convert hue, saturation and brightness ( HSV/HSB ) to RGB
  // "Dimming" is used on saturation and brightness to make
  // the output more visually linear.

  // Apply dimming curves
  uint8_t value = APPLY_DIMMING(hsv.val);  // cppcheck-suppress selfAssignment
  uint8_t saturation = hsv.sat;

  // The brightness floor is minimum number that all of
  // R, G, and B will be set to.
  uint8_t invsat = APPLY_DIMMING(255 - saturation);  // cppcheck-suppress selfAssignment
  uint8_t brightness_floor = (value * invsat) / 256;

  // The color amplitude is the maximum amount of R, G, and B
  // that will be added on top of the brightness_floor to
  // create the specific hue desired.
  uint8_t color_amplitude = value - brightness_floor;

  // Figure out which section of the hue wheel we're in,
  // and how far offset we are withing that section
  uint8_t section = hsv.hue / HSV_SECTION_3;  // 0..2
  uint8_t offset = hsv.hue % HSV_SECTION_3;   // 0..63

  uint8_t rampup = offset;                          // 0..63
  uint8_t rampdown = (HSV_SECTION_3 - 1) - offset;  // 63..0

  // We now scale rampup and rampdown to a 0-255 range -- at least
  // in theory, but here's where architecture-specific decsions
  // come in to play:
  // To scale them up to 0-255, we'd want to multiply by 4.
  // But in the very next step, we multiply the ramps by other
  // values and then divide the resulting product by 256.
  // So which is faster?
  //   ((ramp * 4) * othervalue) / 256
  // or
  //   ((ramp    ) * othervalue) /  64
  // It depends on your processor architecture.
  // On 8-bit AVR, the "/ 256" is just a one-cycle register move,
  // but the "/ 64" might be a multicycle shift process. So on AVR
  // it's faster do multiply the ramp values by four, and then
  // divide by 256.
  // On ARM, the "/ 256" and "/ 64" are one cycle each, so it's
  // faster to NOT multiply the ramp values by four, and just to
  // divide the resulting product by 64 (instead of 256).
  // Moral of the story: trust your profiler, not your insticts.

  // Since there's an AVR assembly version elsewhere, we'll
  // assume what we're on an architecture where any number of
  // bit shifts has roughly the same cost, and we'll remove the
  // redundant math at the source level:

  //  // scale up to 255 range
  //  //rampup *= 4; // 0..252
  //  //rampdown *= 4; // 0..252

  // compute color-amplitude-scaled-down versions of rampup and rampdown
  uint8_t rampup_amp_adj = (rampup * color_amplitude) / (256 / 4);
  uint8_t rampdown_amp_adj = (rampdown * color_amplitude) / (256 / 4);

  // add brightness_floor offset to everything
  uint8_t rampup_adj_with_floor = rampup_amp_adj + brightness_floor;
  uint8_t rampdown_adj_with_floor = rampdown_amp_adj + brightness_floor;

  if (section) {
    if (section == 1) {
      // section 1: 0x40..0x7F
      rgb.r = brightness_floor;
      rgb.g = rampdown_adj_with_floor;
      rgb.b = rampup_adj_with_floor;
    } else {
      // section 2; 0x80..0xBF
      rgb.r = rampup_adj_with_floor;
      rgb.g = brightness_floor;
      rgb.b = rampdown_adj_with_floor;
    }
  } else {
    // section 0: 0x00..0x3F
    rgb.r = rampdown_adj_with_floor;
    rgb.g = rampup_adj_with_floor;
    rgb.b = brightness_floor;
  }
}

/// Convert a fractional input into a constant
#define FIXFRAC8(N, D) (((N) * 256) / (D))

// This function is only an approximation, and it is not
// nearly as fast as the normal HSV-to-RGB conversion.
// See extended notes in the .h file.
CHSV rgb2hsv_approximate(const CRGB &rgb) {
  uint8_t r = rgb.r;
  uint8_t g = rgb.g;
  uint8_t b = rgb.b;
  uint8_t h, s, v;

  // find desaturation
  uint8_t desat = 255;
  if (r < desat)
    desat = r;
  if (g < desat)
    desat = g;
  if (b < desat)
    desat = b;

  // remove saturation from all channels
  r -= desat;
  g -= desat;
  b -= desat;

  // saturation is opposite of desaturation
  s = 255 - desat;

  if (s != 255) {
    // undo 'dimming' of saturation
    s = 255 - sqrt16_t((255 - s) * 256);
  }

  // at least one channel is now zero
  // if all three channels are zero, we had a
  // shade of gray.
  if ((r + g + b) == 0) {
    // we pick hue zero for no special reason
    return CHSV(0, 0, 255 - s);
  }

  // scale all channels up to compensate for desaturation
  if (s < 255) {
    if (s == 0)
      s = 1;
    uint32_t scaleup = 65535 / (s);
    r = ((uint32_t) (r) *scaleup) / 256;
    g = ((uint32_t) (g) *scaleup) / 256;
    b = ((uint32_t) (b) *scaleup) / 256;
  }

  uint16_t total = r + g + b;

  // scale all channels up to compensate for low values
  if (total < 255) {
    if (total == 0)
      total = 1;
    uint32_t scaleup = 65535 / (total);
    r = ((uint32_t) (r) *scaleup) / 256;
    g = ((uint32_t) (g) *scaleup) / 256;
    b = ((uint32_t) (b) *scaleup) / 256;
  }

  if (total > 255) {
    v = 255;
  } else {
    v = qadd8(desat, total);
    // undo 'dimming' of brightness
    if (v != 255)
      v = sqrt16_t(v * 256);
  }

  // since this wasn't a pure shade of gray,
  // the interesting question is what hue is it

  // start with which channel is highest
  // (ties don't matter)
  uint8_t highest = r;
  if (g > highest)
    highest = g;
  if (b > highest)
    highest = b;

  if (highest == r) {
    // Red is highest.
    // Hue could be Purple/Pink-Red,Red-Orange,Orange-Yellow
    if (g == 0) {
      // if green is zero, we're in Purple/Pink-Red
      h = (HUE_PURPLE + HUE_PINK) / 2;
      h += scale8(qsub8(r, 128), FIXFRAC8(48, 128));
    } else if ((r - g) > g) {
      // if R-G > G then we're in Red-Orange
      h = HUE_RED;
      h += scale8(g, FIXFRAC8(32, 85));
    } else {
      // R-G < G, we're in Orange-Yellow.
      //
      // This used to read `(g - 85) + (171 - r)`. Reaching this branch
      // requires g >= r/2, and r is the largest channel, so r > 171 is
      // the normal case here -- making (171 - r) negative. The sum was
      // then truncated to u8 by qsub8() and wrapped to a large value,
      // sending orange out into the greens: rgb(255,153,0) returned hue
      // 122 instead of ~26. See issue #436.
      //
      // (255 - r) is the same measure of "how far r has fallen" but
      // cannot go negative, and the sum (g - 85) + (255 - r) has a
      // minimum of ~42 over this branch's own entry condition, so the
      // wrap is structurally impossible rather than merely unlikely.
      // Spans ~42..170, hence FIXFRAC8(32,170) to land the top of the
      // range on HUE_YELLOW.
      h = HUE_ORANGE;
      h += scale8(qsub8((g - 85) + (255 - r), 4), FIXFRAC8(32, 170));
    }

  } else if (highest == g) {
    // Green is highest
    // Hue could be Yellow-Green, Green-Aqua
    if (b == 0) {
      // if Blue is zero, we're in Yellow-Green
      //   G = 171..255
      //   R = 171..  0
      h = HUE_YELLOW;
      uint8_t radj = scale8(qsub8(171, r), 47);  // 171..0 -> 0..171 -> 0..31
      uint8_t gadj = scale8(qsub8(g, 171), 96);  // 171..255 -> 0..84 -> 0..31;
      uint8_t rgadj = radj + gadj;
      uint8_t hueadv = rgadj / 2;
      h += hueadv;
      // h += scale8( qadd8( 4, qadd8((g - 128), (128 - r))),
      //              FIXFRAC8(32,255)); //
    } else {
      // if Blue is nonzero we're in Green-Aqua
      if ((g - b) > b) {
        h = HUE_GREEN;
        h += scale8(b, FIXFRAC8(32, 85));
      } else {
        h = HUE_AQUA;
        h += scale8(qsub8(b, 85), FIXFRAC8(8, 42));
      }
    }

  } else /* highest == b */ {
    // Blue is highest
    // Hue could be Aqua/Blue-Blue, Blue-Purple, Purple-Pink
    if (r == 0) {
      // if red is zero, we're in Aqua/Blue-Blue
      h = HUE_AQUA + ((HUE_BLUE - HUE_AQUA) / 4);
      h += scale8(qsub8(b, 128), FIXFRAC8(24, 128));
    } else if ((b - r) > r) {
      // B-R > R, we're in Blue-Purple
      h = HUE_BLUE;
      h += scale8(r, FIXFRAC8(32, 85));
    } else {
      // B-R < R, we're in Purple-Pink
      h = HUE_PURPLE;
      h += scale8(qsub8(r, 85), FIXFRAC8(32, 85));
    }
  }

  h += 1;
  return CHSV(h, s, v);
}

}  // namespace esphome::fastled_helper
