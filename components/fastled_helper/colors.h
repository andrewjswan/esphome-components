#pragma once

#include "fastled_slim.h"

#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

[[gnu::hot, gnu::pure]]
CRGB color_blend(CRGB color1, CRGB color2, uint8_t blend);
[[gnu::hot, gnu::pure]]
CRGB color_blend16(CRGB color1, CRGB color2, uint16_t blend);

CRGB blend(const CRGB &p1, const CRGB &p2, uint8_t amountOfP2);
void nblend(CRGB &existing, const CRGB &overlay, uint8_t amountOfOverlay);

void nscale8_video(CRGB *leds, uint16_t num_leds, uint8_t scale);
void nscale8(CRGB *leds, uint16_t num_leds, uint8_t scale);

uint8_t gamma8inv(uint8_t val);

/// The "video" version of scale8()
inline __attribute__((always_inline)) uint8_t scale8_video(uint8_t i, uint8_t scale) {
    uint8_t j = (((int)i * (int)scale) >> 8) + ((i && scale) ? 1 : 0);
    return j;
}

inline __attribute__((always_inline)) static uint8_t dim8_video(uint8_t x) { return scale8_video(x, x); }

/// Pre-defined hue values for hsv8 objects
typedef enum {
    HUE_RED = 0,       ///< Red (0°)
    HUE_ORANGE = 32,   ///< Orange (45°)
    HUE_YELLOW = 64,   ///< Yellow (90°)
    HUE_GREEN = 96,    ///< Green (135°)
    HUE_AQUA = 128,    ///< Aqua (180°)
    HUE_BLUE = 160,    ///< Blue (225°)
    HUE_PURPLE = 192,  ///< Purple (270°)
    HUE_PINK = 224     ///< Pink (315°)
} HSVHue;

/// Convert an HSV value to RGB using a mathematically straight spectrum.
/// This "spectrum" will have more green and blue than a "rainbow",
/// and less yellow and orange.
///
/// ![FastLED 'Spectrum' Hue Chart](https://raw.githubusercontent.com/FastLED/FastLED/gh-pages/images/HSV-spectrum-with-desc.jpg)
///
/// @note This function wraps hsv2rgb_raw() and rescales the hue value to fit
/// the smaller range.
///
/// @param hsv CHSV struct to convert to RGB. Max hue supported is HUE_MAX_SPECTRUM
/// @param rgb CRGB struct to store the result of the conversion (will be modified)
void hsv2rgb_spectrum(const CHSV& hsv, CRGB& rgb);

/// Inline version of hsv2rgb_spectrum which returns a CRGB object.
CRGB hsv2rgb_spectrum(const CHSV& hsv);

/// @copybrief hsv2rgb_spectrum(const CHSV&, CRGB&)
/// @see hsv2rgb_spectrum(const CHSV&, CRGB&)
/// @param phsv CHSV array to convert to RGB. Max hue supported is HUE_MAX_SPECTRUM
/// @param prgb CRGB array to store the result of the conversion (will be modified)
/// @param numLeds the number of array values to process
void hsv2rgb_spectrum(const CHSV* phsv, CRGB * prgb, int numLeds);

/// @copybrief hsv2rgb_spectrum(const CHSV&, CRGB&)
/// @see hsv2rgb_spectrum(const CHSV&, CRGB&)
/// @note The hue is limited to the range 0-191 (HUE_MAX). This
/// results in a slightly faster conversion speed at the expense
/// of color balance.
/// @param hsv CHSV struct to convert to RGB. Max hue supported is HUE_MAX
/// @param rgb CRGB struct to store the result of the conversion (will be modified)
void hsv2rgb_raw(const CHSV& hsv, CRGB & rgb);

/// @copybrief hsv2rgb_raw(const CHSV&, CRGB&)
/// @see hsv2rgb_raw(const CHSV&, CRGB&)
/// @param phsv CHSV array to convert to RGB. Max hue supported is HUE_MAX
/// @param prgb CRGB array to store the result of the conversion (will be modified)
/// @param numLeds the number of array values to process
void hsv2rgb_raw(const CHSV* phsv, CRGB * prgb, int numLeds);

/// Max hue accepted for the hsv2rgb_spectrum() function
#define HUE_MAX_SPECTRUM 255

/// Recover approximate HSV values from RGB.
/// These values are *approximate*, not exact. Why is this "only" an approximation?
/// Because not all RGB colors have HSV equivalents!  For example, there
/// is no HSV value that will ever convert to RGB(255,255,0) using
/// the code provided in this library.   So if you try to
/// convert RGB(255,255,0) "back" to HSV, you'll necessarily get
/// only an approximation.  Emphasis has been placed on getting
/// the "hue" as close as usefully possible, but even that's a bit
/// of a challenge.  The 8-bit HSV and 8-bit RGB color spaces
/// are not a "bijection".
///
/// Nevertheless, this function does a pretty good job, particularly
/// at recovering the 'hue' from fully saturated RGB colors that
/// originally came from HSV rainbow colors.  So if you start
/// with CHSV(hue_in,255,255), and convert that to RGB, and then
/// convert it back to HSV using this function, the resulting output
/// hue will either exactly the same, or very close (+/-1).
/// The more desaturated the original RGB color is, the rougher the
/// approximation, and the less accurate the results.
/// @note This function is a long-term work in progress; expect
/// results to change slightly over time as this function is
/// refined and improved.
/// @par
/// @note This function is most accurate when the input is an
/// RGB color that came from a fully-saturated HSV color to start
/// with.  E.g. CHSV( hue, 255, 255) -> CRGB -> CHSV will give
/// best results.
/// @par
/// @note This function is not nearly as fast as HSV-to-RGB.
/// It is provided for those situations when the need for this
/// function cannot be avoided, or when extremely high performance
/// is not needed.
/// @see https://en.wikipedia.org/wiki/Bijection
/// @param rgb an RGB value to convert
/// @returns the approximate HSV equivalent of the RGB value
CHSV rgb2hsv_approximate(const CRGB& rgb);

}  // namespace esphome::fastled_helper
