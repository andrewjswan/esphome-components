#include "fill.h"
#include "utils.h"
#include "fastled_slim.h"

#include "esphome/core/defines.h"

namespace esphome::fastled_helper {

void fill_solid(CRGB *targetArray, int numToFill,
                const CRGB &color) {
    for (int i = 0; i < numToFill; ++i) {
        targetArray[i] = color;
    }
}

void fill_solid(CHSV *targetArray, int numToFill,
                const CHSV &color) {
    for (int i = 0; i < numToFill; ++i) {
        targetArray[i] = color;
    }
}

void fill_raw_noise8(uint8_t *pData, uint8_t num_points, uint8_t octaves, uint16_t x, int scale, uint16_t time) {
    uint32_t _xx = x;
    uint32_t scx = scale;
    for (int o = 0; o < octaves; ++o) {
        for (int i = 0, xx = _xx; i < num_points; ++i, xx += scx) {
            pData[i] = qadd8(pData[i], inoise8(xx, time) >> o);
        }
        _xx <<= 1;
        scx <<= 1;
    }
}

void fill_raw_noise16into8(uint8_t *pData, uint8_t num_points, uint8_t octaves, uint32_t x, int scale, uint32_t time) {
    uint32_t _xx = x;
    uint32_t scx = scale;
    for (int o = 0; o < octaves; ++o) {
        for (int i = 0, xx = _xx; i < num_points; ++i, xx += scx) {
            pData[i] = qadd8(pData[i], (inoise16(xx, time) >> 8) >> o);
        }
        _xx <<= 1;
        scx <<= 1;
    }
}

void fill_noise8(CRGB *leds, int num_leds,
            uint8_t octaves, uint16_t x, int scale,
            uint8_t hue_octaves, uint16_t hue_x, int hue_scale,
            uint16_t time) {
 
    if (num_leds <= 0) return;
 
    for (int j = 0; j < num_leds; j += 255) {
        const int LedsRemaining = num_leds - j;
        const int LedsPer = LedsRemaining > 255 ? 255 : LedsRemaining;
 
        if (LedsPer <= 0) continue;

        uint8_t V[LedsPer];
        uint8_t H[LedsPer];
 
        memset(V, 0, LedsPer);
        memset(H, 0, LedsPer);
 
        fill_raw_noise8(V, LedsPer, octaves, x, scale, time);
        fill_raw_noise8(H, LedsPer, hue_octaves, hue_x, hue_scale, time);
 
        for (int i = 0; i < LedsPer; ++i) {
            leds[i + j] = CHSV(H[i], 255, V[i]);
        }
    }
}
 
void fill_noise16(CRGB *leds, int num_leds,
            uint8_t octaves, uint16_t x, int scale,
            uint8_t hue_octaves, uint16_t hue_x, int hue_scale,
            uint16_t time, uint8_t hue_shift) {
 
    if (num_leds <= 0) return;
 
    for (int j = 0; j < num_leds; j += 255) {
        const int LedsRemaining = num_leds - j;
        const int LedsPer = LedsRemaining > 255 ? 255 : LedsRemaining;
        if (LedsPer <= 0) continue;

        uint8_t V[LedsPer];
        uint8_t H[LedsPer];
 
        memset(V, 0, LedsPer);
        memset(H, 0, LedsPer);
 
        fill_raw_noise16into8(V, LedsPer, octaves, x, scale, time);
        fill_raw_noise8(H, LedsPer, hue_octaves, hue_x, hue_scale, time);
 
        for (int i = 0; i < LedsPer; ++i) {
            leds[i + j] = CHSV(H[i] + hue_shift, 255, V[i]);
        }
    }
}

void fill_rainbow(CRGB *targetArray, int numToFill, uint8_t initialhue, uint8_t deltahue) {
    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;
    for (int i = 0; i < numToFill; ++i) {
        targetArray[i] = hsv;
        hsv.hue += deltahue;
    }
}
 
void fill_rainbow(CHSV *targetArray, int numToFill, uint8_t initialhue, uint8_t deltahue) {
    CHSV hsv;
    hsv.hue = initialhue;
    hsv.val = 255;
    hsv.sat = 240;
    for (int i = 0; i < numToFill; ++i) {
        targetArray[i] = hsv;
        hsv.hue += deltahue;
    }
}

}  // namespace esphome::fastled_helper
