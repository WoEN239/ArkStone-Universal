#ifndef ARKSTONE_UNIVERSAL_LIGHT_H
#define ARKSTONE_UNIVERSAL_LIGHT_H

#include "RobotConfig.h"
#include <FastLED.h>

#ifdef ARGB_LED
CRGB leds[LED_NUM];
#endif  // ARGB_LED

void initLight() {
#ifdef ARGB_LED
    FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, LED_NUM);
#endif  // ARGB_LED
}

#ifdef ARGB_LED
void setLedColor(size_t ledIndex, const CRGB color, size_t startIndex = 0) {
  leds[ledIndex+startIndex] = color;
  FastLED.show();
}
#endif  // ARGB_LED

void fillLight(const CRGB color) {
#ifdef ARGB_LED
    fill_solid(leds, LED_NUM, color);
    FastLED.show();
#endif  // ARGB_LED
}

#endif  // ARKSTONE_UNIVERSAL_LIGHT_H
