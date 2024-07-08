#ifndef ARKSTONE_UNIVERSAL_LIGHT_H
#define ARKSTONE_UNIVERSAL_LIGHT_H

#include <FastLED.h>

#include "RobotConfig.h"

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
    leds[ledIndex + startIndex] = color;
    FastLED.show();
}
#endif  // ARGB_LED

void fillLight(const CRGB color) {
#ifdef ARGB_LED
    fill_solid(leds, LED_NUM, color);
    FastLED.show();
#endif  // ARGB_LED
}

void fillOddEven(const CRGB oddColor, const CRGB evenColor) {
#ifdef ARGB_LED
    for (size_t i = 0; i < LED_NUM; i++)
        leds[i] = i % 2 ? oddColor : evenColor;
    FastLED.show();
#endif  // ARGB_LED
}

void fillRainbow(uint8_t offset = 0) {
#ifdef ARGB_LED
    fill_rainbow(leds, LED_NUM, offset, 0xFF / (LED_NUM - 2));
    FastLED.show();
#endif  // ARGB_LED
}

void blinkLights(const CRGB color1, const CRGB color2, uint32_t period, uint32_t start = 0) {
    if (!start)
        start = period >> 1;
    fillLight(millis() % period > start ? color1 : color2);
}

void blinkLightsOddEven(const CRGB color1, const CRGB color2, uint32_t period, uint32_t start = 0) {
    if (!start)
        start = period >> 1;
    if (millis() % period > start)
        fillLight(color1);
    else
        fillOddEven(color1, color2);
}

#endif  // ARKSTONE_UNIVERSAL_LIGHT_H
