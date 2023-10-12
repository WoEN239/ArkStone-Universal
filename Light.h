#ifndef ARKSTONE_UNIVERSAL_LIGHT_H
#define ARKSTONE_UNIVERSAL_LIGHT_H

#include <FastLED.h>

#define NUM_LEDS 17
#define DATA_PIN 4

CRGB leds[NUM_LEDS];

void initLight() { 
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
}

void showLight(const CRGB color) { 
  fill_solid(leds,NUM_LEDS,color);
  FastLED.show();
}

#endif //ARKSTONE_UNIVERSAL_LIGHT_H
