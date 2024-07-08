//
// Created by oaleksander on 07.04.2023.
//

#ifndef ARKSTONE_UNIVERSAL_PUCKCOLLECTCOMMONS_H
#define ARKSTONE_UNIVERSAL_PUCKCOLLECTCOMMONS_H

#include <Arduino.h>

#define MICROS_IN_SECOND 1000000.0

void __empty() {}

enum Color {
  COLOR_BLUE = -1,
  COLOR_RED = 1,
  COLOR_NONE = 0
};

#include <FastLED.h>

CRGB colorToCode(Color color){
switch (color) {
        case COLOR_RED:
            return(CRGB::Red);
        case COLOR_BLUE:
            return(CRGB::Blue);
        case COLOR_NONE:
            return(CRGB::Black);
    }
}

uint8_t colorToIndex(Color color) {
  switch (color) {
    case COLOR_RED:
      return 0;
    case COLOR_BLUE:
      return 1;
    case COLOR_NONE:
      return 2;
  }
}

Color indexToColor(uint8_t index) {
  switch (index) {
    case 0:
      return COLOR_RED;
    case 1:
      return COLOR_BLUE;
    default:
      return COLOR_NONE;
  }
}


Color inverseColor(Color color) {
  if (color == COLOR_RED)
    return COLOR_BLUE;
  else if (color == COLOR_BLUE)
    return COLOR_RED;
  return COLOR_NONE;
}

double distance3d(double x1, double y1, double z1, double x2, double y2, double z2) {
  double xd = x1 - x2;
  double yd = y1 - y2;
  double zd = z1 - z2;
  return sqrt(xd * xd + yd * yd + zd * zd);
}

Color matchColor(double r_match, double g_match, double b_match, double r_red_ref, double g_red_ref, double b_red_ref,
                 double r_blue_ref, double g_blue_ref, double b_blue_ref, double r_white_ref, double g_white_ref,
                 double b_white_ref) {
  double whiteDistance = distance3d(r_match, g_match, b_match, r_white_ref, g_white_ref, b_white_ref);
  double redDistance = distance3d(r_match, g_match, b_match, r_red_ref, g_red_ref, b_red_ref);
  double blueDistance = distance3d(r_match, g_match, b_match, r_blue_ref, g_blue_ref, b_blue_ref);
  if (redDistance < whiteDistance && redDistance < blueDistance)
    return COLOR_RED;
  if (blueDistance < whiteDistance && blueDistance < redDistance)
    return COLOR_BLUE;
  if (whiteDistance < redDistance && whiteDistance < blueDistance)
    return COLOR_NONE;
  return COLOR_NONE;
}

#endif  //ARKSTONE_UNIVERSAL_PUCKCOLLECTCOMMONS_H
