//
// Created by oaleksander on 07.04.2023.
//

#ifndef ARKSTONE_UNIVERSAL_FIELDCOLORSENSOR_H
#define ARKSTONE_UNIVERSAL_FIELDCOLORSENSOR_H

#include <Arduino.h>
#include <Prizm_Controller.h>
#include "PuckCollectCommons.h"
#include "Stopwatch.h"

static Color currentFieldColor = COLOR_NONE;

#if FIELD_SENSOR == FIELD_SENSOR_HITECHNIC
#include <HiTechnicColorV2_Arduino.h>
#elif FIELD_SENSOR == FIELD_SENSOR_TCS34725_SOFTI2C
#include <Adafruit_TCS34725softi2c.h>
Adafruit_TCS34725softi2c fieldTcs = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X, FIELD_SENSOR_TCS34725_SOFTI2C_SDA, FIELD_SENSOR_TCS34725_SOFTI2C_SCL);
#else
#error Unknown field sensor type
#endif  //FIELD_SENSOR

void readFieldColorSensor(float* r_float, float* g_float, float* b_float) {
  *r_float = 0;
  *g_float = 0;
  *b_float = 0;
#if FIELD_SENSOR == FIELD_SENSOR_HITECHNIC
  HTCS2readRGB((uint8_t*)r_float, (uint8_t*)g_float, (uint8_t*)b_float);
  *r_float = (float)(*(uint8_t*)r_float);
  *g_float = (float)(*(uint8_t*)g_float);
  *b_float = (float)(*(uint8_t*)b_float);
  double ampl = sqrt(*r_float * *r_float + *g_float * *g_float + *b_float * *b_float);
  if (ampl != 0) {
    *r_float /= ampl;
    *g_float /= ampl;
    *b_float /= ampl;
  }
#elif FIELD_SENSOR == FIELD_SENSOR_TCS34725_SOFTI2C
  fieldTcs.getRGB(r_float, g_float, b_float);
#else
#endif  //FIELD_SENSOR
  
#ifdef DEBUG_FIELD_COLOR_SENSOR
  Serial.print("field sensor r:");
  Serial.print(*r_float);
  Serial.print("g:");
  Serial.print(*g_float);
  Serial.print("b:");
  Serial.println(*b_float);
#endif
  /*
  if (r_float < 1e-4 || g_float < 1e-4 || b_float < 1e-4)
    return COLOR_NONE;
  return matchColor(r_float, g_float, b_float, REDFIELD_R, REDFIELD_G, REDFIELD_B, BLUEFIELD_R, BLUEFIELD_G, BLUEFIELD_B, WHITEFIELD_R,
                    WHITEFIELD_G, WHITEFIELD_B,
                    FIELD_COLOR_DISTANCE_THRESHOLD);
                    */
}

void readFieldColorSensorNonZero(float* r_float, float* g_float, float* b_float) {
  float r, g, b;
  do {
    readFieldColorSensor(&r, &g, &b);
  } while (abs(r) < 1e-5 && abs(g) < 1e-5 && abs(b) < 1e-5);
  *r_float = r;
  *g_float = g;
  *b_float = b;
}

Stopwatch fieldSensorColorDetectionTimer = Stopwatch();

Color lastReadFieldColor = COLOR_NONE;
#ifdef FORCE_TEAM_COLOR
Color teamFieldColor = FORCE_TEAM_COLOR;
#else
Color teamFieldColor = COLOR_NONE;
#endif
Color enemyFieldColor = COLOR_NONE;
#define robotIsOnTeamField (currentFieldColor == teamFieldColor) && (teamFieldColor != COLOR_NONE)
#define robotProbablyOnTeamField (lastReadFieldColor == teamFieldColor) && (teamFieldColor != COLOR_NONE)


Color getFieldColor() {
  float r, g, b;
  readFieldColorSensor(&r, &g, &b);
  
  if (r == 0 && g == 0 && b == 0)
    lastReadFieldColor = COLOR_NONE;
  else lastReadFieldColor = matchColor(r, g, b, REDFIELD_R, REDFIELD_G, REDFIELD_B, BLUEFIELD_R, BLUEFIELD_G, BLUEFIELD_B, WHITEFIELD_R,
                               WHITEFIELD_G, WHITEFIELD_B);
  if (lastReadFieldColor != teamFieldColor) {
    fieldSensorColorDetectionTimer.reset();
  }
  if (fieldSensorColorDetectionTimer.milliseconds() > TEAM_FIELD_DETECTION_TIME_THRESHOLD_MS)
    return lastReadFieldColor;
  else
    return COLOR_NONE;
}




void initFieldColorSensor() {
#if FIELD_SENSOR == FIELD_SENSOR_HITECHNIC
  HTCS2setLightFrequency(FREQUENCY_50HZ);
#elif FIELD_SENSOR == FIELD_SENSOR_TCS34725_SOFTI2C
  fieldTcs.begin();
#else
#endif  //FIELD_SENSOR
  float dummy;
  readFieldColorSensorNonZero(&dummy, &dummy, &dummy);
}

Color detectTeamFieldColor() {
  float r_team = .0, g_team = .0, b_team = .0;
#ifdef SERIAL_DEBUG
  Serial.println("Detecting team field");
#endif
  for (size_t i = 0; i < TEAM_COLOR_DETECTION_N_SAMPLES; i++) {
    float r_sample = .0, g_sample = .0, b_sample = .0;
    readFieldColorSensorNonZero(&r_sample, &g_sample, &b_sample);
    r_team += r_sample;
    g_team += g_sample;
    b_team += b_sample;
    delay(TEAM_COLOR_DETECTION_DELAY_MS);
  }
  r_team /= TEAM_COLOR_DETECTION_N_SAMPLES;
  g_team /= TEAM_COLOR_DETECTION_N_SAMPLES;
  b_team /= TEAM_COLOR_DETECTION_N_SAMPLES;
  float redFieldDistance = distance3d(r_team, g_team, b_team, REDFIELD_R, REDFIELD_G, REDFIELD_B);
  float blueFieldDistance = distance3d(r_team, g_team, b_team, BLUEFIELD_R, BLUEFIELD_G, BLUEFIELD_B);
  if (redFieldDistance < blueFieldDistance) {
    teamFieldColor = COLOR_RED;
    REDFIELD_R = r_team;
    REDFIELD_G = g_team;
    REDFIELD_B = b_team;
  } else {
    teamFieldColor = COLOR_BLUE;
    BLUEFIELD_R = r_team;
    BLUEFIELD_G = g_team;
    BLUEFIELD_B = b_team;
  }
#ifdef DEBUG_FIELD_COLOR_SENSOR
  Serial.print("(");
  Serial.print(r_team);
  Serial.print(",");
  Serial.print(g_team);
  Serial.print(",");
  Serial.print(b_team);
  Serial.print(") ");
  Serial.print("Team field:");
  Serial.println(teamFieldColor);
#endif
  enemyFieldColor = inverseColor(teamFieldColor);
}

Stopwatch fieldSensorUpdateTimer = Stopwatch();

void updateFieldColor() {
  if (fieldSensorUpdateTimer.milliseconds() > FIELD_SENSOR_UPDATE_PERIOD_MS) {
    currentFieldColor = getFieldColor();
#ifdef SERIAL_DEBUG
    Serial.print("field ");
    Serial.println(currentFieldColor);
#endif
    fieldSensorUpdateTimer.reset();
  }
}

#endif  //ARKSTONE_UNIVERSAL_FIELDCOLORSENSOR_H