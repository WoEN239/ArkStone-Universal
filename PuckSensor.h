#ifndef ARKSTONE_UNIVERSAL_PUCKSENSOR_H
#define ARKSTONE_UNIVERSAL_PUCKSENSOR_H

#include <Arduino.h>
#include "Prizm_Controller.h"
#include "PuckCollectCommons.h"
#include "Stopwatch.h"

#if PUCK_SENSOR_TYPE == PUCK_SENSOR_HITECHNIC
#include <HiTechnicColorV2_Arduino.h>
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_TCS34725_SOFTI2C
#include <Adafruit_TCS34725softi2c.h>
Adafruit_TCS34725softi2c separatorTcs = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X, PUCK_SENSOR_TCS34725_SOFTI2C_SDA, PUCK_SENSOR_TCS34725_SOFTI2C_SCL);
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_RGB
#include "UartColorSensor.h"
UartColorSensor uartPuckSensor = UartColorSensor(PUCK_SENSOR_UART_TX);
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_COLOR
#include <SoftwareSerial.h>
SoftwareSerial uartPuckSensor = SoftwareSerial(PUCK_SENSOR_UART_TX, PUCK_SENSOR_UART_TX);
#else
#error Unknown puck sensor type
#endif  // FIELD_SENSOR

Color currentPuckColor = COLOR_NONE;
Color lastReadPuckColor = COLOR_NONE;
Stopwatch puckSensorDetectionTimer = Stopwatch();

#ifdef SEPARATOR_PROPELLER_DETECTION
bool puckSensorAtPropeller = false;
#endif //SEPARATOR_PROPELLER_DETECTION

void initPuckSensor() {
#if PUCK_SENSOR_TYPE == PUCK_SENSOR_HITECHNIC
  HTCS2setLightFrequency(FREQUENCY_50HZ);
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_TCS34725_SOFTI2C
  separatorTcs.begin();
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_RGB
  uartPuckSensor.begin();
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_COLOR
  uartPuckSensor.begin(UARTSENSOR_BAUDRATE);
#else
#error Unknown puck sensor type
#endif  // PUCK_SENSOR
}

#if PUCK_SENSOR_TYPE != PUCK_SENSOR_UART_COLOR
void readPuckColorSensor(float* r_float, float* g_float, float* b_float) {
  *r_float = 0;
  *g_float = 0;
  *b_float = 0;
#if PUCK_SENSOR_TYPE == PUCK_SENSOR_HITECHNIC
  HTCS2readRGB((uint8_t*)r_float, (uint8_t*)g_float, (uint8_t*)b_float);
  *r_float = (float)(*(uint8_t*)r_float);
  *g_float = (float)(*(uint8_t*)g_float);
  *b_float = (float)(*(uint8_t*)b_float);
#elif PUCK_SENSOR_TYPE == FIELD_SENSOR_TCS34725_SOFTI2C
  separatorTcs.getRGB(r_float, g_float, b_float);
#elif PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_RGB
  uartPuckSensor.update();
  *r_float = uartPuckSensor.dataOutput.r,
  *g_float = uartPuckSensor.dataOutput.g,
  *b_float = uartPuckSensor.dataOutput.b;
#endif  //PUCK_SENSOR
#ifdef DEBUG_PUCK_SENSOR
  Serial.print("puck sensor r:");
  Serial.print(*r_float);
  Serial.print("g:");
  Serial.print(*g_float);
  Serial.print("b:");
  Serial.println(*b_float);
#endif  //SERIAL_DEBUG
}
#endif  //PUCK_SENSOR

Stopwatch puckSensorUpdateTimer = Stopwatch();

void updatePuckSensor() {
  if (puckSensorUpdateTimer.milliseconds() > PUCK_SENSOR_UPDATE_PERIOD_MS) {
#if PUCK_SENSOR_TYPE == PUCK_SENSOR_UART_COLOR
    while (uartPuckSensor.available())
      currentPuckColor = uartPuckSensor.read();
#else
    float r, g, b;
    readPuckColorSensor(&r, &g, &b);
    lastReadPuckColor = matchColor(r, g, b, REDPUCK_R, REDPUCK_G, REDPUCK_B, BLUEPUCK_R, BLUEPUCK_G, BLUEPUCK_B, NOPUCK_R, NOPUCK_G, NOPUCK_B);
#ifdef SEPARATOR_PROPELLER_DETECTION
    puckSensorAtPropeller = distance3d(r, g, b, SEPARATOR_PROPELLER_R, SEPARATOR_PROPELLER_G, SEPARATOR_PROPELLER_B) < SEPARATOR_PROPELLER_COLOR_DISTANCE_THRESHOLD;
    if (puckSensorAtPropeller) {
      lastReadPuckColor = COLOR_NONE;
#ifdef DEBUG_PUCK_SENSOR
      Serial.println("Puck sensor at separator");
#endif  //DEBUG_PUCK_SENSOR
    }
#endif  //SEPARATOR_PROPELLER_DETECTION
    if (lastReadPuckColor == COLOR_NONE)
      puckSensorDetectionTimer.reset();
    currentPuckColor = (puckSensorDetectionTimer.milliseconds() > SEPARATOR_PUCK_DETECTION_TIME_THRESHOLD_MS) ? lastReadPuckColor : COLOR_NONE;
#endif  //PUCK_SENSOR_TYPE
    puckSensorUpdateTimer.reset();
  }
}


#endif  // ARKSTONE_UNIVERSAL_PUCKSENSOR_H