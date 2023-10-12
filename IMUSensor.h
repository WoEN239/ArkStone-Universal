//
// Created by User on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_IMUSENSOR_H
#define ARKSTONE_UNIVERSAL_IMUSENSOR_H

#include "Stopwatch.h"
#include "UnitConversion.h"

double imuHeading = 0;
#if IMU_SENSOR == IMU_SENSOR_BNO055
#include "BNO055_support.h"
struct bno055_euler imuHeadingData = { 0, 0, 0 };
struct bno055_t imu;
signed short tiltReference = 0;
const double BNO055_DEGREES_RESOLUTION = 1 / 16.0;
const double BNO055_RADIANS_RESOLUTION = BNO055_RADIANS_RESOLUTION * PI / 180.0;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 imu;
#endif  //IMU_SENSOR

#include <math.h>

double imuHeadingReference = 0;
Stopwatch imuDataReadTimer = Stopwatch();
uint8_t imuHasNewDataFlag = 0;
uint8_t imuOK = false;

void updateIMU() {
#if IMU_SENSOR == IMU_SENSOR_BNO08X
  if (imu.dataAvailable())
#endif  //IMU_SENSOR
    if (imuDataReadTimer.milliseconds() > IMU_REFRESH_PERIOD_MS) {
#if IMU_SENSOR == IMU_SENSOR_BNO055
      bno055_read_euler_hrp(&imuHeadingData);
      imuHeading = imuHeadingData.h * BNO055_RADIANS_RESOLUTION;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
    imuDataReadTimer.reset();
    imuHasNewDataFlag = 1;
    imuHeading = angleWrapRadians(imu.getYaw() - imuHeadingReference);
#endif  //IMU_SENSOR
#ifdef DEBUG_IMU_SENSOR
      Serial.print("IMU:");
      Serial.println(degrees(imuHeading));
#endif  //DEBUG_IMU_SENSOR
    }
}

Stopwatch imuWatchdogTimer = Stopwatch();

void resetIMU();

void initIMU() {
#if IMU_SENSOR == IMU_SENSOR_BNO055
  BNO_Init(&imu);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
  imu.begin();
  imu.enableRotationVector(IMU_REFRESH_PERIOD_MS - 1);
  imuWatchdogTimer.reset();
  delay(100);
  while (!imu.dataAvailable() && imuWatchdogTimer.seconds() < 3.0) {
    delay(10);
  }
  resetIMU();
#endif  //IMUSENSOR
}



void resetIMU() {
#if IMU_SENSOR == IMU_SENSOR_BNO055
  bno055_read_euler_hrp(&imuHeadingData);
  imuHeading = imuHeadingData.h * BNO055_RADIANS_RESOLUTION;
  imuHeadingReference = imuHeading;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
  imuWatchdogTimer.reset();
  while (!imu.dataAvailable() && imuWatchdogTimer.seconds() < 3.0) {
    delay(10);
  }
  imuHeadingReference = imu.getYaw();
#endif
  imuHeading = 0;
}

#endif  //ARKSTONE_UNIVERSAL_IMUSENSOR_H
