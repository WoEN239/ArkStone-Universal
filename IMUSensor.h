//
// Created by oaleksander on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_IMUSENSOR_H
#define ARKSTONE_UNIVERSAL_IMUSENSOR_H

#include "Stopwatch.h"
#include "UnitConversion.h"

double imuHeading = 0;
#if IMU_SENSOR == IMU_SENSOR_BNO055
#include "BNO055_support.h"
struct bno055_euler imuHeadingData = {0, 0, 0};
struct bno055_t imu;
signed short tiltReference = 0;
const double BNO055_DEGREES_RESOLUTION = -1.0 / 16.0;
const double BNO055_RADIANS_RESOLUTION = BNO055_DEGREES_RESOLUTION * PI / 180.0;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
#include "BNO08X.h"
BNO08X imu;
#endif  // IMU_SENSOR

#include <math.h>

double imuHeadingReference = 0;
Stopwatch imuDataReadTimer = Stopwatch();
uint8_t imuHasNewDataFlag = 0;
uint8_t imuOK = false;

Stopwatch imuFixTimer = Stopwatch();
boolean imuFixActive = false;

const double IMU_FIX_TELEPORT_THRESHOLD = 20.0;
const double IMU_FIX_TIME_THRESHOLD_S = 2.0;

void enableIMUfix() {
    imuFixTimer.reset();
    imuFixActive = true;
}

void updateIMU() {
#if IMU_SENSOR == IMU_SENSOR_BNO08X
    if (imu.dataAvailable()) {
#else
    if (imuDataReadTimer.milliseconds() > IMU_REFRESH_PERIOD_MS) {
#endif  // IMU_SENSOR
        imuDataReadTimer.reset();
#if IMU_SENSOR == IMU_SENSOR_BNO055
        bno055_read_euler_hrp(&imuHeadingData);
        imuHeading = angleWrapRadians(imuHeadingData.h * BNO055_RADIANS_RESOLUTION - imuHeadingReference);
        imuHasNewDataFlag = 1;
        if(imuHeading != 0)
            imuOK = true;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
        double imuRead = imu.getYaw();
        if (imuFixActive) {
            if (imuFixTimer.seconds() > IMU_FIX_TIME_THRESHOLD_S)
                imuFixActive = false;
            else if (abs(angleWrapRadians(imuRead - imuHeading)) > IMU_FIX_TELEPORT_THRESHOLD) {
                imuHeadingReference += imuRead - imuHeading;
                imuFixActive = false;
            }
        }
        if (!isnan(imuRead)) {
            imuHeading = angleWrapRadians(imuRead - imuHeadingReference);
            imuHasNewDataFlag = 1;
            imuOK = true;
        }
#endif  // IMU_SENSOR
#ifdef DEBUG_IMU_SENSOR
        Serial.print("IMU:");
        Serial.println(degrees(imuHeading));
#endif  // DEBUG_IMU_SENSOR
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
    imu.enableGameRotationVector(IMU_REFRESH_PERIOD_MS);
    imuWatchdogTimer.reset();
    delay(100);
    resetIMU();
#endif  // IMUSENSOR
}

void resetIMU() {
    return;
#if IMU_SENSOR == IMU_SENSOR_BNO055
    bno055_read_euler_hrp(&imuHeadingData);
    imuHeading = (double) imuHeadingData.h * BNO055_RADIANS_RESOLUTION;
    imuHeadingReference = imuHeading;
#elif IMU_SENSOR == IMU_SENSOR_BNO08X
    imuWatchdogTimer.reset();
    while (!imu.dataAvailable() && imuWatchdogTimer.seconds() < 3.0) {
        delay(IMU_REFRESH_PERIOD_MS);
    }
    if (!isnan(imu.getYaw()))
        imuHeadingReference = imu.getYaw();
    else {
        imu.softReset();
        imu.enableGameRotationVector(IMU_REFRESH_PERIOD_MS);
    }
#endif
    imuHeading = 0;
    imuDataReadTimer.reset();
}

#endif  // ARKSTONE_UNIVERSAL_IMUSENSOR_H
