//
// Created by oaleksander on 23.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_ANGLEESTIMATOR_H
#define ARKSTONE_UNIVERSAL_ANGLEESTIMATOR_H

#include "Drivetrain.h"
#include "IMUSensor.h"
#include "LowPassFilter.h"
#include "UnitConversion.h"

double gyroAngleReference = 0;
double encoderAngleReference = 0;
double gyroEncoderAngleReference = 0;
double robotHeading = 0;
double robotHeadingOffset = 0;
Stopwatch angleSampleTimer = Stopwatch();

void updateAngleEstimator() {
    double encoderAngleMeasurement = drivetrainGetHeadingRadians();
#ifdef IMU_SENSOR
    if (imuHasNewDataFlag) {
        double timeStep = angleSampleTimer.seconds();
        angleSampleTimer.reset();
        // gyroAngleReference = imuHeading;
        // encoderAngleReference = encoderAngleMeasurement;
        gyroEncoderAngleReference = LowPassFilter::nextStep(gyroEncoderAngleReference, imuHeading - encoderAngleMeasurement + robotHeadingOffset, 0.15, timeStep);
        // robotHeading = angleWrapRadians(gyroAngleReference + robotHeadingOffset);  // gyroAngleReference;  //
        imuHasNewDataFlag = 0;
    }
#endif  // IMU_SENSOR
    robotHeading = angleWrapRadians(gyroEncoderAngleReference + encoderAngleMeasurement);

#ifdef DEBUG_ANGLE_ESTIMATOR
    Serial.print("Heading:");
    Serial.println(degrees(robotHeading));
#endif  // DEBUG_ANGLE_ESTIMATOR
}

void resetAngleEstimator() {
#ifdef IMU_SENSOR
    updateIMU();
#endif  // IMU_SENSOR
    updateAngleEstimator();
    robotHeadingOffset = -robotHeading;
    gyroEncoderAngleReference = robotHeadingOffset;
    robotHeading = 0;
    angleSampleTimer.reset();
}

#endif  // ARKSTONEPIONEER_PRIZM_ANGLEESTIMATOR_H
