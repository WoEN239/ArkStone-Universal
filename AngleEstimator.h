//
// Created by User on 23.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_ANGLEESTIMATOR_H
#define ARKSTONE_UNIVERSAL_ANGLEESTIMATOR_H

#include "IMUSensor.h"
#include "Drivetrain.h"
#include "UnitConversion.h"

double gyroAngleReference = 0;
double encoderAngleReference = 0;
double robotHeading = 0;
double robotHeadingOffset = 0;

void updateAngleEstimator() {
#ifdef IMU_SENSOR
  if (imuHasNewDataFlag) {
    gyroAngleReference = imuHeading;
    encoderAngleReference = drivetrainGetHeadingRadians();
    robotHeading = angleWrapRadians(gyroAngleReference);  //gyroAngleReference;  //
    imuHasNewDataFlag = 0;
  } else
    robotHeading = gyroAngleReference + drivetrainGetHeadingRadians() - encoderAngleReference;  //angleWrapRadians(gyroAngleReference + drivetrainGetHeadingRadians() - encoderAngleReference);
#else
  robotHeading = angleWrapRadians(drivetrainGetHeadingRadians());
  //robotHeading = drivetrainGetHeadingRadians();
  robotHeading += robotHeadingOffset;
#endif  // IMUSENSOR

#ifdef DEBUG_ANGLE_ESTIMATOR
  Serial.print("Heading:");
  Serial.println(degrees(robotHeading));
#endif  //DEBUG_ANGLE_ESTIMATOR
}

void resetAngleEstimator() {
#ifdef IMU_SENSOR
  updateIMU();
#endif  //IMU_SENSOR
  updateAngleEstimator();
  robotHeadingOffset = -robotHeading;
}

#endif  //ARKSTONEPIONEER_PRIZM_ANGLEESTIMATOR_H
