#ifndef UNITCONVERSION_H
#define UNITCONVERSION_H
#include "RobotConfig.h"
#include <Arduino.h>

const double DRIVETRAIN_WHEEL_RPM = DRIVETRAIN_MOTOR_RPM / DRIVETRAIN_MOTOR_GEARBOX_RATIO;
const double DRIVETRAIN_MOTOR_ENCODER_GEARBOX_RESOLUTION = DRIVETRAIN_MOTOR_ENCODER_RESOLUTION * DRIVETRAIN_MOTOR_GEARBOX_RATIO;

const double DRIVETRAIN_WHEEL_LENGTH_MM = (DRIVETRAIN_WHEEL_RADIUS_MM * PI);
const double DRIVETRAIN_MM_TO_ENCODER_RATIO = DRIVETRAIN_WHEEL_LENGTH_MM / DRIVETRAIN_MOTOR_ENCODER_GEARBOX_RESOLUTION;
const double DRIVETRAIN_ENCODER_TO_MM_RATIO = DRIVETRAIN_MOTOR_ENCODER_GEARBOX_RESOLUTION / DRIVETRAIN_WHEEL_LENGTH_MM;

const double DRIVETRAIN_HEADING_RADIANS_TO_ENCODER_DIFFERENCE_RATIO = DRIVETRAIN_MM_TO_ENCODER_RATIO / DRIVETRAIN_TRACKWIDTH_MM;
const double DRIVETRAIN_ENCODER_DIFFERENCE_TO_HEADING_RADIANS_RATIO = DRIVETRAIN_TRACKWIDTH_MM / DRIVETRAIN_ENCODER_TO_MM_RATIO;

const double DRIVETRAIN_RPM_TO_VOLTS_RATIO = DRIVETRAIN_WHEEL_RPM / DRIVETRAIN_NOMINAL_VOLTAGE;
const double DRIVETRAIN_VOLTS_TO_RPM_RATIO = DRIVETRAIN_NOMINAL_VOLTAGE / DRIVETRAIN_WHEEL_RPM;

const double DRIVETRAIN_MM_S_TO_RPM_RATIO = DRIVETRAIN_WHEEL_LENGTH_MM / 60.0;
const double DRIVETRAIN_RPM_TO_MM_S_RATIO = 60.0 / DRIVETRAIN_WHEEL_LENGTH_MM;

const double DRIVETRAIN_MM_S_TO_VOLTS_RATIO = DRIVETRAIN_MM_S_TO_RPM_RATIO * DRIVETRAIN_RPM_TO_VOLTS_RATIO;
const double DRIVETRAIN_VOLTS_TO_MM_S_RATIO = DRIVETRAIN_VOLTS_TO_RPM_RATIO * DRIVETRAIN_RPM_TO_MM_S_RATIO;

const double DRIVETRAIN_MAX_SPEED_MM_S = DRIVETRAIN_MM_S_TO_VOLTS_RATIO * MAX_BATTERY_VOLTAGE;

double drivetrainMmToEncoder(double mm) {
  return DRIVETRAIN_ENCODER_TO_MM_RATIO * mm;
}

double drivetrainEncoderToMm(double encoder) {
  return DRIVETRAIN_MM_TO_ENCODER_RATIO * encoder;
}

double drivetrainHeadingRadiansToEncoderDiff(double heading) {
  return heading * DRIVETRAIN_ENCODER_DIFFERENCE_TO_HEADING_RADIANS_RATIO;
}

double drivetrainEncoderDifferenceToHeadingRadians(double encoderDiff) {
  return encoderDiff * DRIVETRAIN_HEADING_RADIANS_TO_ENCODER_DIFFERENCE_RATIO;
}

double drivetrainVoltsToRpm(double volts) {
  return volts * DRIVETRAIN_RPM_TO_VOLTS_RATIO;
}

double drivetrainRpmToVots(double rpm) {
  return rpm * DRIVETRAIN_VOLTS_TO_RPM_RATIO;
}

double drivetrainRpmToMmS(double rpm) {
  return rpm * DRIVETRAIN_MM_S_TO_RPM_RATIO;
}

double drivetrainMmSToRpm(double mms) {
  return mms * DRIVETRAIN_RPM_TO_MM_S_RATIO;
}

double drivetrainVoltsToMmS(double volts) {
  return volts * DRIVETRAIN_MM_S_TO_VOLTS_RATIO;
}

double drivetrainMmSToVolts(double mms) {
  return mms * DRIVETRAIN_VOLTS_TO_MM_S_RATIO;
}

double angleWrapDegrees(double degs) {
  while (degs > 180.0)
      degs -= 360.0;
  while (degs < -180.0)
      degs += 360.0;
  return degs ;
}

double angleWrapRadians(double rads) {
  if (rads > PI)
    do
      rads -= PI * 2;
    while (rads > PI);
  else if (rads < -PI)
    do
      rads += PI * 2;
    while (rads < -PI);
  return rads;
}

#endif  //UNITCONVERSION_H
