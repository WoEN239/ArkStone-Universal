#ifndef ARKSTONE_UNIVERSAL_DRIVETRAIN_H
#define ARKSTONE_UNIVERSAL_DRIVETRAIN_H

#include <Prizm_Controller.h>
#include "RobotConfig.h"
#include "VoltageSensor.h"

int32_t drivetrainLeftEncoderValue = 0;
int32_t drivetrainRightEncoderValue = 0;

#ifdef DRIVETRAIN_POWER_FILTERING
#include "LowPassFilter.h"
LowPassFilter leftPowerFilter = LowPassFilter(DRIVETRAIN_POWER_FILTER_T);
LowPassFilter rightPowerFilter = LowPassFilter(DRIVETRAIN_POWER_FILTER_T);
#endif //DRIVETRAIN_POWER_FILTERING

int16_t leftDrivePower = 0;
int16_t rightDrivePower = 0;

double drivetrainForwardPower = 0.0;

void drivetrainInit() {
  driveExpansion.setZeroPowerBehavior(DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER, BRAKE);
  driveExpansion.setZeroPowerBehavior(DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER, BRAKE);
  driveExpansion.setDirection(DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER, DRIVETRAIN_MOTOR_INVERSE ? FORWARD : REVERSE);
  driveExpansion.setDirection(DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER, DRIVETRAIN_MOTOR_INVERSE ? REVERSE : FORWARD);
  driveExpansion.resetEncoders();
  leftPowerFilter.reset();
  rightPowerFilter.reset();
  driveExpansion.setPowers(0, 0);
}

double drivetrainGetDistanceMm() {
  return drivetrainEncoderToMm(double(drivetrainLeftEncoderValue + drivetrainRightEncoderValue) / 2.0);
}

void drivetrainSetPowers(int16_t forwardPower, int16_t rotationPower) {
  forwardPower = constrain(forwardPower, -100, 100);
  rotationPower = constrain(rotationPower, -100, 100);
  leftDrivePower = forwardPower - rotationPower;
  leftDrivePower = forwardPower + rotationPower;
  int16_t max = max(abs(leftDrivePower), abs(rightDrivePower));
  if (max > DRIVETRAIN_MAX_MOTOR_POWER) {
    float multiplier = (float)DRIVETRAIN_MAX_MOTOR_POWER / (float)max;
    leftDrivePower = float(leftDrivePower) * multiplier;
    rightDrivePower = float(rightDrivePower) * multiplier;
  }
  
}

const int32_t PROBLEMATIC_TETRIX_ENCODER_VALUE = -257;
const int32_t PROBLEMATIC_TETRIX_ENCODER_THRESHOLD = 50;

void drivetrainUpdate() {
  int32_t newEncoderValue = driveExpansion.getCurrentPosition(DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER);
  if (!(newEncoderValue == PROBLEMATIC_TETRIX_ENCODER_VALUE && abs(drivetrainLeftEncoderValue - PROBLEMATIC_TETRIX_ENCODER_VALUE) > PROBLEMATIC_TETRIX_ENCODER_THRESHOLD))
    drivetrainLeftEncoderValue = newEncoderValue;
  newEncoderValue = driveExpansion.getCurrentPosition(DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER);
  if (!(newEncoderValue == PROBLEMATIC_TETRIX_ENCODER_VALUE && abs(drivetrainRightEncoderValue - PROBLEMATIC_TETRIX_ENCODER_VALUE) > PROBLEMATIC_TETRIX_ENCODER_THRESHOLD))
    drivetrainRightEncoderValue = newEncoderValue;

  #ifdef DRIVETRAIN_POWER_FILTERING
  leftDrivePower = leftPowerFilter.update(leftDrivePower);
  rightDrivePower = rightPowerFilter.update(rightDrivePower);
  #endif //#ifdef DRIVETRAIN_POWER_FILTERING

  drivetrainForwardPower = (leftDrivePower + rightDrivePower) * 0.5;
  driveExpansion.setPowers(leftDrivePower, rightDrivePower);
#ifdef DEBUG_DRIVETRAIN
  Serial.print("Dt dist:");
  Serial.println(drivetrainGetDistanceMm());
#endif
}

double drivetrainGetHeadingRadians() {
  return drivetrainEncoderDifferenceToHeadingRadians(double(drivetrainRightEncoderValue - drivetrainLeftEncoderValue));
}

double drivetrainVoltageToDutyCycle(double voltage) {
  return voltage / batteryVoltage * MAX_DUTY_CYCLE;
}

void drivetrainStop() {
  driveExpansion.setPowers(0, 0);
}



#endif  //ARKSTONE_UNIVERSAL_DRIVETRAIN_H
