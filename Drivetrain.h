#ifndef ARKSTONE_UNIVERSAL_DRIVETRAIN_H
#define ARKSTONE_UNIVERSAL_DRIVETRAIN_H

#include <Prizm_Controller.h>
#include "RobotConfig.h"
#include "VoltageSensor.h"
#include "LowPassFilter.h"

int32_t drivetrainLeftEncoderValue = 0;
int32_t drivetrainRightEncoderValue = 0;

LowPassFilter leftPowerFilter = LowPassFilter(0.0075);
LowPassFilter rightPowerFilter = LowPassFilter(0.0075);


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
  int16_t leftPower = forwardPower - rotationPower;
  int16_t rightPower = forwardPower + rotationPower;
  int16_t max = max(abs(leftPower), abs(rightPower));
  if (max > DRIVETRAIN_MAX_MOTOR_POWER) {
    float multiplier = (float)DRIVETRAIN_MAX_MOTOR_POWER / (float)max;
    leftPower = float(leftPower) * multiplier;
    rightPower = float(rightPower) * multiplier;
  }
  //leftPower = leftPowerFilter.update(leftPower);
  //rightPower = rightPowerFilter.update(rightPower);
  drivetrainForwardPower = (leftPower + rightPower) * 0.5;
  driveExpansion.setPowers(leftPower, rightPower);
}

void drivetrainUpdate() {
  int32_t drivetrainLeftNewEncoderValue = driveExpansion.getCurrentPosition(DRIVETRAIN_LEFT_MOTOR_PORT_NUMBER);
  if (!(drivetrainLeftNewEncoderValue == -257 && abs(drivetrainLeftEncoderValue + 257) > 50))
    drivetrainLeftEncoderValue = drivetrainLeftNewEncoderValue;
  int32_t drivetrainRightNewEncoderValue = driveExpansion.getCurrentPosition(DRIVETRAIN_RIGHT_MOTOR_PORT_NUMBER);
  if (!(drivetrainRightNewEncoderValue == -257 && abs(drivetrainRightEncoderValue + 257) > 50))
    drivetrainRightEncoderValue = drivetrainRightNewEncoderValue;
  leftPowerFilter.reset();
  rightPowerFilter.reset();
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
