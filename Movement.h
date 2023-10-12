//
// Created by oaleksander on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_MOVEMENT_H
#define ARKSTONE_UNIVERSAL_MOVEMENT_H

#include "Prizm_Controller.h"
#include "RobotConfig.h"

#include "Drivetrain.h"
#include "Arduino.h"
#include "PIDVASRegulator.h"
#include "DistanceSensor.h"
#include "Stopwatch.h"
#include "PuckCollectCommons.h"
#include "AngleEstimator.h"
#include "UnitConversion.h"

double drivetrainGetDistance() {
  return drivetrainGetDistanceMm() / 10.0;
}

#define MOVEMENT_ANGLE_THRESHOLD 3
#define MOVEMENT_DISTANCE_THRESHOLD 5.0
const double MOVEMENT_TIMEOUT_MS_DEFAULT = 2000.0;
const double MOVEMENT_DISTANCE_TO_WALL = 35.0;


struct pidfParameters anglePidfParameters = { ANGLE_KP, ANGLE_KI, ANGLE_KD, 5.0, ANGLE_MAX_I, ANGLE_MIN_SAMPLE_TIME };
struct pidfParameters distancePidfParameters = { DISTANCE_KP, DISTANCE_KI, DISTANCE_KD, 0.0, DISTANCE_MAX_I, DISTANCE_MIN_SAMPLE_TIME };

enum movementState {
  ROTATE,
  MOVE,
  MOVE2WALL,
  CONSTANTVELOCITY,
  WAIT
};
enum movementState movementState;

int32_t timeoutMs = MOVEMENT_TIMEOUT_MS_DEFAULT;
int32_t startTime = 0;

void resetMovementTime() {
  startTime = (int32_t)millis();
}

int32_t getMovementTimeMs() {
  return (int32_t)millis() - startTime;
}

double getMovementTimeSeconds() {
  return double(getMovementTimeMs()) / 1000.0;
}

double angleTarget = 0.0;

struct pidfState anglePidfState = { .0, .0, .0, .0, .0 };

double getAngleError() {
  return angleWrapDegrees(angleTarget - degrees(robotHeading));
  //return angleTarget - degrees(robotHeading);
}

double getAnglePower() {
  return constrain(pidfUpdate(anglePidfParameters, &anglePidfState, getAngleError()), -ROTATION_SPEED, ROTATION_SPEED);
}

double distanceTarget = 0.0;

struct pidfState distancePidfState = { .0, .0, .0, .0, .0 };

double getDistanceError() {
  return distanceTarget - drivetrainGetDistance();
}

double getDistancePower() {
  return constrain(pidfUpdate(distancePidfParameters, &distancePidfState, getDistanceError()), -MOVEMENT_SPEED, MOVEMENT_SPEED);
}

double getWallError() {
  return distanceToWall - MOVEMENT_DISTANCE_TO_WALL;
}

double getWallPower() {
  return constrain(pidfUpdate(distancePidfParameters, &distancePidfState, getWallError()), -MOVEMENT_SPEED, MOVEMENT_SPEED);
}

double velocityTarget = .0;
double velocityTargetDistance = .0;
double constantVelocityStart = 0.0;

double getConstantVelocityPower() {
  distanceTarget = velocityTarget * getMovementTimeSeconds() + constantVelocityStart;
  return getDistancePower();
}

double getVelocityDistanceError() {
  return (velocityTargetDistance + constantVelocityStart) - drivetrainGetDistance();
}

void movementReset() {
  pidfReset(&anglePidfState);
  pidfReset(&distancePidfState);
}

//void (*movementUpdateRegulators)() = &__empty;


uint8_t movementIsAtTarget() {
  switch (movementState) {
    case ROTATE:
      return abs(getAngleError()) < MOVEMENT_ANGLE_THRESHOLD;
    case MOVE:
      return abs(getDistanceError()) < MOVEMENT_DISTANCE_THRESHOLD;
    case MOVE2WALL:
      return distanceToWall - MOVEMENT_DISTANCE_TO_WALL <= 0;
    case CONSTANTVELOCITY:
      return getVelocityDistanceError() * sign(velocityTarget) < 0;
    case WAIT:
      return getMovementTimeMs() >= timeoutMs;
  }
  return false;
}

void movementUpdate() {
  //(*movementUpdateRegulators)();
  loop();
  switch (movementState) {
    case ROTATE:
      drivetrainSetPowers(0, getAnglePower());
      break;
    case MOVE2WALL:
      drivetrainSetPowers(getWallPower(), 0);  //, getAnglePower());
      break;
    case WAIT:
    case MOVE:
      drivetrainSetPowers(getDistancePower(), getAnglePower());
      break;
    case CONSTANTVELOCITY:
      angleTarget = 2.5 * sin(PI * double(millis()) / 100.0);
      drivetrainSetPowers(getConstantVelocityPower(), getAnglePower());
      break;
  }
}



void movementReachTarget() {
  if (movementState != CONSTANTVELOCITY && movementState != WAIT)
    movementReset();
  resetMovementTime();
  do {
    movementUpdate();
  } while (!movementIsAtTarget() && getMovementTimeMs() < timeoutMs);
  if (movementState != CONSTANTVELOCITY)
    drivetrainStop();
}

void rotateGlobal(double _angleTarget) {
  angleTarget = _angleTarget;
  movementState = ROTATE;
  timeoutMs = MOVEMENT_TIMEOUT_MS_DEFAULT;  //OLEG
  movementReachTarget();
}

void rotate(double _angleTarget) {
  rotateGlobal(_angleTarget + angleTarget);
}

void move(double _distanceTarget) {
  distanceTarget = _distanceTarget + drivetrainGetDistance();
  movementState = MOVE;
  timeoutMs = MOVEMENT_TIMEOUT_MS_DEFAULT;
  movementReachTarget();
}

Stopwatch killTimer = Stopwatch();

void moveKill(double ms) {
  killTimer.reset();
  while (killTimer.milliseconds() < ms) {
    drivetrainSetPowers(100 , 0);
    loop();
  }
  drivetrainStop();
}

void moveToWall() {
  //move(10);
  movementState = MOVE2WALL;
  timeoutMs = MOVEMENT_TIMEOUT_MS_DEFAULT;
  movementReachTarget();
  move(-20);
}

void constantVelocity(double velocity, double _timeoutMs, double distance) {
  velocityTargetDistance = distance;
  timeoutMs = int32_t(_timeoutMs);
  velocityTarget = velocity;
  movementState = CONSTANTVELOCITY;
  constantVelocityStart = drivetrainGetDistance();
  movementReachTarget();
}

void constantVelocity(double velocity, double _timeoutMs) {
  constantVelocity(velocity, _timeoutMs, INT32_MAX);
}

void wait(int32_t waitMs){
  movementState = WAIT;
  timeoutMs = waitMs;
  movementReachTarget();
}

#endif  //ARKSTONE_UNIVERSAL_MOVEMENT_H
