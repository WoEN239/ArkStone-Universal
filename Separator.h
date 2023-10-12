//
// Created by Oaleksander on 07.04.2023.
//

#ifndef ARKSTONE_UNIVERSAL_SEPARATOR_H
#define ARKSTONE_UNIVERSAL_SEPARATOR_H

#include <Arduino.h>
#include "Prizm_Controller.h"

#include "RobotConfig.h"
#include "PuckCollectCommons.h"
#include "PIDVASRegulator.h"
#include "FieldColorSensor.h"
#include "PuckSensor.h"


PrizmDCExpansion separatorMotorController = Prizm.MotorController;
#define SEPARATOR_LIGHT_PORT_NUMBER 2

double separatorEncoderDegrees = 0;
double separatorPositionError = 0;
double separatorPositionTarget = .0;

double getSeparatorEncoderRaw() {
  return separatorMotorController.getCurrentPosition(SEPARATOR_MOTOR_PORT_NUMBER) * 0.5;
}

double separatorEncoderOffset = 0;

void updateSeparatorEncoder() {
  separatorEncoderDegrees = getSeparatorEncoderRaw() + separatorEncoderOffset;
  separatorPositionError = separatorPositionTarget - separatorEncoderDegrees;
}

void resetSeparatorEncoder() {
  separatorMotorController.resetEncoder(SEPARATOR_MOTOR_PORT_NUMBER);
}

void setSeparatorColor(Color color) {
  if (color == COLOR_RED)
#ifndef SEPARATOR_INVERSE
    separatorMotorController.setDirection(SEPARATOR_MOTOR_PORT_NUMBER, FORWARD);
  else
    separatorMotorController.setDirection(SEPARATOR_MOTOR_PORT_NUMBER, REVERSE);
#else
    separatorMotorController.setDirection(SEPARATOR_MOTOR_PORT_NUMBER, REVERSE);
  else
    separatorMotorController.setDirection(SEPARATOR_MOTOR_PORT_NUMBER, FORWARD);
#endif
}

void setSeparatorPower(int8_t power) {
  if (abs(power) < 8) {
    if (abs(power) < 4)
      power = 0;
    else power = 8 * sign(power);
  }
  power = constrain(power, -SEPARATOR_POWER_LIMIT, SEPARATOR_POWER_LIMIT);
  separatorMotorController.setPower(SEPARATOR_MOTOR_PORT_NUMBER, power);
}

struct pidfParameters separatorPositonPidfParameters = { SEPARATOR_KP, SEPARATOR_KI, SEPARATOR_KD, 0.0, SEPARATOR_MAXI, SEPARATOR_MIN_SAMPLE_TIME };

struct pidfState separatorPositionPidfState = { .0, .0, .0, .0, .0 };

int8_t calculateSeparatorPower() {
  return constrain(
    pidfUpdate(separatorPositonPidfParameters, &separatorPositionPidfState, separatorPositionError), -100.0d, 100.0d);
}

void updateSeparatorMotor() {
  setSeparatorPower(calculateSeparatorPower());
}

boolean separatorAtPosition() {
  return (separatorPositionError < SEPARATOR_POSITION_THRESHOLD && separatorPositionError > -SEPARATOR_POSITION_THRESHOLD);
}

#ifdef SEPARATOR_PROPELLER_PARKING
void parkSeparator();
#endif //SEPARATOR_PROPELLER_PARKING

void initSeparator() {
  separatorMotorController.setZeroPowerBehavior(SEPARATOR_MOTOR_PORT_NUMBER, BRAKE);
  resetSeparatorEncoder();
#ifdef SEPARATOR_PROPELLER_PARKING
  parkSeparator();
#endif  //SEPARATOR_PROPELLER_PARKING
}

#define SEPARATOR_STEP_DEGREES 120

Color lastSortedColor = COLOR_NONE;

boolean separatorWasAtPosition = true;

byte collectedPucksCount[3] = { 0, 0, 0 };
Stopwatch separatorAtPositionTimer = StopWatch();

Stopwatch separatorUpdateTimer = Stopwatch();

void updateSeparator() {
  if (separatorUpdateTimer.milliseconds() > SEPARATOR_UPDATE_PERIOD_MS) {
    updateSeparatorEncoder();
    boolean separatorIsAtPosition = separatorAtPosition();
    if (separatorIsAtPosition)
      separatorAtPositionTimer.reset();
#ifdef DEBUG_SEPARATOR
    Serial.print("Separator at position: ");
    Serial.println(separatorIsAtPosition);
#endif

    boolean separatorDetectionActive = separatorIsAtPosition && currentFieldColor == COLOR_NONE;
    if (!separatorWasAtPosition && separatorIsAtPosition)
      collectedPucksCount[colorToIndex(lastSortedColor)]++;

    if (separatorDetectionActive) {
#ifdef PIONEER_SEPARATOR
      for (size_t i = 0; i < 2; i++)
        if (collectedPucksCount[i] > 0b00001111) collectedPucksCount[i] = 0b00001111;
      pioneerUartSendPucksCount(collectedPucksCount[COLOR_RED], collectedPucksCount[COLOR_BLUE]);
#endif
      if (currentPuckColor != COLOR_NONE) {
        separatorPositionTarget += SEPARATOR_STEP_DEGREES * currentPuckColor;
        lastSortedColor = currentPuckColor;
        puckSensorDetectionTimer.reset();
      }
    } else
      puckSensorDetectionTimer.reset();
    if (separatorAtPositionTimer.milliseconds() > SEPARATOR_MOVEMENT_TIMEOUT_MS) {
#ifdef DEBUG_SEPARATOR
      Serial.println("Separator stalled!!!");
#endif
      separatorAtPositionTimer.reset();
      pidfReset(&separatorPositionPidfState);
      separatorPositionTarget -= SEPARATOR_STEP_DEGREES * sign(separatorPositionError);
    }
    updateSeparatorMotor();
    separatorWasAtPosition = separatorIsAtPosition;
    separatorUpdateTimer.reset();
  }
}

#ifdef SEPARATOR_PROPELLER_PARKING

void parkSeparator() {
  delay(SEPARATOR_PROPELLER_PARKING_DELAY);
  updatePuckSensor();
  if (puckSensorAtPropeller) {
    setSeparatorPower(-SEPARATOR_PROPELLER_PARKING_SPEED);
    double propellerPositionStart = getSeparatorEncoderRaw();
    while(getSeparatorEncoderRaw() - propellerPositionStart > -20.0)
      updatePuckSensor();
  }
  setSeparatorPower(0);
  setSeparatorPower(SEPARATOR_PROPELLER_PARKING_SPEED);
  while (!puckSensorAtPropeller)
    updatePuckSensor();
  double propellerPositionStart = getSeparatorEncoderRaw();
  delay(SEPARATOR_PROPELLER_PARKING_DELAY);
  while (puckSensorAtPropeller)
    updatePuckSensor();
  delay(SEPARATOR_PROPELLER_PARKING_DELAY);
  setSeparatorPower(-SEPARATOR_PROPELLER_PARKING_SPEED);
  while (!puckSensorAtPropeller)
    updatePuckSensor();
  double propellerPositionEnd = getSeparatorEncoderRaw();
  separatorEncoderOffset = 60.0 - (propellerPositionStart + propellerPositionEnd) * 0.5 + 5;
  do
    updateSeparator();
  while (!separatorAtPosition());
  setSeparatorPower(0);
}
#endif  //SEPARATOR_PROPELLER_PARKING

#endif  //ARKSTONE_UNIVERSAL_SEPARATOR_H
