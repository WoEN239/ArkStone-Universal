#ifndef ARKSTONE_UNIVERSAL_DISTANCESENSOR_H
#define ARKSTONE_UNIVERSAL_DISTANCESENSOR_H

#include <Arduino.h>

#include "Drivetrain.h"
#include "PuckCollectCommons.h"
#include "RobotConfig.h"
#include "UnitConversion.h"

#if SONAR_SENSOR == SONAR_SENSOR_PING
#include "NewPing.h"
#define SONAR1_TRIG_PIN 9
#define SONAR1_ECHO_PIN 2
NewPing sonar1(SONAR1_TRIG_PIN, SONAR1_ECHO_PIN, SONAR_MAX_DISTANCE);
uint8_t sonar1_last_read = SONAR_MAX_DISTANCE;
#define readSonar1Distance() sonar1.ping_cm()

#ifdef DUAL_SONAR
#define SONAR2_TRIG_PIN 5
#define SONAR2_ECHO_PIN 4
NewPing sonar2(SONAR2_TRIG_PIN, SONAR2_ECHO_PIN, SONAR_MAX_DISTANCE);
uint8_t sonar2_last_read = SONAR_MAX_DISTANCE;
uint8_t current_sonar_to_read = 0;
#define readSonar2Distance() sonar2.ping_cm()
#endif  // DUAL_SONAR

#elif ROBOT == MODERN
#include "MB1242.h"
#define readSonar1Distance() MB1242getRange()
#elif SONAR_SENSOR == SONAR_SENSOR_MB1242_SOFTI2C
#include <MB1242SoftI2C.h>
#include <SoftwareWire.h>
SoftwareWire distanceSensorWire(SONAR_SDA, SONAR_SCL);
MB1242SoftI2C distanceSensor(&distanceSensorWire);
#define readSonar1Distance() distanceSensor.getRangeCm()
#else
#error Unknown sonar type
#endif  // SONAR_SENSOR

uint16_t sonarDistance = SONAR_MAX_DISTANCE;
double encoderDistanceReference = 0;
double distanceSensorLastDistance = SONAR_MAX_DISTANCE;
Stopwatch distanceUpdateTimer = Stopwatch();
double distanceToWall = SONAR_MAX_DISTANCE;

void initDistanceSensor() {
#if ROBOT == SUPERLEICHT
    distanceSensorWire.begin();
#endif  // ROBOT
}

void updateDistanceSensor() {
    double result;
#ifdef USE_ANALOG_WALL_SENSOR
    int leftAnalogSensorValue = analogRead(ANALOG_WALL_SENSOR_LEFT);
    int rightAnalogSensorValue = analogRead(ANALOG_WALL_SENSOR_RIGHT);
#ifdef DEBUG_ANALOG_DISTANCE_SENSOR
    Serial.print("anal:");
    Serial.print(leftAnalogSensorValue);
    Serial.print(",anar:");
    Serial.println(rightAnalogSensorValue);
#endif  // DEBUG_DISTANCE_SENSOR
    if (leftAnalogSensorValue < ANALOG_LEFT_WALL_SENSOR_THRESHOLD && rightAnalogSensorValue < ANALOG_RIGHT_WALL_SENSOR_THRESHOLD) {
        result = ANALOG_WALL_SENSOR_TRIGGER_DISTANCE;
    } else
        result = SONAR_MAX_DISTANCE;
    distanceToWall = result;
#else
    if (distanceUpdateTimer.milliseconds() >= SONAR_UPDATE_PERIOD_MS) {
#ifdef DUAL_SONAR
        if (current_sonar_to_read) {
            sonar1_last_read = readSonar1Distance();
        } else {
            sonar2_last_read = readSonar2Distance();
        }
        current_sonar_to_read = !current_sonar_to_read;
        if (sonar2_last_read < 5 && sonar1_last_read < 5) {
            sonarDistance = SONAR_MAX_DISTANCE;
        } else if (sonar1_last_read < 5) {
            sonarDistance = sonar2_last_read;
        } else if (sonar2_last_read < 5) {
            sonarDistance = sonar1_last_read;
        } else {
            sonarDistance = (sonar1_last_read + sonar2_last_read) / 2;
        }
#else
        sonarDistance = readSonar1Distance();
        if (sonarDistance > SONAR_MAX_DISTANCE) {
            sonarDistance = SONAR_MAX_DISTANCE;
        }
#endif  // ROBOT
        double maxEncoderDelta = (-drivetrainGetDistanceMm() * 0.1 - encoderDistanceReference);
        encoderDistanceReference = -drivetrainGetDistanceMm() * 0.1;
        double maxTheoreticalDelta = -drivetrainVoltsToMmS(batteryVoltage) * 0.1 * distanceUpdateTimer.seconds();
        double maxMotorPowerDelta = -drivetrainVoltsToMmS(batteryVoltage * drivetrainForwardPower * MAX_DUTY_CYCLE_INV) * 0.1 * distanceUpdateTimer.seconds();
        double maxDistanceDelta = maxEncoderDelta * 0.4 + maxTheoreticalDelta * 0.2 + maxMotorPowerDelta * 0.4;
        distanceUpdateTimer.reset();
#ifdef SONAR_DEADZONE
        if (sonarDistance <= SONAR_MIN_DISTANCE || (sonarDistance > SONAR_DEADZONE_START && sonarDistance < SONAR_DEADZONE_END) || sonarDistance == SONAR_MAX_DISTANCE)
#else
        if (sonarDistance <= SONAR_MIN_DISTANCE)
#endif  // SONAR_DEADZONE
            result = distanceToWall - maxEncoderDelta * 0.5 - maxMotorPowerDelta * 0.5;
        else {
            if (sonarDistance < distanceToWall)
                result = max(distanceToWall + maxDistanceDelta, sonarDistance);
            else if (sonarDistance > distanceToWall)
                result = min(sonarDistance, distanceToWall - maxDistanceDelta);
            else
                result = sonarDistance;
        }
        distanceSensorLastDistance = result;
    } else {
        result = -drivetrainGetDistanceMm() * 0.1 - encoderDistanceReference + distanceSensorLastDistance;
    }
    distanceToWall = result;
    if (distanceToWall > SONAR_MAX_DISTANCE) {
        distanceToWall = SONAR_MAX_DISTANCE;
    } else if (distanceToWall < 0) {
        distanceToWall = 0;
    }
#endif  // USE_ANALOG_WALL_SENSOR
#ifdef DEBUG_DISTANCE_SENSOR
    Serial.print("Dist2wall:");
    Serial.println(distanceToWall);
    Serial.print("Sonar:");
    Serial.println(sonarDistance);
#endif  // DEBUG_DISTANCE_SENSOR
}

#endif  // ARKSTONEPIONEER_PRIZM_DISTANCESENSOR_H
