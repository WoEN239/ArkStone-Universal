//
// Created by User on 07.04.2023.
//

#ifndef ARKSTONE_UNIVERSAL_BARRIER_H
#define ARKSTONE_UNIVERSAL_BARRIER_H
#include <Arduino.h>
#include "FieldColorSensor.h"
#include "RobotConfig.h"

#define BARRIER_SERVO_NUMBER 1

void setBarrierServoPosition(bool open){
    barrierServoController.setPosition(BARRIER_SERVO_NUMBER, open?BARRIER_SERVO_OPEN:BARRIER_SERVO_CLOSE);
}

void initBarrier(){
    setBarrierServoPosition(false);
}

void updateBarrier(){
    setBarrierServoPosition(robotIsOnTeamField);
}

#endif //ARKSTONE_UNIVERSAL_BARRIER_H
