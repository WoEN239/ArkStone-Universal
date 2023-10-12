//
// Created by oaleksander on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_INTAKE_H
#define ARKSTONE_UNIVERSAL_INTAKE_H

#include <Arduino.h>
#include "Prizm_DC_Expansion.h"
#include "RobotConfig.h"

PrizmDCExpansion intakeDcExpansion = Prizm.MotorController;

void initIntake(){
    intakeDcExpansion.setZeroPowerBehavior(INTAKE_PORT_NUMBER, BRAKE);
    intakeDcExpansion.setDirection(INTAKE_PORT_NUMBER, FORWARD);
}

void setIntakePower(int8_t intakePower){
   intakeDcExpansion.setPower(INTAKE_PORT_NUMBER, intakePower);
}

void intakeEnable(uint8_t enable){
    setIntakePower(enable?INTAKE_POWER:0);
}

#endif //ARKSTONE_UNIVERSAL_INTAKE_H
