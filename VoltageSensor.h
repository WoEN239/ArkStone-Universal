#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

#include <Prizm_Controller.h>
#include "RobotConfig.h"

double batteryVoltage = 12.0;

void voltageSensorUpdate() {
  batteryVoltage = (double)Prizm.readBatteryVoltage() / 100.0;
}

double voltageToDutyCycle(double voltage) {
  return voltage / batteryVoltage * MAX_DUTY_CYCLE;
}

#endif  //VOLTAGE_SENSOR_H