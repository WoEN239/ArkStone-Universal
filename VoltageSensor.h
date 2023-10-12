#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

#include <Prizm_Controller.h>

double batteryVoltage = 12.0;

void voltageSensorUpdate() {
  batteryVoltage = (double)Prizm.readBatteryVoltage() / 100.0;
}

#endif  //VOLTAGE_SENSOR_H