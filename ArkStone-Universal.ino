#include <Prizm_Controller.h>

#include "RobotConfig.h"
#include "UnitConversion.h"
#include "Drivetrain.h"
#include "VoltageSensor.h"
#include "FieldColorSensor.h"
#include "PuckSensor.h"
#include "Speaker.h"
#include "Separator.h"
#include "Movement.h"
#include "StartButtton.h"
#include "Stopwatch.h"
#include "Barrier.h"
#include "DistanceSensor.h"
#include "IMUSensor.h"
#include "AngleEstimator.h"
#include "Intake.h"
#include "Light.h"
#include <FastLED.h>

void initRobot() {
  initLight();
  fillLight(CRGB::Cyan);
#ifdef UART_DEBUG
  Serial.begin(HARDWARE_SERIAL_BAUDRATE);
#endif
  Wire.begin();
  Wire.setClock(WIRE_CLOCK);
  driveExpansion.controllerReset();
  Prizm.begin(DO_NOT_WAIT_FOR_START_BUTTON);
  speakerTone(0, 400, 75);
  delay(55);

  initIMU();
  drivetrainInit();
  initPuckSensor();
  initFieldColorSensor();
  initBarrier();
  initSeparator();
  initDistanceSensor();
  fillLight(CRGB::Orange);
}
void setup() {
  initRobot();
  speakerTone(0, 400, 75);
  delay(75);
  speakerTone(0, 800, 75);
  delay(75);

  //while (!readStartButton()) loop();
  detectTeamFieldColor();
  switch (teamFieldColor) {
    case COLOR_RED:
      fillLight(0xFF0000);
      break;
    case COLOR_BLUE:
      fillLight(0x0000FF);
      break;
    case COLOR_NONE:
      fillLight(0xFFFFFF);
      break;
  }
  setSeparatorColor(teamFieldColor);

  speakerTone(0, 400, 75);
  delay(75);
  while (!readStartButton())
    loop();
  while (readStartButton())
    ;
  setIntakePower(INTAKE_POWER / 4);
  resetIMU();
  resetAngleEstimator();
  rotate(45 * 0);
  speakerTone(0, 800, 70);
  while (!readStartButton())
    rotateGlobal(45 * 0);
  /*for (;;) {
    move(45);
    wait(100);
    rotate(90);
    wait(100);
  }*/
  intakeEnable(true);
  moveKill(1000);
  for (;;) {
    /*
    rotateGlobal(angle);
    if (readStartButton())
      angle = (angle < 45) ? 90 : 0;
      */

    //waitStartButton();
    moveToWall();
    rotate(random(30, 150));
  }
  // while(!myIMU.dataAvailable());
  // angleReference = myIMU.getYaw();
}

#ifdef DEBUG_LOOP_TIMES

#endif
Stopwatch loopTimer = Stopwatch();
Stopwatch loopTimer2 = Stopwatch();

void loop() {
  /// Serial.print("GEN-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  updateFieldColor();
  // Serial.print("FD-");
  // Serial.println(loopTimer.milliseconds());
  //  loopTimer.reset();
  updatePuckSensor();
  // Serial.print("PS-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  voltageSensorUpdate();
  // Serial.print("VS-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  updateSeparator();
  // Serial.print("SP-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  updateDistanceSensor();
  // Serial.print("DS-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  drivetrainUpdate();
  //Serial.print("DT-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  updateBarrier();
  // Serial.print("BA-");
  // Serial.println(loopTimer.milliseconds());
  // loopTimer.reset();
  updateIMU();
  // Serial.print("IMU-");
  // Serial.println(loopTimer.milliseconds());
  //  loopTimer.reset();
  updateAngleEstimator();
  // Serial.print("AE-");
  // Serial.println(loopTimer.milliseconds());
//  loopTimer.reset();
//updateLight();
#ifdef DEBUG_LOOP_TIMES
  {
    double time = loopTimer2.milliseconds();
    if (time > 25.0) {
      Serial.print("Loop ms:");
      Serial.println(time);
    }
    loopTimer2.reset();
  }
#endif
}
