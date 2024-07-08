#include <FastLED.h>
#include <Prizm_Controller.h>

#include "AngleEstimator.h"
#include "Barrier.h"
#include "DistanceSensor.h"
#include "Drivetrain.h"
#include "FieldColorSensor.h"
#include "IMUSensor.h"
#include "Intake.h"
#include "Light.h"
#include "Movement.h"
#include "PuckSensor.h"
#include "RobotConfig.h"
#include "Separator.h"
#include "Speaker.h"
#include "StartButtton.h"
#include "Stopwatch.h"
#include "UartClient.h"
#include "UnitConversion.h"
#include "VoltageSensor.h"

void initRobot() {
    initLight();
    fillRainbow();
#if defined(UART_DEBUG) || defined(ARKSONE_UNIVERSAL_UARTCLIENT_H)
    Serial.begin(HARDWARE_SERIAL_BAUDRATE);
#endif
    Wire.begin();
    Wire.setClock(WIRE_CLOCK);
    driveExpansion.controllerReset();
    Prizm.begin(DO_NOT_WAIT_FOR_START_BUTTON);
    speakerTone(0, 432, 75);
    initIMU();
    drivetrainInit();
    initPuckSensor();
    initFieldColorSensor();
    initBarrier();
    initSeparator();
    initDistanceSensor();
    fillLight(CRGB::Orange);
}

const uint32_t readyTime = 10 * 1000;
const uint32_t idleTime = 30 * 1000;

void readyIndication() {
    if (millis() > readyTime || !imuOK) {
        blinkLightsOddEven(colorToCode(teamFieldColor), CRGB::Green, imuOK ? 1000 : 100);
    } else
        displayTeamColor();
}

void displayTeamColor() {
    fillLight(colorToCode(teamFieldColor));
}

void setup() {
    initRobot();
    speakerTone(0, 432, 75);

    // while (!readStartButton()) loop();
    detectTeamFieldColor();
    displayTeamColor();
    setSeparatorColor(teamFieldColor);

    speakerTone(0, 864, 75);
    while (!readStartButton()) {
        loop();
        updateUartClient();
        if (gamepadActive) {
            drivetrainSetPowers(gamepadForward, gamepadTurn);
            if (leds[0] != CRGB::Green)
                fillLight(CRGB::Green);
        } else {
            drivetrainStop();
            if (millis() < idleTime)
                readyIndication();
            else
                fillRainbow(millis() >> 3);
        }
    }
    while (readStartButton()) {
        fillLight(CRGB::White);
    }
    displayTeamColor();
    //setIntakePower(INTAKE_POWER / 3);
    //  resetIMU();
    resetAngleEstimator();
    // enableIMUfix();
    double rotateDeg = 45;
    rotateGlobal(rotateDeg);
    setIntakePower(INTAKE_POWER / 2);
    while (!readStartButton()) {
        movementUpdate();
        // blinkLights(colorToCode(teamFieldColor), CRGB::White, 3000, 50);
    }
    drivetrainStop();
    intakeEnable(true);
    #if ROBOT == SUPERLEICHT
    moveKill(1000);
    #else
    moveKill(1500);
    #endif // ROBOT
    for (;;) {
        /*
        rotateGlobal(angle);
        if (readStartButton())
          angle = (angle < 45) ? 90 : 0;
          */

        // waitStartButton();
        moveToWall();
        rotate(random(85, 179));
        if(robotIsOnTeamField)
            move(-15);
    }
    // while(!myIMU.dataAvailable());
    // angleReference = myIMU.getYaw();
}

#ifdef DEBUG_LOOP_TIMES

#endif
Stopwatch loopTimer = Stopwatch();
Stopwatch loopTimer2 = Stopwatch();

void loop() {
    updateIMU();
    // Serial.print("IMU-");
    // Serial.println(loopTimer.milliseconds());
    //  loopTimer.reset();
    updateAngleEstimator();
    // Serial.print("AE-");
    // Serial.println(loopTimer.milliseconds());
    //  loopTimer.reset();
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
    // Serial.print("DT-");
    //  Serial.println(loopTimer.milliseconds());
    //  loopTimer.reset();
    updateBarrier();
    // Serial.print("BA-");
    // Serial.println(loopTimer.milliseconds());
    // loopTimer.reset();
// updateLight();
#ifdef DEBUG_LOOP_TIMES
    {
        double time = loopTimer2.milliseconds();
        if (time > 25.0) {
        Serial.print("L");
        Serial.println(int(time));
        }
        loopTimer2.reset();
    }
#endif
}
