//
// Created by User on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_STARTBUTTTON_H
#define ARKSTONE_UNIVERSAL_STARTBUTTTON_H

#include "RobotConfig.h"
#include <Arduino.h>
#include <Prizm_Controller.h>

uint8_t readStartButton() {
  uint8_t startButton;
  if (START_BUTTON_PIN != NOT_A_PIN) {
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    startButton = digitalRead(START_BUTTON_PIN);
#ifndef START_BUTTON_INVERSE
    startButton = !startButton;
#endif
  } else
    startButton = false;
  uint8_t pressed = startButton || Prizm.readStartButton();
#ifdef DEBUG_START_BUTTON
  Serial.print("Start button value: ");
  Serial.println(pressed);
#endif  //DEBUG_START_BUTTON
  return pressed;
}

void waitStartButton() {
  while (!readStartButton()) {
#ifdef DEBUG_START_BUTTON
    Serial.println("Waiting start button...");
#endif  //DEBUG_START_BUTTON
  }
  delay(100);
  while (readStartButton())
    ;
#ifdef DEBUG_START_BUTTON
  Serial.println("Start button pressed!");
#endif  //DEBUG_START_BUTTON
}

#endif  //ARKSTONE_UNIVERSAL_STARTBUTTTON_H
