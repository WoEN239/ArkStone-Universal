//
// Created by User on 22.10.2022.
//

#ifndef ARKSTONE_UNIVERSAL_SPEAKER_H
#define ARKSTONE_UNIVERSAL_SPEAKER_H

#include <Arduino.h>
#include "Prizm_DC_Expansion.h"
#include "RobotConfig.h"

void initSpeaker() {
#ifdef SPEAKER_STEREO
  speakerDcExpansion.setZeroPowerBehavior(1, BRAKE);
  speakerDcExpansion.setZeroPowerBehavior(2, BRAKE);
#else
  speakerDcExpansion.setZeroPowerBehavior(SPEAKER_PORT_NUMBER, BRAKE);
#endif  //SPEAKER_STEREO
}


void speakerTone(int8_t secondMotorPower, uint16_t frequency, uint32_t duration) {
  double f1 = (double)frequency;
  double f2 = f1 * f1;
  double f3 = f1 * f2;
  double f4 = f2 * f2;
  double f5 = f2 * f3;
  double f6 = f3 * f3;
  double f7 = f3 * f4;
  double dDelay = 14734.2960228674 - 182.2679956773 * f1 + 1.13910128641602 * f2 - 0.00408855355555643 * f3 + 0.00000870830821415256 * f4 - 0.000000010840344156628 * f5 + 0.00000000000726733965935332 * f6 - 0.00000000000000202297588524196 * f7;
  uint16_t iDelay = dDelay;
  uint32_t t_start = millis();
  int8_t speakerPower;
#ifndef SPEAKER_MUTE
  speakerPower = 100;
#endif  //SPEAKER_MUTE
  while (millis() - t_start < duration) {
#ifdef SPEAKER_STEREO
    speakerDcExpansion.setPowers(speakerPower, -speakerPower);
    delayMicroseconds(iDelay);
    speakerDcExpansion.setPowers(-speakerPower, speakerPower);
#else
    speakerDcExpansion.setPowers(speakerPower, secondMotorPower);
    delayMicroseconds(iDelay);
    speakerDcExpansion.setPowers(-speakerPower, secondMotorPower);
#endif  //SPEAKER_STEREO
    delayMicroseconds(iDelay);
  }
#ifdef SPEAKER_STEREO
  speakerDcExpansion.setPowers(0, 0);
#else
  speakerDcExpansion.setPowers(0, secondMotorPower);
#endif  //SPEAKER_STEREO
}

#endif  //ARKSTONEPIONEER_PRIZM_SPEAKER_H
