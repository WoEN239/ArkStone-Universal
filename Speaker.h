//
// Created by oaleksander on 22.10.2022.
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

const double k_frequency[] = {14734.2960228674, -182.2679956773, 1.13910128641602, -0.00408855355555643, 0.00000870830821415256, -0.000000010840344156628, -0.000000010840344156628, 0.00000000000726733965935332, -0.00000000000000202297588524196};

void speakerTone(int8_t secondMotorPower, uint16_t frequency, uint32_t duration) {
  double f[8] = {1, (double)frequency};
  f[2] = f[1] * f[1];
  f[3] = f[1] * f[2];
  f[4] = f[2] * f[2];
  f[5] = f[2] * f[3];
  f[6] = f[3] * f[3];
  f[7] = f[3] * f[4];
  double dDelay = 0;
  for(uint8_t i = 0; i < 8; i++)
    dDelay += k_frequency[i] * f[i];
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
