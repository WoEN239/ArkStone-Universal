#ifndef ARKSTONE_UNIVERSAL_LOWPASSFILTER_H
#define ARKSTONE_UNIVERSAL_LOWPASSFILTER_H

#include "Stopwatch.h"

class LowPassFilter {
private:
  Stopwatch sampleTimer = Stopwatch();
  double state;
  double inverseT;
public:
  LowPassFilter(double T, double initialCondition = 0) {
    this->inverseT = 1.0 / T;
    this->state = initialCondition;
  }

  void reset(double initialCondition = 0) {
    this->state = initialCondition;
    sampleTimer.reset();
  }

  double update(double input) {
    double sampleTime = sampleTimer.seconds();
    sampleTimer.reset();
    return state += sampleTime * inverseT * (input - state);
  }
};

#endif  //ARKSTONE_UNIVERSAL_LOWPASSFILTER_H