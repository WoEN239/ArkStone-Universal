#ifndef ARKSTONE_UNIVERSAL_LOWPASSFILTER_H
#define ARKSTONE_UNIVERSAL_LOWPASSFILTER_H

#include "Stopwatch.h"

class LowPassFilter {
   private:
    Stopwatch sampleTimer = Stopwatch();
    double state;
    double Tf;

   public:
    LowPassFilter(double T, double initialCondition = 0) {
        this->Tf = 1.0 / T;
        this->state = initialCondition;
    }

    void reset(double initialCondition = 0) {
        this->state = initialCondition;
        sampleTimer.reset();
    }

    double update(double input) {
        double Ts = sampleTimer.seconds();
        sampleTimer.reset();
        return this->state = nextStep(this->state, input, this->Tf, Ts);
    }

    static double nextStep(double state, double input, double Tf, double Ts) {
        float a = Tf / (Tf + Ts);
        return state * a + (1 - a) * input;
    }
};

#endif  // ARKSTONE_UNIVERSAL_LOWPASSFILTER_H