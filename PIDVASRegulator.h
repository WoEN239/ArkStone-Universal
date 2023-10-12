//
// Created by DmitriiDenisov on 16.10.2022.
//
//extern "C" {
#ifndef ARKSTONEPIONEER_PRIZM_PIDVASREGULATOR_H
#define ARKSTONEPIONEER_PRIZM_PIDVASREGULATOR_H

#include "MathExtend.hpp"
#include <Arduino.h>
#include "Stopwatch.h"

struct pidfParameters {
    double kP;
    double kI;
    double kD;
    double kF;
    double maxI;
    double minTimeDelta;
};

struct pidfState {
    double timeOld;
    double errorOld;
    double I;
    double D;
    double lastDeltaTime;
};

double pidfUpdate(struct pidfParameters parameters, struct pidfState *state, double error) {
    double timeNow = timeSeconds();
    double P = error * parameters.kP;
    state->I = state-> I + (parameters.kI * error) * (timeNow - state->timeOld);
    if (abs(state->I) > parameters.maxI) state->I = sign(state->I) * parameters.maxI;
    if (timeNow - state->lastDeltaTime > parameters.minTimeDelta) {
        state->D = (error - state->errorOld) * parameters.kD / (timeNow - state->lastDeltaTime);
        state->lastDeltaTime = timeNow;
        state->errorOld = error;
    }
    state->timeOld = timeNow;
    double F = parameters.kF * sign(error);
    return (P + state->I + state->D + F);
}

void pidfReset(struct pidfState *state) {
    state->I = 0.0;
    state->errorOld = 0.0;
    state->timeOld = 0.0;
    state->lastDeltaTime = 0.0;
    state->D = 0.0;
}

struct pidfState pidfInit(struct pidfParameters parameters, double error) {
    struct pidfState state;
    state.I = .0;
    state.timeOld = timeSeconds();
    state.lastDeltaTime = state.timeOld;
    state.errorOld = error;
    return state;
}

#endif //ARKSTONEPIONEER_PRIZM_PIDVASREGULATOR_H