#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <inttypes.h>

#define MICROS_IN_SECOND 1000000
#define MILLIS_IN_SECOND 1000
#define MICROS_IN_MILLIS 1000

#define StopWatch Stopwatch

class Stopwatch {
private:
  uint32_t timeStart = 0;

public:
  void reset() {
    timeStart = micros();
  }

  uint32_t microseconds() {
    return micros() - timeStart;
  }

  float milliseconds() {
    return microseconds() / float(MICROS_IN_MILLIS);
  }

  float seconds() {
    return (float)microseconds() / (float)MICROS_IN_SECOND;
  }
};

double timeSeconds() {
  return micros() / (float)MICROS_IN_SECOND;
}

#endif  //STOPWATCH_H
