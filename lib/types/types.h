#ifndef OPENDRUM_TYPES_H_
#define OPENDRUM_TYPES_H_

#include <BasicLinearAlgebra.h>

using namespace std;

const chrono::hours FOREVER(numeric_limits<int>::max());

typedef struct sensorDataContainer {
  BLA::Matrix<3> accel;
  BLA::Matrix<3> gyro;
  BLA::Matrix<3> mag;
} sensorDataContainer;

typedef struct midiDataContainer {
  byte note;
  byte velocity;
} midiDataContainer;

#endif  // OPENDRUM_TYPES_H_