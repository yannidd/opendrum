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

class DifferentiableValue {
 private:
  float _x[3] = {};
  float _dx[3] = {};
  float _ddx[3] = {};
  float _delta_t;

 public:
  DifferentiableValue(float delta_t) : _delta_t(delta_t) {}
  void push(float x) {
    // http://www.engineering.uco.edu/~aaitmoussa/Courses/ENGR3703/Chapter6/ch6.pdf
    // pages 6 and 7
    _x[2] = _x[1];
    _x[1] = _x[0];
    _x[0] = x;

    float dx = (_x[0] - _x[1]) / _delta_t;
    float ddx = (_x[2] - 2 * _x[1] + _x[0]) / (_delta_t * _delta_t);

    _dx[2] = _dx[1];
    _dx[1] = _dx[0];
    _dx[0] = dx;

    _ddx[2] = _ddx[1];
    _ddx[1] = _ddx[0];
    _ddx[0] = ddx;
  }

  const float (&x)[3] = _x;
  const float (&dx)[3] = _dx;
  const float (&ddx)[3] = _ddx;
};

#endif  // OPENDRUM_TYPES_H_