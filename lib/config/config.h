#ifndef OPENDRUM_CONFIG_H_
#define OPENDRUM_CONFIG_H_

#include "Arduino.h"

const int T_SAMPLE = 2;  // ms
const int FUSE_DECIMATE_FACTOR = 2;
const int MAIL_SIZE = 4;

// The angular velocity threshold for detecting a strike. A higher value will
// result in the rejection of low velocity strikes.
const float STRIKE_GYRO_THRESHOLD = 4.8869;  // rad/s

// The second time derivative of angular velocity threshold for detecting a
// strike. A lower value will result in the rejection of non-sharp strikes.
const float STRIKE_DDGYRO_THRESHOLD = -50000.0;  // rad/s^3

// The minimum allowed time between two strikes. If you hear more than one note
// being played on a single strike, increase this value.
const float STRIKE_TIME_SEPARATION = 20;  // ms

// Pin definitions.
const int BUTTON_PIN = D2;

#endif  // OPENDRUM_CONFIG_H_