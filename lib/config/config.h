#ifndef OPENDRUM_CONFIG_H_
#define OPENDRUM_CONFIG_H_

#include <Arduino.h>

#include "midi_drums.h"

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

// Definition of drumming zones with boundaries in degrees. There are 6 zones -
// 3 above the BOUNDARY_HORIZONTAL and 3 below. The 3 zones above and below are
// separated by the BOUNDARY_VERTICAL_LEFT and BOUNDARY_VERTICAL_RIGHT.
const int BOUNDARY_HORIZONTAL = 0;
const int BOUNDARY_VERTICAL_LEFT = -45;
const int BOUNDARY_VERTICAL_RIGHT = 45;

// Definition of drums.
const byte DRUM_BOTTOM_LEFT = drums::CLOSED_HI_HAT;
const byte DRUM_BOTTOM_MIDDLE = drums::ELECTRIC_SNARE;
const byte DRUM_BOTTOM_RIGHT = drums::LOW_FLOOR_TOM;
const byte DRUM_TOP_LEFT = drums::LOW_MID_TOM;
const byte DRUM_TOP_MIDDLE = drums::LOW_TOM;
const byte DRUM_TOP_RIGHT = drums::CRASH_CYMBAL_1;

#endif  // OPENDRUM_CONFIG_H_