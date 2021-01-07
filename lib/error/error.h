#ifndef OPENDRUM_ERROR_H_
#define OPENDRUM_ERROR_H_

#include "Arduino.h"

// Initialisation of the MARG sensor went wrong.
const int ERROR_SENSOR_INIT = 3;

void error_blink(int count) {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  while (true) {
    for (int i = 0; i < count; i++) {
      digitalWrite(LED_RED, LOW);
      delay(200);
      digitalWrite(LED_RED, HIGH);
      delay(200);
    }
    delay(2000);
  }
}

#endif  // OPENDRUM_ERROR_H_
