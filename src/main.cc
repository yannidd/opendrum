/**
 * Source code for OpenDrum.
 *
 * The program is separated into tasks which run in separate threads. Tasks are
 * defined in <task name>_task() functions. Communication between tasks is done
 * via queues named mail_from_<task 1>_to_<task 2> and event flags named
 * <NAME>_FLAG.
 */

#include <Arduino.h>
#include <BLEMIDI_Transport.h>
#include <BasicLinearAlgebra.h>
#include <Duration.h>
#include <MatrixMath.h>
#include <Wire.h>
#include <hardware/BLEMIDI_ArduinoBLE.h>
#include <mbed.h>
#include <rtos.h>

#include <limits>

#include "ahrs.h"
#include "config.h"
#include "error.h"
#include "marg.h"
#include "misc.h"
#include "types.h"

using namespace rtos;
using namespace std;
using namespace std::chrono;

// BLE -------------------------------------------------------------------------
BLEMIDI_CREATE_INSTANCE("Drumless", MIDI);

// Event flags -----------------------------------------------------------------
const int BLE_IS_CONNECTED_FLAG = (1UL << 0);
const int RUN_SENSOR_TASK_FLAG = (1UL << 1);
const int RUN_BLE_TASK_FLAG = (1UL << 2);
const int RUN_BUTTON_TASK_FLAG = (1UL << 3);
const int BUTTON_PRESSED_FLAG = (1UL << 4);
const int BUTTON_DOUBLE_PRESSED_FLAG = (1UL << 5);
const int BUTTON_TRIPLE_PRESSED_FLAG = (1UL << 6);
const int BUTTON_LONG_PRESSED_FLAG = (1UL << 7);
static EventFlags event_flags;

// Mails for data passing between threads --------------------------------------
Mail<sensorDataContainer, MAIL_SIZE> mail_from_sensor_to_fusion;
Mail<sensorDataContainer, MAIL_SIZE> mail_from_sensor_to_detection;
Mail<fusionDataContainer, MAIL_SIZE> mail_from_fusion_to_detection;
Mail<midiDataContainer, MAIL_SIZE> mail_from_detection_to_ble;

/**
 * Sensor task for real-time data acquisition.
 *
 * The sensor task is responsible for querying sensor data from the IMU at a
 * constant frequency defined by F_SAMPLE Hz.
 */
void sensor_task() {
  MARG marg;
  float g_bias[3] = {0.061999, 0.090717, -0.037404};

  if (!marg.begin()) error_blink(ERROR_SENSOR_INIT);
  marg.set_gyro_calib(g_bias);

  mbed::Ticker timer;
  timer.attach([]() { event_flags.set(RUN_SENSOR_TASK_FLAG); },
               chrono::milliseconds(T_SAMPLE));

  float a[3], g[3], m[3], t;

  int start = micros();

  int iteration_counter = 0;
  while (true) {
    event_flags.wait_all(RUN_SENSOR_TASK_FLAG);

    iteration_counter++;

    marg.read(a, g, m, t);

    // Trigger fusion task every N_TODO iterations.
    if (iteration_counter == FUSE_DECIMATE_FACTOR) {
      iteration_counter = 0;

      sensorDataContainer *message_to_fusion =
          mail_from_sensor_to_fusion.try_alloc();
      if (message_to_fusion != nullptr) {
        copy(begin(a), end(a), begin(message_to_fusion->accel));
        copy(begin(g), end(g), begin(message_to_fusion->gyro));
        copy(begin(m), end(m), begin(message_to_fusion->mag));
        mail_from_sensor_to_fusion.put(message_to_fusion);
      }

      sensorDataContainer *message_to_detection =
          mail_from_sensor_to_detection.try_alloc();
      if (message_to_detection != nullptr) {
        copy(begin(a), end(a), begin(message_to_detection->accel));
        copy(begin(g), end(g), begin(message_to_detection->gyro));
        copy(begin(m), end(m), begin(message_to_detection->mag));
        mail_from_sensor_to_detection.put(message_to_detection);
      }
    }

    // Measure fps.
    int end = micros();
    // Serial.println(end - start);
    start = end;
  }
}

/**
 * Fusion task for AHRS.
 *
 * The fusion task is responsible for fusing the accelerometer, gyroscope and
 * magnetometer readings into attitude and heading information. It is called at
 * a constant frequency of F_SAMPLE/FUSE_DECIMATION_FACTOR Hz.
 */
void fusion_task() {
  // Mahony filter;
  Madgwick filter;
  // TODO: Investigate why the frequency needs to be multiplied by 2.
  filter.begin(2000.0 / (T_SAMPLE * FUSE_DECIMATE_FACTOR));
  int start = micros();
  float a[3], g[3], m[3];
  float euler[3], quat[4];
  int tmp_counter = 0;

  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *message_from_sensor =
        mail_from_sensor_to_fusion.try_get_for(FOREVER);
    if (message_from_sensor != nullptr) {
      copy(begin(message_from_sensor->accel), end(message_from_sensor->accel),
           begin(a));
      copy(begin(message_from_sensor->gyro), end(message_from_sensor->gyro),
           begin(g));
      copy(begin(message_from_sensor->mag), end(message_from_sensor->mag),
           begin(m));
      mail_from_sensor_to_fusion.free(message_from_sensor);

      // Fuse the sensor data to attitude and heading...
      filter.update(g[0], g[1], g[2], a[0], a[1], a[2], 0, 0, 0);
      filter.get_angles(euler);

      // Send the attitude and heading to the detection task...
      fusionDataContainer *message_to_detection =
          mail_from_fusion_to_detection.try_alloc();
      if (message_to_detection != nullptr) {
        copy(begin(euler), end(euler), begin(message_to_detection->euler));
        mail_from_fusion_to_detection.put(message_to_detection);
      }

      // Measure fps.
      int end = micros();
      tmp_counter++;
      if (tmp_counter % 10 == 0) {
        Serial.print((int)euler[0]);
        Serial.print(" ");
        Serial.print((int)euler[1]);
        Serial.print(" ");
        Serial.println((int)euler[2]);
      }
      // Serial.println(end - start);
      start = end;
    }

    if (event_flags.get() & BUTTON_PRESSED_FLAG) {
      filter.clear_reference();
      filter.get_quaternion(quat);
      filter.set_reference(quat);
    }
  }
}

/**
 * Task for sending BLE MIDI messages.
 *
 * This task is responsible for sending BLE MIDI messages with the appropriate
 * note and velocity values, whenever a drumstrike has been detected.
 */
void ble_task() {
  mbed::Ticker timer;
  timer.attach([]() { event_flags.set(RUN_BLE_TASK_FLAG); },
               chrono::milliseconds(T_SAMPLE));

  while (true) {
    event_flags.wait_any(RUN_BLE_TASK_FLAG);
    MIDI.read();
    // Check for a drum strike, non-blocking.
    midiDataContainer *message_from_detection =
        mail_from_detection_to_ble.try_get_for(0s);
    if (message_from_detection != nullptr) {
      byte note = message_from_detection->note;
      byte velocity = message_from_detection->velocity;
      mail_from_detection_to_ble.free(message_from_detection);
      MIDI.sendNoteOn(note, velocity, 10);
    }
  }
}

/**
 * Task for drum strike detection.
 *
 * This task runs the drum strike detection algorithm. It determines if a drum
 * has been struck, the type of the drum and the striking velocity. It takes
 * gyroscope readings, attitude and heading information.
 */
void detection_task() {
  DifferentiableValue g(T_SAMPLE / 1000.0);
  unsigned long time_of_last_strike = 0;
  float euler[3];
  byte drums[2][3] = {
      {DRUM_TOP_LEFT, DRUM_TOP_MIDDLE, DRUM_TOP_RIGHT},
      {DRUM_BOTTOM_LEFT, DRUM_BOTTOM_MIDDLE, DRUM_BOTTOM_RIGHT}};

  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *message_from_sensor =
        mail_from_sensor_to_detection.try_get_for(FOREVER);
    if (message_from_sensor != nullptr) {
      g.push(message_from_sensor->gyro[1]);
      mail_from_sensor_to_detection.free(message_from_sensor);
    }

    // Check for new AHRS data, non-blocking...
    fusionDataContainer *message_from_fusion =
        mail_from_fusion_to_detection.try_get_for(0s);
    if (message_from_fusion != nullptr) {
      copy(begin(message_from_fusion->euler), end(message_from_fusion->euler),
           begin(euler));
      mail_from_fusion_to_detection.free(message_from_fusion);
    }

    unsigned long now = millis();
    if ((g.x[0] > STRIKE_GYRO_THRESHOLD) &&
        ((g.dx[1] >= 0 && g.dx[0] < 0) || (g.dx[1] > 0 && g.dx[0] <= 0)) &&
        (g.ddx[0] < STRIKE_DDGYRO_THRESHOLD) &&
        (now - time_of_last_strike > STRIKE_TIME_SEPARATION)) {
      // Detect which drum was struck.
      int drum_row = 0, drum_col = 0;

      if (euler[1] > BOUNDARY_HORIZONTAL)  // UP
        drum_row = 0;
      else  // DOWN
        drum_row = 1;

      if (euler[2] < BOUNDARY_VERTICAL_LEFT)  // LEFT
        drum_col = 0;
      else if (euler[2] > BOUNDARY_VERTICAL_RIGHT)  // RIGHT
        drum_col = 2;
      else  // MIDDLE
        drum_col = 1;

      // Send a message to the ble task to request a sound play.
      midiDataContainer *message_to_ble =
          mail_from_detection_to_ble.try_alloc();
      if (message_to_ble != nullptr) {
        message_to_ble->note = drums[drum_row][drum_col];
        message_to_ble->velocity =
            27 + 100 * ((min(g.x[0], 34.9) - STRIKE_GYRO_THRESHOLD) /
                        (34.9 - STRIKE_GYRO_THRESHOLD));
        mail_from_detection_to_ble.put(message_to_ble);
      }

      time_of_last_strike = now;
    }
  }
}

void button_task() {
  bool curr_state = false;
  bool last_state = false;

  mbed::Ticker timer;
  timer.attach([]() { event_flags.set(RUN_BUTTON_TASK_FLAG); },
               chrono::milliseconds(T_SAMPLE));

  while (true) {
    event_flags.wait_any(RUN_BUTTON_TASK_FLAG);
    curr_state = !digitalRead(BUTTON_PIN);

    if (curr_state == true) {
      event_flags.set(BUTTON_PRESSED_FLAG);
    } else {
      event_flags.clear(BUTTON_PRESSED_FLAG);
    }

    last_state = curr_state;
  }
}

void setup() {
  // GPIO init.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Serial init.
  Serial.begin(115200);

  // BLE MIDI init.
  BLEMIDI.setHandleConnected([]() {
    digitalWrite(LED_BUILTIN, HIGH);
    event_flags.set(BLE_IS_CONNECTED_FLAG);
  });
  BLEMIDI.setHandleDisconnected([]() {
    digitalWrite(LED_BUILTIN, LOW);
    event_flags.clear(BLE_IS_CONNECTED_FLAG);
  });
  MIDI.begin();

  // Thread definition.
  Thread sensor_thread(osPriorityRealtime);
  Thread ble_thread(osPriorityRealtime);
  Thread fusion_thread(osPriorityRealtime);
  Thread detection_thread(osPriorityRealtime);
  Thread button_thread(osPriorityRealtime);

  // Start threads.
  sensor_thread.start(mbed::callback(sensor_task));
  fusion_thread.start(mbed::callback(fusion_task));
  detection_thread.start(mbed::callback(detection_task));
  ble_thread.start(mbed::callback(ble_task));
  button_thread.start(mbed::callback(button_task));

  // Put main thread to sleep.
  ThisThread::sleep_for(FOREVER);
}

void loop() {}
