#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
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
const int RUN_FUSION_TASK_FLAG = (1UL << 3);
EventFlags event_flags;

// Mails for data passing between threads --------------------------------------
Mail<sensorDataContainer, MAIL_SIZE> mail_from_sensor_to_fusion;
Mail<sensorDataContainer, MAIL_SIZE> mail_from_sensor_to_detection;
Mail<BLA::Matrix<3>, MAIL_SIZE> mail_from_fusion_to_detection;
Mail<midiDataContainer, MAIL_SIZE> mail_from_detection_to_ble;

void OnConnected() {
  digitalWrite(LED_BUILTIN, HIGH);
  event_flags.set(BLE_IS_CONNECTED_FLAG);
}

void OnDisconnected() {
  digitalWrite(LED_BUILTIN, LOW);
  event_flags.clear(BLE_IS_CONNECTED_FLAG);
}

/**
 * Sensor task for real-time data acquisition.
 *
 * The sensor task is responsible for querying sensor data from the IMU at a
 * constant frequency defined by F_SAMPLE Hz.
 */
void sensor_task() {
  Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(&Wire1);

  mbed::Ticker timer;
  timer.attach([]() { event_flags.set(RUN_SENSOR_TASK_FLAG); },
               chrono::milliseconds(T_SAMPLE));

  Serial.println("Starting IMU!");
  if (!imu.begin()) {
    Serial.println("Unable to initialize the LSM9DS1!");
    while (1) {
    };
  }
  Serial.println("Found LSM9DS1 9DOF");

  imu.setupGyro(imu.LSM9DS1_GYROSCALE_2000DPS);
  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_16G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
  Wire1.setClock(400000);
  float a_scalar = SENSORS_GRAVITY_STANDARD * LSM9DS1_ACCEL_MG_LSB_16G / 1000;
  float g_scalar = LSM9DS1_GYRO_DPS_DIGIT_2000DPS * SENSORS_DPS_TO_RADS;
  BLA::Matrix<3> a, g, m;
  BLA::Matrix<3> g_bias = {0.061999, -0.090717, -0.037404};

  int start = micros();

  int iteration_counter = 0;
  while (true) {
    event_flags.wait_all(RUN_SENSOR_TASK_FLAG);

    iteration_counter++;

    imu.read();
    a << imu.accelData.x * a_scalar, imu.accelData.y * a_scalar,
        imu.accelData.z * a_scalar;
    g << imu.gyroData.x * g_scalar, imu.gyroData.y * g_scalar,
        imu.gyroData.z * g_scalar;
    m << 0.0, 0.0, 0.0;

    g -= g_bias;

    // Trigger fusion task every N_TODO iterations.
    if (iteration_counter == FUSE_DECIMATE_FACTOR) {
      iteration_counter = 0;

      sensorDataContainer *message_to_fusion =
          mail_from_sensor_to_fusion.try_alloc();
      message_to_fusion->accel = a;
      message_to_fusion->gyro = g;
      message_to_fusion->mag = m;
      mail_from_sensor_to_fusion.put(message_to_fusion);

      sensorDataContainer *message_to_detection =
          mail_from_sensor_to_detection.try_alloc();
      message_to_detection->accel = a;
      message_to_detection->gyro = g;
      message_to_detection->mag = m;
      mail_from_sensor_to_detection.put(message_to_detection);
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
 * magnetometer readings into attitude and heading information. It is called
 at
 * a constant frequency of F_SAMPLE/FUSE_DECIMATION_FACTOR Hz.
 */
void fusion_task() {
  // Mahony filter;
  Madgwick filter;
  // TODO: Investigate why the frequency needs to be multiplied by 2.
  filter.begin(2000.0 / (T_SAMPLE * FUSE_DECIMATE_FACTOR));
  int start = micros();
  BLA::Matrix<3> a, g, m;
  int tmp_counter = 0;

  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *message_from_sensor =
        mail_from_sensor_to_fusion.try_get_for(FOREVER);
    if (message_from_sensor != nullptr) {
      a = message_from_sensor->accel;
      g = message_from_sensor->gyro;
      m = message_from_sensor->mag;
      mail_from_sensor_to_fusion.free(message_from_sensor);

      // Fuse the sensor data to attitude and heading...
      filter.update(g(0), -g(1), g(2), a(0), -a(1), a(2), 0.0, 0.0, 0.0);

      // Send the attitude and heading to the detection task...
      BLA::Matrix<3> *message_to_detection =
          mail_from_fusion_to_detection.try_alloc();
      *message_to_detection = filter.getAngles();
      mail_from_fusion_to_detection.put(message_to_detection);

      // Measure fps.
      int end = micros();
      tmp_counter++;
      if (tmp_counter % 10 == 0) {
        // Serial.print((int)filter.getYaw());
        // Serial.print(" ");
        // Serial.print((int)filter.getPitch());
        // Serial.print(" ");
        // Serial.println((int)filter.getRoll());
      }
      // Serial.println(end - start);
      start = end;
    }
  }
}

/**
 * Task for sending BLE MIDI messages.
 *
 * This task is responsible for sending BLE MIDI messages with the
 appropriate
 * note and velocity values, whenever a drumstrike has been detected.
 */
void ble_task() {
  unsigned long t0 = millis();
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

  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *message_from_sensor =
        mail_from_sensor_to_detection.try_get_for(FOREVER);
    if (message_from_sensor != nullptr) {
      g.push(message_from_sensor->gyro(1));
      mail_from_sensor_to_detection.free(message_from_sensor);
    }

    // Check for new AHRS data, non-blocking...
    BLA::Matrix<3> *message_from_fusion =
        mail_from_fusion_to_detection.try_get_for(0s);
    if (message_from_fusion != nullptr) {
      mail_from_fusion_to_detection.free(message_from_fusion);
    }

    unsigned long now = millis();
    if ((g.x[0] > STRIKE_GYRO_THRESHOLD) &&
        ((g.dx[1] >= 0 && g.dx[0] < 0) || (g.dx[1] > 0 && g.dx[0] <= 0)) &&
        (g.ddx[0] < STRIKE_DDGYRO_THRESHOLD) &&
        (now - time_of_last_strike > STRIKE_TIME_SEPARATION)) {
      // Send a message to the ble task to request a sound play.
      midiDataContainer *message_to_ble =
          mail_from_detection_to_ble.try_alloc();
      message_to_ble->note = 1;
      message_to_ble->velocity =
          27 + 100 * ((min(g.x[0], 34.9) - STRIKE_GYRO_THRESHOLD) /
                      (34.9 - STRIKE_GYRO_THRESHOLD));
      mail_from_detection_to_ble.put(message_to_ble);

      time_of_last_strike = now;
    }
  }
}

void setup() {
  // GPIO init.
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial init.
  Serial.begin(115200);

  // BLE MIDI init.
  BLEMIDI.setHandleConnected(OnConnected);
  BLEMIDI.setHandleDisconnected(OnDisconnected);
  MIDI.begin();

  // Thread definition.
  Thread sensor_thread(osPriorityRealtime);
  Thread ble_thread(osPriorityRealtime);
  Thread fusion_thread(osPriorityRealtime);
  Thread detection_thread(osPriorityRealtime);

  // Start threads.
  sensor_thread.start(mbed::callback(sensor_task));
  fusion_thread.start(mbed::callback(fusion_task));
  detection_thread.start(mbed::callback(detection_task));
  ble_thread.start(mbed::callback(ble_task));

  // Put main thread to sleep.
  ThisThread::sleep_for(FOREVER);
}

void loop() {}
