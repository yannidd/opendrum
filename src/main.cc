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

using namespace rtos;
using namespace std;
using namespace std::chrono;

// Config ----------------------------------------------------------------------
const int T_SAMPLE = 2;  // ms
const int FUSE_DECIMATE_FACTOR = 3;

// Misc ------------------------------------------------------------------------
const chrono::hours FOREVER(numeric_limits<int>::max());

// BLE -------------------------------------------------------------------------
BLEMIDI_CREATE_INSTANCE("Drumless", MIDI);

// Event flags -----------------------------------------------------------------
const int BLE_IS_CONNECTED_FLAG = (1UL << 0);
const int RUN_SENSOR_TASK_FLAG = (1UL << 1);
const int RUN_BLE_TASK_FLAG = (1UL << 2);
const int RUN_FUSION_TASK_FLAG = (1UL << 3);
EventFlags event_flags;

// Queues ----------------------------------------------------------------------
typedef struct sensorDataContainer {
  BLA::Matrix<3> accel;
  BLA::Matrix<3> gyro;
  BLA::Matrix<3> mag;
} sensorDataContainer;

typedef struct midiDataContainer {
  byte note;
  byte velocity;
} midiDataContainer;

Mail<sensorDataContainer, 4> sensor2fusion_pipe;
Mail<sensorDataContainer, 4> sensor2detection_pipe;
Mail<BLA::Matrix<4>, 4> fusion2detection_pipe;
Mail<midiDataContainer, 4> detection2ble_pipe;

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

      sensorDataContainer *sensor2fusion_message =
          sensor2fusion_pipe.try_alloc();
      sensor2fusion_message->accel = a;
      sensor2fusion_message->gyro = g;
      sensor2fusion_message->mag = m;
      sensor2fusion_pipe.put(sensor2fusion_message);

      sensorDataContainer *sensor2detection_message =
          sensor2detection_pipe.try_alloc();
      sensor2detection_message->accel = a;
      sensor2detection_message->gyro = g;
      sensor2detection_message->mag = m;
      sensor2detection_pipe.put(sensor2detection_message);
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
  filter.begin(2000.0 / (T_SAMPLE * FUSE_DECIMATE_FACTOR));
  int start = micros();
  BLA::Matrix<3> a, g, m;
  int tmp_counter = 0;

  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *sensor2fusion_message =
        sensor2fusion_pipe.try_get_for(FOREVER);
    if (sensor2fusion_message != nullptr) {
      a = sensor2fusion_message->accel;
      g = sensor2fusion_message->gyro;
      m = sensor2fusion_message->mag;
      sensor2fusion_pipe.free(sensor2fusion_message);

      // Fuse the sensor data to attitude and heading...
      filter.update(g(0), -g(1), g(2), a(0), -a(1), a(2), 0.0, 0.0, 0.0);


      // Send the attitude and heading to the detection task...
      BLA::Matrix<4> *fusion2detection_message =
          fusion2detection_pipe.try_alloc();
      *fusion2detection_message << 0.0, 0.0, 1.0, 0.0;
      fusion2detection_pipe.put(fusion2detection_message);

      // Measure fps.
      int end = micros();      
      tmp_counter++;
      if (tmp_counter % 10 == 0) {
        Serial.print((int)filter.getYaw());
        Serial.print(" ");
        Serial.print((int)filter.getPitch());
        Serial.print(" ");
        Serial.println((int)filter.getRoll());
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
    if ((event_flags.get() & BLE_IS_CONNECTED_FLAG) && (millis() - t0) > 1000) {
      t0 = millis();
      MIDI.sendNoteOn(60, 100, 1);
    }
  }
}

/**
 * Task for drum strike detection.
 *
 * This task runs the drum strike detection algorithm. It determines if a
 drum
 * has been struck, the type of the drum and the striking velocity. It takes
 * gyroscope readings, attitude and heading information.
 */
void detection_task() {
  while (true) {
    // Wait for new sensor data...
    sensorDataContainer *sensor2detection_message =
        sensor2detection_pipe.try_get_for(FOREVER);
    if (sensor2detection_message != nullptr) {
      sensor2detection_pipe.free(sensor2detection_message);
    }

    // Check for new AHRS data, non-blocking...
    BLA::Matrix<4> *q = fusion2detection_pipe.try_get_for(0s);
    if (q != nullptr) {
      // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      fusion2detection_pipe.free(q);
    }
  }
}

void setup() {
  // General setup.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  BLEMIDI.setHandleConnected(OnConnected);
  BLEMIDI.setHandleDisconnected(OnDisconnected);
  MIDI.begin();

  // Thread definition.
  Thread sensor_thread(osPriorityRealtime);
  Thread ble_thread(osPriorityRealtime);
  Thread fusion_thread(osPriorityRealtime);
  Thread detection_thread(osPriorityRealtime);

  // // Start threads.
  sensor_thread.start(mbed::callback(sensor_task));
  ble_thread.start(mbed::callback(ble_task));
  fusion_thread.start(mbed::callback(fusion_task));
  detection_thread.start(mbed::callback(detection_task));

  // // Put main thread to sleep.
  ThisThread::sleep_for(FOREVER);
}

void loop() {}
