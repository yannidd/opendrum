//==============================================================================
// madgwick.c
//==============================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include <math.h>

#include "ahrs.h"
#include "types.h"

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef 250.0f  // sample frequency in Hz
// #define betaDef         0.1f            // 2 * proportional gain
#define betaDef 0.2f  // 2 * proportional gain

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

Madgwick::Madgwick() {
  _beta = betaDef;
  _q0 = 1.0f;
  _q1 = 0.0f;
  _q2 = 0.0f;
  _q3 = 0.0f;
  invSampleFreq = 1.0f / sampleFreqDef;
  angles_computed = false;
  clear_reference();
}

void Madgwick::update(float gx, float gy, float gz, float ax, float ay,
                      float az, float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  // clang-format off
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  // clang-format on

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    update_imu(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
  qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
  qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
  qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = fast_inv_sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * _q0 * mx;
    _2q0my = 2.0f * _q0 * my;
    _2q0mz = 2.0f * _q0 * mz;
    _2q1mx = 2.0f * _q1 * mx;
    _2q0 = 2.0f * _q0;
    _2q1 = 2.0f * _q1;
    _2q2 = 2.0f * _q2;
    _2q3 = 2.0f * _q3;
    _2q0q2 = 2.0f * _q0 * _q2;
    _2q2q3 = 2.0f * _q2 * _q3;
    q0q0 = _q0 * _q0;
    q0q1 = _q0 * _q1;
    q0q2 = _q0 * _q2;
    q0q3 = _q0 * _q3;
    q1q1 = _q1 * _q1;
    q1q2 = _q1 * _q2;
    q1q3 = _q1 * _q3;
    q2q2 = _q2 * _q2;
    q2q3 = _q2 * _q3;
    q3q3 = _q3 * _q3;

    // Reference direction of Earth's magnetic field
    // clang-format off
		hx = mx * q0q0 - _2q0my * _q3 + _2q0mz * _q2 + mx * q1q1 + _2q1 * my * _q2 + _2q1 * mz * _q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * _q3 + my * q0q0 - _2q0mz * _q1 + _2q1mx * _q2 - my * q1q1 + my * q2q2 + _2q2 * mz * _q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * _q2 + _2q0my * _q1 + mz * q0q0 + _2q1mx * _q3 - mz * q1q1 + _2q2 * my * _q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
    // clang-format on

    // Gradient decent algorithm corrective step
    // clang-format off
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * _q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _q3 + _2bz * _q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * _q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _q2 + _2bz * _q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _q3 - _4bz * _q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * _q2 - _2bz * _q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _q1 + _2bz * _q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _q0 - _4bz * _q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * _q3 + _2bz * _q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _q0 + _2bz * _q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    // clang-format on
    recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                              s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= _beta * s0;
    qDot2 -= _beta * s1;
    qDot3 -= _beta * s2;
    qDot4 -= _beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  _q0 += qDot1 * invSampleFreq;
  _q1 += qDot2 * invSampleFreq;
  _q2 += qDot3 * invSampleFreq;
  _q3 += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = fast_inv_sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
  _q0 *= recipNorm;
  _q1 *= recipNorm;
  _q2 *= recipNorm;
  _q3 *= recipNorm;

  angles_computed = false;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void Madgwick::update_imu(float gx, float gy, float gz, float ax, float ay,
                          float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
  qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
  qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
  qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * _q0;
    _2q1 = 2.0f * _q1;
    _2q2 = 2.0f * _q2;
    _2q3 = 2.0f * _q3;
    _4q0 = 4.0f * _q0;
    _4q1 = 4.0f * _q1;
    _4q2 = 4.0f * _q2;
    _8q1 = 8.0f * _q1;
    _8q2 = 8.0f * _q2;
    q0q0 = _q0 * _q0;
    q1q1 = _q1 * _q1;
    q2q2 = _q2 * _q2;
    q3q3 = _q3 * _q3;

    // Gradient decent algorithm corrective step
    // clang-format off
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
    // clang-format on
    recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                              s3 * s3);  // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= _beta * s0;
    qDot2 -= _beta * s1;
    qDot3 -= _beta * s2;
    qDot4 -= _beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  _q0 += qDot1 * invSampleFreq;
  _q1 += qDot2 * invSampleFreq;
  _q2 += qDot3 * invSampleFreq;
  _q3 += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = fast_inv_sqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
  _q0 *= recipNorm;
  _q1 *= recipNorm;
  _q2 *= recipNorm;
  _q3 *= recipNorm;

  angles_computed = false;
}

//-------------------------------------------------------------------------------------------

void Madgwick::clear_reference() {
  float q_temp[4] = {1, 0, 0, 0};
  quat_conj(q_temp, _q_ref);
  angles_computed = false;
}

void Madgwick::set_reference(float q[4]) {
  quat_conj(q, _q_ref);
  angles_computed = false;
}

void Madgwick::get_reference(float q[4]) {
  q[0] = _q_ref[0];
  q[1] = _q_ref[1];
  q[2] = _q_ref[2];
  q[3] = _q_ref[3];
}

void Madgwick::compute_angles() {
  if (angles_computed) return;

  float q[4] = {_q0, _q1, _q2, _q3};
  quat_mul(q, _q_ref, _q_out);

  float q0 = _q_out[0];
  float q1 = _q_out[1];
  float q2 = _q_out[2];
  float q3 = _q_out[3];

  _roll = RAD2DEG * atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  _pitch = -RAD2DEG * asinf(-2.0f * (q1 * q3 - q0 * q2));
  _yaw = -RAD2DEG * atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);

  angles_computed = true;
}

/**
 * Get the estimated orientation in quaternion.
 *
 * @return An array with quaternion values.
 */
void Madgwick::get_quaternion(float quaternion[4]) {
  compute_angles();
  quaternion[0] = _q_out[0];
  quaternion[1] = _q_out[1];
  quaternion[2] = _q_out[2];
  quaternion[3] = _q_out[3];
}

/**
 * Get the estimated orientation in Euler angles.
 *
 * @return An array with values for roll, pitch, and yaw in degrees.
 */
void Madgwick::get_angles(float euler[3]) {
  compute_angles();
  euler[0] = _roll;
  euler[1] = _pitch;
  euler[2] = _yaw;
}