//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
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
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>

/**
 * Compute 1 / sqrt(x) fast.
 *
 * Estimates ​the reciprocal of the square root of a 32-bit floating-point
 * number. See http://en.wikipedia.org/wiki/Fast_inverse_square_root for more
 * information.
 *
 * @param The input number.
 * @return The reciprocal of the square root.
 */
float fast_inv_sqrt(float x);

/**
 * Compute the conjugate of a quaternion.
 *
 * @param q The input quaternion.
 * @param q_conj The output quaternion.
 */
void quat_conj(float q[4], float q_conj[4]);

/**
 * Multiply two quaternions.
 *
 * @param q1 The first quaternion.
 * @param q2 The second quaternion.
 * @param q_res The resulting quaternion.
 */
void quat_mul(float q1[4], float q2[4], float q_res[4]);

class Madgwick {
 private:
  float _beta;               // algorithm gain
  float _q0, _q1, _q2, _q3;  // quaternion from sensor frame to world frame
  float _q_ref[4];           // a reference quaternion frame
  float _q_out[4];           // quaternion from sensor frame to reference frame
  float _roll;
  float _pitch;
  float _yaw;
  float invSampleFreq;
  bool angles_computed;
  void compute_angles();

 public:
  Madgwick(void);
  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  void update_imu(float gx, float gy, float gz, float ax, float ay, float az);
  void get_quaternion(float q[4]);
  void get_angles(float a[3]);
  void set_reference(float q[4]);
  void get_reference(float q[4]);
  void clear_reference();
};

class Mahony {
 private:
  float twoKp;  // 2 * proportional gain (Kp)
  float twoKi;  // 2 * integral gain (Ki)
  float q0, q1, q2,
      q3;  // quaternion of sensor frame relative to auxiliary frame
  float integralFBx, integralFBy,
      integralFBz;  // integral error terms scaled by Ki
  float invSampleFreq;
  float roll, pitch, yaw;
  char anglesComputed;
  static float invSqrt(float x);
  void computeAngles();

  //-------------------------------------------------------------------------------------------
  // Function declarations

 public:
  Mahony();
  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
  void update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
  float getRoll() {
    if (!anglesComputed) computeAngles();
    return roll * 57.29578f;
  }
  float getPitch() {
    if (!anglesComputed) computeAngles();
    return pitch * 57.29578f;
  }
  float getYaw() {
    if (!anglesComputed) computeAngles();
    return yaw * 57.29578f + 180.0f;
  }
  float getRollRadians() {
    if (!anglesComputed) computeAngles();
    return roll;
  }
  float getPitchRadians() {
    if (!anglesComputed) computeAngles();
    return pitch;
  }
  float getYawRadians() {
    if (!anglesComputed) computeAngles();
    return yaw;
  }
};
#endif
