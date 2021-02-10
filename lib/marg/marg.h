#ifndef OPENDRUM_MARG_H_
#define OPENDRUM_MARG_H_

#include <Wire.h>

class MARG {
 private:
  TwoWire* _wire;

  // Scaling values for sensors. Converts to real units.
  float _g_per_lsb;
  float _dps_per_lsb;
  float _mgauss_per_lsb;

  // Calibration values.
  float _g_bias[3] = {0.0};

  int read_reg(uint8_t slave_address, uint8_t address);
  int read_regs(uint8_t slave_address, uint8_t address, uint8_t* data,
                size_t length);
  int write_reg(uint8_t slave_address, uint8_t address, uint8_t value);

 public:
  typedef enum {
    WHO_AM_I_XG = 0x0F,
    CTRL_REG1_G = 0x10,
    CTRL_REG2_G = 0x11,
    CTRL_REG3_G = 0x12,
    TEMP_OUT_L = 0x15,
    TEMP_OUT_H = 0x16,
    STATUS_REG = 0x17,
    OUT_X_L_G = 0x18,
    OUT_X_H_G = 0x19,
    OUT_Y_L_G = 0x1A,
    OUT_Y_H_G = 0x1B,
    OUT_Z_L_G = 0x1C,
    OUT_Z_H_G = 0x1D,
    CTRL_REG4 = 0x1E,
    CTRL_REG5_XL = 0x1F,
    CTRL_REG6_XL = 0x20,
    CTRL_REG7_XL = 0x21,
    CTRL_REG8 = 0x22,
    CTRL_REG9 = 0x23,
    CTRL_REG10 = 0x24,
    OUT_X_L_XL = 0x28,
    OUT_X_H_XL = 0x29,
    OUT_Y_L_XL = 0x2A,
    OUT_Y_H_XL = 0x2B,
    OUT_Z_L_XL = 0x2C,
    OUT_Z_H_XL = 0x2D,
    // Mag registers:
    CTRL_REG1_M = 0x20,
    CTRL_REG2_M = 0x21,
    CTRL_REG3_M = 0x22,
    STATUS_REG_M = 0x27,
    OUT_X_L_M = 0x28,
  } registers;

  typedef enum {
    MAG = 0x1E,
    IMU = 0x6B,
  } slave;

  typedef enum {
    G2 = (0b00 << 3),
    G4 = (0b10 << 3),
    G8 = (0b11 << 3),
    G16 = (0b01 << 3),
  } accel_range;

  typedef enum {
    GAUSS4 = (0b00 << 5),
    GAUSS8 = (0b01 << 5),
    GAUSS12 = (0b10 << 5),
    GAUSS16 = (0b11 << 5),
  } mag_gain;

  typedef enum {
    DPS245 = (0b00 << 4),
    DPS500 = (0b01 << 4),
    DPS2000 = (0b11 << 4),
  } gyro_scale;

  MARG();
  bool begin();
  void setup_accel(accel_range range);
  void setup_gyro(gyro_scale scale);
  void setup_mag(mag_gain gain);
  void read_accel(float a[3], bool compensate = true);  // m/s^2
  void read_gyro(float g[3], bool compensate = true);   // rad/s
  void read_mag(float m[3], bool compensate = true);    // mGauss
  void read_temp(float& t);                             // degree C
  void read(float a[3], float g[3], float m[3], float& t,
            bool compensate = true);
  void set_gyro_calib(float bias[3]);
  void estimate_gyro_bias();
};

#endif  // OPENDRUM_MARG_H_