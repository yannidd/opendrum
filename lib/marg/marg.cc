#include <Wire.h>
#include <marg.h>

void join_xys_bytes(byte buffer[6], int16_t xyz[3]) {
  xyz[0] = buffer[0] | (buffer[1] << 8);
  xyz[1] = buffer[2] | (buffer[3] << 8);
  xyz[2] = buffer[4] | (buffer[5] << 8);
}

MARG::MARG() { _wire = &Wire1; }

bool MARG::begin() {
  _wire->begin();

  // Reset IMU & MAG.
  write_reg(slave::IMU, registers::CTRL_REG8, 0x05);
  write_reg(slave::MAG, registers::CTRL_REG2_M, 0x0C);

  delay(10);

  // Check if the sensors are responding correctly.
  if (read_reg(slave::IMU, registers::WHO_AM_I_XG) != 0x68) return false;
  if (read_reg(slave::MAG, registers::WHO_AM_I_XG) != 0x3d) return false;

  // Enable gyro continuous mode.
  write_reg(slave::IMU, registers::CTRL_REG1_G, 0xC0);

  // Enable accel continuous mode.
  write_reg(slave::IMU, registers::CTRL_REG5_XL, 0x38);
  // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing.
  write_reg(slave::IMU, registers::CTRL_REG6_XL, 0xC0);

  // Temperature compensation enable, ultra-high performance, >80 Hz.
  write_reg(slave::MAG, registers::CTRL_REG1_M, 0xFE);
  // Continuous conversion mode.
  write_reg(slave::MAG, registers::CTRL_REG3_M, 0x00);

  // Set default ranges for the various sensors
  setup_accel(accel_range::G16);
  setup_gyro(gyro_scale::DPS2000);
  setup_mag(mag_gain::GAUSS4);

  return true;
}

void MARG::setup_accel(accel_range range) {
  uint8_t reg = read_reg(slave::IMU, registers::CTRL_REG6_XL);
  reg &= ~(0b00011000);
  reg |= range;
  write_reg(slave::IMU, registers::CTRL_REG6_XL, reg);

  switch (range) {
    case G2:
      _g_per_lsb = 0.000061;
      break;
    case G4:
      _g_per_lsb = 0.000122;
      break;
    case G8:
      _g_per_lsb = 0.000244;
      break;
    case G16:
      _g_per_lsb = 0.000732;
      break;
  }
}

void MARG::setup_gyro(gyro_scale scale) {
  uint8_t reg = read_reg(slave::IMU, registers::CTRL_REG1_G);
  reg &= ~(0b00110000);
  reg |= scale;
  write_reg(slave::IMU, registers::CTRL_REG1_G, reg);

  switch (scale) {
    case DPS245:
      _dps_per_lsb = 0.00875;
      break;
    case DPS500:
      _dps_per_lsb = 0.01750;
      break;
    case DPS2000:
      _dps_per_lsb = 0.07000;
      break;
  }
}

void MARG::setup_mag(mag_gain gain) {
  write_reg(slave::MAG, registers::CTRL_REG2_M, gain);

  switch (gain) {
    case GAUSS4:
      _mgauss_per_lsb = 0.14;
      break;
    case GAUSS8:
      _mgauss_per_lsb = 0.29;
      break;
    case GAUSS12:
      _mgauss_per_lsb = 0.43;
      break;
    case GAUSS16:
      _mgauss_per_lsb = 0.58;
      break;
  }
}

void MARG::read_accel(float a[3], bool compensate) {
  // Read the accelerometer.
  byte buffer[6];
  read_regs(slave::IMU, registers::OUT_X_L_XL, buffer, 6);
  int16_t xyz[3];

  join_xys_bytes(buffer, xyz);

  a[0] = xyz[0] * _g_per_lsb * 9.80665;
  a[1] = -xyz[1] * _g_per_lsb * 9.80665;
  a[2] = xyz[2] * _g_per_lsb * 9.80665;

  if (compensate) {
    // TODO: Correct sensor bias and scale.
  }
}

void MARG::read_gyro(float g[3], bool compensate) {
  // Read gyro.
  byte buffer[6];
  read_regs(slave::IMU, registers::OUT_X_L_G, buffer, 6);
  int16_t xyz[3];

  join_xys_bytes(buffer, xyz);

  g[0] = xyz[0] * _dps_per_lsb * 0.017453293;
  g[1] = -xyz[1] * _dps_per_lsb * 0.017453293;
  g[2] = xyz[2] * _dps_per_lsb * 0.017453293;

  if (compensate) {
    // TODO: Correct sensor bias and scale.
  }
}

void MARG::read_mag(float m[3], bool compensate) {
  byte buffer[6];
  read_regs(slave::MAG, registers::OUT_X_L_M, buffer, 6);
  int16_t xyz[3];

  join_xys_bytes(buffer, xyz);

  m[0] = -xyz[1] * _mgauss_per_lsb * 0.1;
  m[1] = xyz[0] * _mgauss_per_lsb * 0.1;
  m[2] = xyz[2] * _mgauss_per_lsb * 0.1;

  if (compensate) {
    // TODO: Correct sensor bias and scale.
  }
}

void MARG::read_temp(float& t) {}

void MARG::read(float a[3], float g[3], float m[3], float& t, bool compensate) {
  read_accel(a, compensate);
  read_gyro(g, compensate);
  read_mag(m, compensate);
  read_temp(t);
}

int MARG::read_reg(uint8_t slave_address, uint8_t address) {
  _wire->beginTransmission(slave_address);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slave_address, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int MARG::read_regs(uint8_t slave_address, uint8_t address, uint8_t* data,
                    size_t length) {
  _wire->beginTransmission(slave_address);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(slave_address, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int MARG::write_reg(uint8_t slave_address, uint8_t address, uint8_t value) {
  _wire->beginTransmission(slave_address);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}
