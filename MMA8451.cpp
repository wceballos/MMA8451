/**
 * @file      MMA8451.cpp
 * @brief     Driver for the MMA8451 3-axis Accelerometer
 * @author    Wilmin Ceballos (wceballos)
 * @version   1.0
 * @copyright MIT License
 *
 * Copyright (c) 2019 Wilmin Ceballos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "MMA8451.h"

MMA8451::MMA8451(I2C &i2c, uint8_t i2cAddr) {
  _i2c = &i2c;
  _i2cAddr = static_cast<uint8_t>(i2cAddr) << 1;
  _i2c->frequency(MMA8451_I2C_CLOCK);
}

MMA8451::~MMA8451() {

}

bool MMA8451::init() {

  if(id() != 0x1A)
    return false;

  writeRegister(MMA8451_CTRL_REG2, 0x40); // Trigger soft reset
  // RST bit in CTRL_REG2 is deasserted when boot is finished
  uint8_t ctrlReg2 = 0;
  do {
    readRegister(MMA8451_CTRL_REG2, &ctrlReg2);
  } while(ctrlReg2 & 0x40);
  setActiveMode();
  _sens = MMA8451_SENSITIVITY_2G; // This is the default after reset

  return true;
}

void MMA8451::getXYZ() {
  char ibuf[6] = {0}; // Incoming data
  char baseReg = MMA8451_OUT_X_MSB;
  
  _i2c->write(_i2cAddr, &baseReg, 1, true);
  _i2c->read(_i2cAddr, ibuf, 6, true);

  // Data is 14 bits -> 0b00DDDDDD_DDDDDDDD
  int16_t x_int = (static_cast<int16_t>(ibuf[0]))<<8 | ibuf[1]; x_int>>=2;
  int16_t y_int = (static_cast<int16_t>(ibuf[2]))<<8 | ibuf[3]; y_int>>=2;
  int16_t z_int = (static_cast<int16_t>(ibuf[4]))<<8 | ibuf[5]; z_int>>=2;

  int16_t div = 1;
  if(_sens == MMA8451_SENSITIVITY_2G)
    div = 4096;
  else if(_sens == MMA8451_SENSITIVITY_4G)
    div = 2048;
  else if(_sens == MMA8451_SENSITIVITY_8G)
    div = 1024;

  x = static_cast<float>(x_int) / div;
  y = static_cast<float>(y_int) / div;
  z = static_cast<float>(z_int) / div;
}

float MMA8451::getX() {
  x = _readAxis(MMA8451_OUT_X_MSB);
  return x;
}

float MMA8451::getY() {
  y = _readAxis(MMA8451_OUT_Y_MSB);
  return y;
}

float MMA8451::getZ() {
  z = _readAxis(MMA8451_OUT_Z_MSB);
  return z;
}

uint8_t MMA8451::id() {
    readRegister(MMA8451_WHO_AM_I, &_whoami);
    return _whoami;
}

bool MMA8451::readRegister(uint8_t reg, uint8_t *data) {
  int status = _i2c->write(_i2cAddr, (char*)&reg, 1, true);
  status |= _i2c->read(_i2cAddr, (char*)data, 1, false);
  if(status != 0)
    return false;
  return true;
}

bool MMA8451::writeRegister(uint8_t reg, uint8_t data) {
  char buf[2] = {reg, data};
  int status = _i2c->write(_i2cAddr, buf, 2, false);
  if(status != 0)
    return false;
  return true;
}

float MMA8451::_readAxis(uint8_t reg) {
  char buf[2] = {0}; // Incoming data

  _i2c->write(_i2cAddr, (char*)&reg, 1, true);
  _i2c->read(_i2cAddr, buf, 2, true);

  // Data is 14 bits -> 0b00DDDDDD_DDDDDDDD
  int16_t axisData = (static_cast<int16_t>(buf[0]))<<8 | buf[1]; axisData>>=2;

  int16_t div = 1;
  if(_sens == MMA8451_SENSITIVITY_2G)
    div = 4096;
  else if(_sens == MMA8451_SENSITIVITY_4G)
    div = 2048;
  else if(_sens == MMA8451_SENSITIVITY_8G)
    div = 1024;

  return (static_cast<float>(axisData) / div);
}

void MMA8451::setStandbyMode() {
  uint8_t ctrlReg1;
  const uint8_t ACTIVE_MASK = 0x01;
  readRegister(MMA8451_CTRL_REG1, &ctrlReg1);
  writeRegister(MMA8451_CTRL_REG1, (ctrlReg1 & ~ACTIVE_MASK));
}

void MMA8451::setActiveMode() {
  uint8_t ctrlReg1;
  const uint8_t ACTIVE_MASK = 0x01;
  readRegister(MMA8451_CTRL_REG1, &ctrlReg1);
  writeRegister(MMA8451_CTRL_REG1, (ctrlReg1 | ACTIVE_MASK));
}

void MMA8451::setDataRate(mma8451_dataRate_t rate) {
  uint8_t ctrlReg1;
  const uint8_t DR_MASK = 0x38;
  uint8_t dataRate = static_cast<uint8_t>(rate) << 3;

  setStandbyMode();
  readRegister(MMA8451_CTRL_REG1, &ctrlReg1);
  writeRegister(MMA8451_CTRL_REG1, (ctrlReg1 & ~DR_MASK));
  writeRegister(MMA8451_CTRL_REG1, (ctrlReg1 | dataRate));
  setActiveMode();
}

void MMA8451::setSensitivity(mma8451_sensitivity_t sens) {
  uint8_t dataCfgReg = 0;
  const uint8_t FS_MASK = 0x03;

  setStandbyMode();
  readRegister(MMA8451_XYZ_DATA_CFG, &dataCfgReg);
  writeRegister(MMA8451_XYZ_DATA_CFG, (dataCfgReg & ~FS_MASK));
  writeRegister(MMA8451_XYZ_DATA_CFG, (dataCfgReg | sens));
  setActiveMode();
  _sens = sens;
}

