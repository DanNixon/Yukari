#pragma once

#include "IMU.h"

#include <I2Cdev.h>
#include <MPU9150.h>

class IMU_MPU9150 : public IMU, public MPU9150
{
public:
  virtual bool init() override;

  virtual void sampleGyro() override;
  virtual void sampleAccel() override;
  virtual void sampleMag() override;

private:
  uint8_t m_gyroFSD;
  uint8_t m_accelFSD;
};
