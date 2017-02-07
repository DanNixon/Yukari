/** @file */

#pragma once

#include "MSPGrabber.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabberIMU : public MSPGrabber
  {
  public:
    MSPGrabberIMU(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberIMU();

    virtual IMUFrame_sptr grabFrame();

  protected:
    int16_t m_gyro[3];
    int16_t m_acc[3];
    int16_t m_mag[3];
  };
}
}
