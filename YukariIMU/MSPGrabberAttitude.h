/** @file */

#pragma once

#include "MSPGrabber.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabberAttitude : public MSPGrabber
  {
  public:
    MSPGrabberAttitude(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberAttitude();

    virtual IMUFrame_sptr grabFrame();

    /* protected: */
    float m_attitude[3];
  };
}
}
