/** @file */

#pragma once

#include "IGrabber.h"

#include <chrono>
#include <serial/serial.h>

namespace Yukari
{
namespace IMU
{
  class MSPGrabberIMU : public IGrabber
  {
  public:
    MSPGrabberIMU(const std::string &port, unsigned int baud = 115200);
    virtual ~MSPGrabberIMU();

    virtual void open();
    virtual void close();
    virtual bool isOpen() const;

    virtual IMUFrame_sptr grabFrame();

  protected:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
    serial::Serial m_port;
  };
}
}
