/** @file */

#pragma once

#include "IIMUGrabber.h"

#include <chrono>
#include <memory>
#include <serial/serial.h>

#include "MSPClient.h"

namespace Yukari
{
namespace IMU
{
  class DummyIMUGrabber : public IIMUGrabber
  {
  public:
    DummyIMUGrabber();

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual IMUFrame_sptr grabFrame() override;

  protected:
    bool m_open;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastFrameTime;
  };
}
}