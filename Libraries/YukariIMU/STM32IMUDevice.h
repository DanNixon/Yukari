/** @file */

#pragma once

#include "ISerialGrabber.h"

namespace Yukari
{
namespace IMU
{
  class STM32IMUDevice : public ISerialGrabber
  {
  public:
    STM32IMUDevice(const std::string &port, unsigned int baud = 115200);
    virtual ~STM32IMUDevice();

    virtual IMUFrame::Ptr grabFrame() override;
  };
}
}
