#pragma once

#include "IIMUGrabber.h"

#include <serial/serial.h>

namespace Yukari
{
namespace IMU
{
  class ISerialGrabber: public IIMUGrabber
  {
  public:
    ISerialGrabber(const std::string &port, unsigned int baud = 115200);
    virtual ~ISerialGrabber();

    virtual void open() override;
    virtual void close() override;
    virtual bool isOpen() const override;

    virtual IMUFrame::Ptr grabFrame() override = 0;

  protected:
    serial::Timeout m_defaultTimeout;
    serial::Serial m_port;
  };
}
}
