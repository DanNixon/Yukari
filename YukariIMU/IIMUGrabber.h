/** @file */

#pragma once

#include <memory>

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  class IIMUGrabber
  {
  public:
    virtual void open()
    {
    }

    virtual void close()
    {
    }

    virtual bool isOpen() const
    {
      return true;
    }

    virtual IMUFrame_sptr grabFrame() = 0;
  };

  typedef std::shared_ptr<IIMUGrabber> IIMUGrabber_sptr;
}
}
