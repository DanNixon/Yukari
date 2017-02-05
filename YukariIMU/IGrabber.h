/** @file */

#pragma once

#include "IMUFrame.h"

namespace Yukari
{
namespace IMU
{
  class IGrabber
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
      return false;
    }

    virtual IMUFrame_sptr grabFrame() = 0;
  };
}
}
