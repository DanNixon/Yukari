/** @file */

#pragma once

#include "IGrabber.h"

namespace Yukari
{
namespace IMU
{
  class MSPGrabber : public IGrabber
  {
  public:
    MSPGrabber();
    virtual ~MSPGrabber();

    virtual void open();
    virtual void close();
    virtual bool isOpen() const;

    virtual IMUFrame_sptr grabFrame();
  };
}
}
