/** @file */

#pragma once

#include "ICloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  class OpenNI2CloudGrabber
  {
  public:
    OpenNI2CloudGrabber();
    virtual ~OpenNI2CloudGrabber();

    virtual void open();
    virtual void close();
    virtual bool isOpen() const;

    virtual CloudConstPtr getCloud();
  };
}
}
