/** @file */

#pragma once

#include <memory>

namespace Yukari
{
namespace CloudCapture
{
  class ICloudGrabber
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

    virtual CloudConstPtr getCloud() = 0;
  };

  typedef std::shared_ptr<ICloudGrabber> ICloudGrabber_sptr;
}
}
