/** @file */

#pragma once

#include <memory>
#include <string>

#include "ICloudGrabber.h"

#include "OpenNI2CloudGrabber.h"

namespace Yukari
{
namespace CloudCapture
{
  class CloudGrabberFactory
  {
  public:
    static ICloudGrabber_sptr Create(const std::string &name)
    {
    }

    template <class... Args>
    static ICloudGrabber_sptr Create(const std::string &type, Args &&... args)
    {
      if (type == "openni2")
        return std::make_shared<OpenNI2CloudGrabber>(std::forward<Args>(args)...);

      return {};
    }
  };
}
}
