/** @file */

#pragma once

#include <memory>
#include <string>

#include "IIMUGrabber.h"

#include "MSPGrabberAttitude.h"
#include "MSPGrabberIMU.h"

namespace Yukari
{
namespace IMU
{
  class IMUGrabberFactory
  {
  public:
    template <class... Args>
    static IIMUGrabber_sptr Create(const std::string &type, Args &&... args)
    {
      if (type == "attitude")
        return std::make_shared<MSPGrabberAttitude>(std::forward<Args>(args)...);

      if (type == "imu")
        return std::make_shared<MSPGrabberIMU>(std::forward<Args>(args)...);

      return {};
    }
  };
}
}
