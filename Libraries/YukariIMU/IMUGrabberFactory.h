/** @file */

#pragma once

#include <map>
#include <string>

#include "IIMUGrabber.h"

namespace Yukari
{
namespace IMU
{
  class IMUGrabberFactory
  {
  public:
    static IIMUGrabber::Ptr Create(const std::string &fullCommand);
    static IIMUGrabber::Ptr Create(const std::string &type,
                                   std::map<std::string, std::string> &parameters);
  };
}
}
