/** @file */

#pragma once

#include <string>
#include <map>

#include "IIMUGrabber.h"

namespace Yukari
{
namespace IMU
{
  class IMUGrabberFactory
  {
  public:
    static IIMUGrabber_sptr Create(const std::string &fullCommand);
    static IIMUGrabber_sptr Create(const std::string &type, std::map<std::string, std::string> & parameters);
  };
}
}
