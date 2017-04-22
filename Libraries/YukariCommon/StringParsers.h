/** @file */

#pragma once

#include <map>
#include <string>

#include <Eigen/Geometry>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "LoggingService.h"

namespace Yukari
{
namespace Common
{
  class StringParsers
  {
  public:
    inline static void CleanString(std::string &s)
    {
      boost::to_lower(s);
      boost::trim(s);
    }

    static bool ParseCommand(const std::string &input, std::string &command,
                             std::map<std::string, std::string> &parameters);
    static bool ParseParameterList(const std::string &input,
                                   std::map<std::string, std::string> &parameters);

    static Eigen::Quaternionf ParseQuaternion(const std::string &input);
    static Eigen::Vector3f ParseVector(const std::string &input);
  };
}
}