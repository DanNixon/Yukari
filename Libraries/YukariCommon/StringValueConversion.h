/** @file */

#pragma once

#include <string>
#include <vector>

#include <boost/any.hpp>

namespace Yukari
{
namespace Common
{
  class StringValueConversion
  {
  public:
    static boost::any Convert(std::string type, const std::string value);
    static std::vector<boost::any> Convert(std::string type,
                                           const std::vector<std::string> &values);

    static std::string Convert(std::string type, const boost::any &value);
  };
}
}
