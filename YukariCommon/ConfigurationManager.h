/** @file */

#pragma once

#include <boost/property_tree/ptree.hpp>

namespace Yukari
{
namespace Common
{
  class ConfigurationManager
  {
  public:
    static boost::property_tree::ptree Load(const std::string &filename);
  };
}
}
