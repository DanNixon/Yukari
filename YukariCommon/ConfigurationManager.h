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
    typedef boost::property_tree::ptree Config;

  public:
    static Config Load(const std::string &filename);
  };
}
}
