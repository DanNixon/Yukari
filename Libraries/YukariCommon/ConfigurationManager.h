/** @file */

#pragma once

#include <boost/filesystem.hpp>
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
    static Config LoadFromAppDataDirectory(const std::string &appName, const std::string &filename);

    static boost::filesystem::path GetUserHomeDirectory();
    static boost::filesystem::path GetAppDataDirectory(const std::string &appName);
  };
}
}
