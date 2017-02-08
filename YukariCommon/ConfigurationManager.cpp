/** @file */

#include "ConfigurationManager.h"

#include <boost/property_tree/json_parser.hpp>

#if defined(__linux__)
#include <pwd.h>
#endif

#if defined(_WIN32)
/* TODO */
#endif

#include "LoggingService.h"

namespace Yukari
{
namespace Common
{
  ConfigurationManager::Config ConfigurationManager::Load(const std::string &filename)
  {
    auto logger = LoggingService::GetLogger("ConfigurationManager");

    boost::property_tree::ptree pt;
    try
    {
      logger->debug("Reading configuration file \"{}\"", filename);
      boost::property_tree::read_json(filename, pt);
    }
    catch (boost::property_tree::json_parser_error &e)
    {
      logger->warn("Failed to read configuration file \"{}\": {}", filename, e.what());
    }

    return pt;
  }

  ConfigurationManager::Config
  ConfigurationManager::LoadFromAppDataDirectory(const std::string &appName,
                                                 const std::string &filename)
  {
    auto path = GetAppDataDirectory(appName) / filename;
    return Load(path.string());
  }

  boost::filesystem::path ConfigurationManager::GetUserHomeDirectory()
  {
#if defined(__linux__)
    struct passwd *pwd = getpwuid(getuid());
    if (pwd)
      return boost::filesystem::path(pwd->pw_dir);
    else
      return boost::filesystem::path(getenv("HOME"));
#endif

#if defined(_WIN32)
/* TODO */
#endif

    throw std::runtime_error("Unknown platform");
  }

  boost::filesystem::path ConfigurationManager::GetAppDataDirectory(const std::string &appName)
  {
#if defined(__linux__)
    auto dir = GetUserHomeDirectory() / ("." + appName);
    return dir;
#endif

#if defined(_WIN32)
/* TODO */
#endif

    throw std::runtime_error("Unknown platform");
  }
}
}
