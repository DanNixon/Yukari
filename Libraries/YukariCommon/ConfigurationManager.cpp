/** @file */

#include "ConfigurationManager.h"

#include <boost/property_tree/json_parser.hpp>

#ifdef __linux__
#include <pwd.h>
#endif

#ifdef _WIN32
#include <Shlobj.h>
#include <Windows.h>
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
    auto logger = LoggingService::GetLogger("ConfigurationManager");
    boost::filesystem::path retVal;

#if defined(__linux__)
    struct passwd *pwd = getpwuid(getuid());
    if (pwd)
      retVal = boost::filesystem::path(pwd->pw_dir);
    else
      retVal = boost::filesystem::path(getenv("HOME"));
#endif

#if defined(_WIN32)
    WCHAR path[MAX_PATH];
    char charPath[MAX_PATH];
    char c = ' ';
    if (SUCCEEDED(SHGetFolderPathW(NULL, CSIDL_PROFILE, NULL, 0, path)) &&
        SUCCEEDED(WideCharToMultiByte(CP_ACP, 0, path, -1, charPath, 260, &c, NULL)))
    {
      retVal = boost::filesystem::path(charPath);
    }
#endif

    logger->debug("Got home directory: {}", retVal);

    return retVal;
  }

  boost::filesystem::path ConfigurationManager::GetAppDataDirectory(const std::string &appName)
  {
    auto logger = LoggingService::GetLogger("ConfigurationManager");
    boost::filesystem::path retVal;

#if defined(__linux__)
    retVal = GetUserHomeDirectory() / ("." + appName);
#endif

#if defined(_WIN32)
    WCHAR path[MAX_PATH];
    char charPath[MAX_PATH];
    char c = ' ';
    if (SUCCEEDED(SHGetFolderPathW(NULL, CSIDL_COMMON_APPDATA, NULL, 0, path)) &&
        SUCCEEDED(WideCharToMultiByte(CP_ACP, 0, path, -1, charPath, 260, &c, NULL)))
    {
      retVal = boost::filesystem::path(charPath) / appName;
    }
#endif

    logger->debug("Got application data directory: {}", retVal);

    return retVal;
  }
}
}
