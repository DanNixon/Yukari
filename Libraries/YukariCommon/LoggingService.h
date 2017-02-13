/** @file */

#pragma once

#include <memory>

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/spdlog.h>

#include "ConfigurationManager.h"

namespace Yukari
{
namespace Common
{
  class LoggingService
  {
  public:
    typedef std::shared_ptr<spdlog::logger> Logger;

  public:
    static void Configure(ConfigurationManager::Config &config);

    static Logger GetLogger(const std::string &name);

  private:
    static spdlog::level::level_enum GetLogLevelFromStr(const std::string &levelStr);
    static void EnsureLogDirectoryExists(const std::string &filename);

  private:
    static std::shared_ptr<spdlog::sinks::dist_sink_st> m_sink;
  };
}
}
