/** @file */

#pragma once

#include <memory>

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
    static void Init();
    static void Configure(ConfigurationManager::Config &config);

    static std::shared_ptr<spdlog::logger> GetLogger(const std::string &name);

  private:
    static spdlog::level::level_enum GetLogLevelFromStr(const std::string &levelStr);

  private:
    static std::shared_ptr<spdlog::sinks::dist_sink_st> m_sink;
  };
}
}
