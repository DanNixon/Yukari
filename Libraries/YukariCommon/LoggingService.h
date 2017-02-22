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
    typedef spdlog::sinks::stdout_sink_st DefaultSink;
	typedef std::shared_ptr<DefaultSink> DefaultSinkPtr;

  public:
    static void Configure(ConfigurationManager::Config &config);
    static void Disable();
    static void Flush();
    static Logger GetLogger(const std::string &name);

  private:
    static DefaultSinkPtr GetDefaultSink();
    static spdlog::level::level_enum GetLogLevelFromStr(const std::string &levelStr);
    static void EnsureLogDirectoryExists(const std::string &filename);

  private:
    static DefaultSinkPtr m_defaultSink;
    static std::shared_ptr<spdlog::sinks::dist_sink_st> m_sink;
  };
}
}
