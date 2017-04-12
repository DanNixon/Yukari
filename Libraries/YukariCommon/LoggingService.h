/** @file */

#pragma once

#include <memory>

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#include "NamedDistLogSink.h"

namespace Yukari
{
namespace Common
{
  class LoggingService
  {
  public:
    typedef std::shared_ptr<spdlog::logger> Logger;

  public:
    static spdlog::level::level_enum GetLogLevelFromStr(const std::string &levelStr);
    static void EnsureLogDirectoryExists(const std::string &filename);

  public:
    static LoggingService &Instance()
    {
      static LoggingService instance;
      return instance;
    }

  private:
    LoggingService();
    ~LoggingService();

  public:
    inline std::shared_ptr<NamedDistLogSink_mt> getSink()
    {
      return m_sink;
    }

    inline void disable()
    {
      m_sink->set_level(spdlog::level::off);
    }

    inline void flush()
    {
      m_sink->flush();
    }

    Logger getLogger(const std::string &name);

  private:
    std::shared_ptr<NamedDistLogSink_mt> m_sink;
  };
}
}
