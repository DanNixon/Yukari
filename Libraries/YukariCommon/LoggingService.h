/** @file */

#pragma once

#include <memory>

#include <boost/program_options.hpp>
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
    void configure(boost::program_options::variables_map & args);

    inline void setLevel(spdlog::level::level_enum level)
    {
      m_sink->set_level(level);
      getLogger("LoggingService")->info("Global log level: {}", level);
    }

    inline void disable()
    {
      m_sink->set_level(spdlog::level::off);
    }

    inline void flush()
    {
      m_sink->flush();
    }

    inline std::shared_ptr<NamedDistLogSink_mt> getSink()
    {
      return m_sink;
    }

    Logger getLogger(const std::string &name);

  private:
    std::shared_ptr<NamedDistLogSink_mt> m_sink;
  };
}
}
