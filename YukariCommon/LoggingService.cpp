/** @file */

#include "LoggingService.h"

#include <iostream>

#include <boost/foreach.hpp>
#include <spdlog/sinks/base_sink.h>

namespace Yukari
{
namespace Common
{
  std::shared_ptr<spdlog::sinks::dist_sink_st> LoggingService::m_sink;

  void LoggingService::Init()
  {
    if (!m_sink)
    {
      m_sink = std::make_shared<spdlog::sinks::dist_sink_st>();
    }
    else
    {
      std::cerr << "Logger already initialised!\n";
    }
  }

  void LoggingService::Configure(ConfigurationManager::Config &config)
  {
    if (!m_sink)
    {
      std::cerr << "Logger not initialised!\n";
      return;
    }

    auto sinks = config.get_child_optional("logging.sinks");

    /* Create and add sinks */
    if (sinks && !sinks->empty())
    {
      for (auto it = sinks->begin(); it != sinks->end(); ++it)
      {
        std::shared_ptr<spdlog::sinks::sink> sink;

        const std::string type = it->second.get<std::string>("type");
        if (type == "console")
        {
          sink = std::make_shared<spdlog::sinks::stdout_sink_st>();
        }
        else if (type == "file_simple")
        {
          const std::string logFile = it->second.get<std::string>("filename", "yukari.log");
          sink = std::make_shared<spdlog::sinks::simple_file_sink_st>(logFile);
        }
        else
        {
          throw std::runtime_error("Unknown log sink type: \"" + type + "\"");
        }

        if (sink)
        {
          /* Set level */
          const std::string levelStr = it->second.get<std::string>("level", "info");
          auto level = GetLogLevelFromStr(levelStr);
          sink->set_level(level);

          /* Add to distribution sink */
          m_sink->add_sink(sink);
        }
      }
    }
    else
    {
      std::cerr << "No log sinks defined, no logs will be available!\n";
    }

    GetLogger("LoggingService")->info("Logger configured.");
  }

  std::shared_ptr<spdlog::logger> LoggingService::GetLogger(const std::string &name)
  {
    auto logger = spdlog::get(name);
    if (!logger)
    {
      logger = std::make_shared<spdlog::logger>(name, m_sink);
      spdlog::register_logger(logger);
    }

    return logger;
  }

  spdlog::level::level_enum LoggingService::GetLogLevelFromStr(const std::string &levelStr)
  {
    auto end = spdlog::level::level_names + 7;
    auto pos = std::find(spdlog::level::level_names, end, levelStr);
    if (pos == end)
      throw std::runtime_error("Invalid log level: \"" + levelStr + "\"");
    spdlog::level::level_enum level = (spdlog::level::level_enum)(pos - spdlog::level::level_names);
    return level;
  }
}
}
