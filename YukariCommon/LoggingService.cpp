/** @file */

#include "LoggingService.h"

namespace Yukari
{
namespace Common
{
  std::shared_ptr<spdlog::sinks::dist_sink_st> LoggingService::m_sink;

  void LoggingService::Init()
  {
    if (!m_sink)
      m_sink = std::make_shared<spdlog::sinks::dist_sink_st>();
  }

  void LoggingService::Configure()
  {
    if (!m_sink)
      return;

    /* TODO */
    auto sink1 = std::make_shared<spdlog::sinks::stdout_sink_st>();
    auto sink2 =
        std::make_shared<spdlog::sinks::simple_file_sink_st>("mylog.log");

    m_sink->add_sink(sink1);
    m_sink->add_sink(sink2);
  }

  std::shared_ptr<spdlog::logger>
  LoggingService::GetLogger(const std::string &name)
  {
    auto logger = spdlog::get(name);
    if (!logger)
    {
      logger = std::make_shared<spdlog::logger>(name, m_sink);
      spdlog::register_logger(logger);
    }

    return logger;
  }
}
}
