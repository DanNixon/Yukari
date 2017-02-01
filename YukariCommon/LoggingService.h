/** @file */

#pragma once

#include <memory>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/dist_sink.h>

namespace Yukari
{
namespace Common
{
  class LoggingService
  {
    public:
      static void Init();
      static void Configure();

      static std::shared_ptr<spdlog::logger> GetLogger(const std::string &name);

    private:
      static std::shared_ptr<spdlog::sinks::dist_sink_st> m_sink;
  };
}
}
