#pragma once

#include <spdlog/details/log_msg.h>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/sink.h>

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>

namespace Yukari
{
namespace Common
{
  template <class MUTEX_TYPE> class NamedDistLogSink : public spdlog::sinks::base_sink<MUTEX_TYPE>
  {
  public:
    explicit NamedDistLogSink()
        : m_sinks()
    {
    }

    NamedDistLogSink(const NamedDistLogSink &) = delete;
    NamedDistLogSink &operator=(const NamedDistLogSink &) = delete;
    virtual ~NamedDistLogSink() = default;

    void flush() override
    {
      std::lock_guard<MUTEX_TYPE> lock(spdlog::sinks::base_sink<MUTEX_TYPE>::_mutex);
      for (auto it = m_sinks.begin(); it != m_sinks.end(); ++it)
        it->second->flush();
    }

    void addSink(const std::string &name, std::shared_ptr<spdlog::sinks::sink> sink)
    {
      std::lock_guard<MUTEX_TYPE> lock(spdlog::sinks::base_sink<MUTEX_TYPE>::_mutex);
      m_sinks[name] = sink;
    }

    void removeSink(const std::string &name)
    {
      std::lock_guard<MUTEX_TYPE> lock(spdlog::sinks::base_sink<MUTEX_TYPE>::_mutex);
      auto it = m_sinks.find(name);
      if (it != m_sinks.end())
        m_sinks.erase(it);
    }

  protected:
    void _sink_it(const spdlog::details::log_msg &msg) override
    {
      for (auto it = m_sinks.begin(); it != m_sinks.end(); ++it)
      {
        if (it->second->should_log(msg.level))
          it->second->log(msg);
      }
    }

  protected:
    std::map<std::string, std::shared_ptr<spdlog::sinks::sink>> m_sinks;
  };

  typedef NamedDistLogSink<std::mutex> NamedDistLogSink_mt;
  typedef NamedDistLogSink<spdlog::details::null_mutex> NamedDistLogSink_st;
}
}
