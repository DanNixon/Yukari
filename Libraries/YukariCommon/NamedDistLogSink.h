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
  template <class Mutex> class NamedDistLogSink : public spdlog::sinks::base_sink<Mutex>
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
      std::lock_guard<Mutex> lock(base_sink<Mutex>::_mutex);
      for (auto &sink : m_sinks)
        sink.second->flush();
    }

    void addSink(const std::string &name, std::shared_ptr<sink> sink)
    {
      std::lock_guard<Mutex> lock(base_sink<Mutex>::_mutex);
      m_sinks[name] = sink;
    }

    void removeSink(const std::string &name)
    {
      std::lock_guard<Mutex> lock(base_sink<Mutex>::_mutex);
      auto it = m_sinks.find(name);
      if (it != m_sinks.end())
        m_sinks.erase(it);
    }

  protected:
    void _sink_it(const spdlog::details::log_msg &msg) override
    {
      for (auto &sink : m_sinks)
      {
        if (sink.second->should_log(msg.level))
        {
          sink.second->log(msg);
        }
      }
    }

  protected:
    std::map<std::string, std::shared_ptr<sink>> m_sinks;
  };

  typedef NamedDistLogSink<std::mutex> NamedDistLogSink_mt;
  typedef NamedDistLogSink<spdlog::details::null_mutex> NamedDistLogSink_st;
}
}
