/** @file */

#include "TimelapseCaptureTrigger.h"

namespace Yukari
{
namespace CaptureTriggers
{
  TimelapseCaptureTrigger::TimelapseCaptureTrigger(std::chrono::seconds duration)
      : m_duration(duration)
      , m_enabled(false)
  {
  }

  void TimelapseCaptureTrigger::enable()
  {
    m_enabled = true;

    m_worker = std::thread(&TimelapseCaptureTrigger::workerFunc, this);
  }

  void TimelapseCaptureTrigger::disable()
  {
    m_enabled = false;

    if (m_worker.joinable())
      m_worker.join();
  }

  void TimelapseCaptureTrigger::workerFunc()
  {
    while (m_enabled)
    {
      std::this_thread::sleep_for(m_duration);
      m_handlerFunc();
    }
  }
}
}
