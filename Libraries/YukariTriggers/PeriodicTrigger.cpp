/** @file */

#include "PeriodicTrigger.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Triggers
{
  PeriodicTrigger::PeriodicTrigger(std::chrono::milliseconds duration)
      : m_duration(duration)
      , m_enabled(false)
  {
    LoggingService::Instance()
        .getLogger("PeriodicTrigger")
        ->debug("Init periodic trigger with duration {}ms", m_duration.count());
  }

  void PeriodicTrigger::enable()
  {
    m_enabled = true;

    m_worker = std::thread(&PeriodicTrigger::workerFunc, this);
  }

  void PeriodicTrigger::disable()
  {
    m_enabled = false;

    if (m_worker.joinable())
      m_worker.join();
  }

  void PeriodicTrigger::workerFunc()
  {
    while (m_enabled)
    {
      std::this_thread::sleep_for(m_duration);
      m_handlerFunc();
    }
  }
}
}
