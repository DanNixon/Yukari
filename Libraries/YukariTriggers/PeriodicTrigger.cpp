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

  PeriodicTrigger::~PeriodicTrigger()
  {
    disable();
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
    auto nextWake = std::chrono::high_resolution_clock::now();
    while (m_enabled)
    {
      /* Sleep until *duration* from now */
      std::this_thread::sleep_until(nextWake);
      nextWake += m_duration;

      /* Run handler */
      m_handlerFunc();
    }
  }
}
}
