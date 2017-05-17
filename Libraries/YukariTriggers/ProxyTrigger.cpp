/** @file */

#include "ProxyTrigger.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;

namespace Yukari
{
namespace Triggers
{
  ProxyTrigger::ProxyTrigger()
      : m_enabled(false)
  {
  }

  void ProxyTrigger::enable()
  {
    m_enabled = true;
  }

  void ProxyTrigger::disable()
  {
    m_enabled = false;
  }

  void ProxyTrigger::trigger()
  {
    LoggingService::Instance().getLogger("ProxyTrigger")->debug("Triggered.");
    if (m_enabled)
      m_handlerFunc();
  }
}
}
