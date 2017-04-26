/** @file */

#include "ProxyTrigger.h"

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
    if (m_enabled)
      m_handlerFunc();
  }
}
}
