/** @file */

#pragma once

#include "ITrigger.h"

namespace Yukari
{
namespace Triggers
{
  class ProxyTrigger : public ITrigger
  {
  public:
    ProxyTrigger();

    virtual void enable() override;
    virtual void disable() override;

    void trigger();

  private:
    bool m_enabled;
  };
}
}
