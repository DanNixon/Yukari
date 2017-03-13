/** @file */

#pragma once

#include "ITrigger.h"

namespace Yukari
{
namespace CaptureTriggers
{
  class MSPAUXTrigger : public ITrigger
  {
  public:
    MSPAUXTrigger();

    virtual void enable() override;
    virtual void disable() override;
  };
}
}
