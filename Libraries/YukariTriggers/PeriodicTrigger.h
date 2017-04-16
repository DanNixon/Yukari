/** @file */

#pragma once

#include "ITrigger.h"

#include <chrono>
#include <thread>

namespace Yukari
{
namespace Triggers
{
  class PeriodicTrigger : public ITrigger
  {
  public:
    PeriodicTrigger(std::chrono::milliseconds duration);

    virtual void enable() override;
    virtual void disable() override;

  private:
    void workerFunc();

  private:
    std::chrono::milliseconds m_duration;

    bool m_enabled;
    std::thread m_worker;
  };
}
}
