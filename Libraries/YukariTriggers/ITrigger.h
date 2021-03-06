/** @file */

#pragma once

#include <functional>
#include <memory>

namespace Yukari
{
namespace Triggers
{
  class ITrigger
  {
  public:
    typedef std::function<void(void)> TriggerHandlerFunc;

    typedef std::shared_ptr<ITrigger> Ptr;

  public:
    virtual void enable() = 0;
    virtual void disable() = 0;

    inline void setHandler(TriggerHandlerFunc func)
    {
      m_handlerFunc = func;
    }

  protected:
    TriggerHandlerFunc m_handlerFunc;
  };
}
}
