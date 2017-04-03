/** @file */

#pragma once

#include <functional>
#include <memory>

namespace Yukari
{
namespace CaptureApp
{
  class ITrigger
  {
  public:
    typedef std::function<void(void)> TriggerHandlerFunc;

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

  typedef std::shared_ptr<ITrigger> ITrigger_sptr;
}
}
