/** @file */

#pragma once

#include "ITrigger.h"
#include <YukariCommon/SignalBroker.h>

namespace Yukari
{
namespace CaptureApp
{
  class SignalTrigger : public ITrigger, public Common::ISignalSubscriber
  {
  public:
    SignalTrigger(int signal);
    virtual ~SignalTrigger();

    virtual void enable() override;
    virtual void disable() override;
    virtual void handleSignal(int signal) override;

  private:
    int m_signal;
  };
}
}
