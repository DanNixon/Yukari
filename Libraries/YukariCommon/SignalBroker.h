/** @file */

#pragma once

#include <csignal>
#include <vector>

namespace Yukari
{
namespace Common
{
  class ISignalSubscriber
  {
  public:
    virtual void handleSignal(int signal) = 0;
  };

  class SignalBroker
  {
  public:
    static void HandlerSubscribe(ISignalSubscriber *handler);
    static void HandlerUnsubscribe(ISignalSubscriber *handler);

    static void SignalSubscribe(int signal);

  private:
    static void HandlerFunc(int signal);

  private:
    static std::vector<ISignalSubscriber *> m_handlerSubscriptions;
    static std::vector<int> m_signalSubscriptions;
  };
}
}
