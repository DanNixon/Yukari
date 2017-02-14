/** @file */

#include "SignalBroker.h"

#include "LoggingService.h"

namespace Yukari
{
namespace Common
{
  std::vector<ISignalSubscriber *> SignalBroker::m_handlerSubscriptions;
  std::vector<int> SignalBroker::m_signalSubscriptions;

  void SignalBroker::HandlerSubscribe(ISignalSubscriber *handler)
  {
    auto it = std::find(m_handlerSubscriptions.begin(), m_handlerSubscriptions.end(), handler);
    if (it == m_handlerSubscriptions.end())
      m_handlerSubscriptions.push_back(handler);
  }

  void SignalBroker::HandlerUnsubscribe(ISignalSubscriber *handler)
  {
    auto it = std::find(m_handlerSubscriptions.begin(), m_handlerSubscriptions.end(), handler);
    if (it != m_handlerSubscriptions.end())
      m_handlerSubscriptions.erase(it);
  }

  void SignalBroker::SignalSubscribe(int signal)
  {
    auto it = std::find(m_signalSubscriptions.begin(), m_signalSubscriptions.end(), signal);
    if (it == m_signalSubscriptions.end())
    {
      std::signal(signal, SignalBroker::HandlerFunc);
      m_signalSubscriptions.push_back(signal);
    }
  }

  void SignalBroker::HandlerFunc(int signal)
  {
    for (auto it = m_handlerSubscriptions.begin(); it != m_handlerSubscriptions.end(); ++it)
      (*it)->handleSignal(signal);
  }
}
}
