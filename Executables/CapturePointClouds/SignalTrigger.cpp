/** @file */

#include "SignalTrigger.h"

using namespace Yukari::Common;

namespace Yukari
{
namespace CaptureApp
{
  SignalTrigger::SignalTrigger(int signal)
      : m_signal(signal)
  {
    SignalBroker::SignalSubscribe(signal);
  }

  SignalTrigger::~SignalTrigger()
  {
    SignalBroker::HandlerUnsubscribe(this);
  }

  void SignalTrigger::enable()
  {
    SignalBroker::HandlerSubscribe(this);
  }

  void SignalTrigger::disable()
  {
    SignalBroker::HandlerUnsubscribe(this);
  }

  void SignalTrigger::handleSignal(int signal)
  {
    if ((signal == m_signal) && m_handlerFunc)
      m_handlerFunc();
  }
}
}
