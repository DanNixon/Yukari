/** @file */

#include "CaptureCLI.h"

namespace Yukari
{
namespace CaptureApp
{
  CaptureCLI::CaptureCLI(std::istream &in, std::ostream &out)
      : CLI(in, out, false)
      , m_hasInit(false)
      , m_startTrigger(std::make_shared<CLITrigger>("start", "Start new capture"))
      , m_stopTrigger(std::make_shared<CLITrigger>("stop", "Stops current capture"))
      , m_captureTrigger(std::make_shared<CLITrigger>("capture", "Captures new frame"))
      , m_exitTrigger(std::make_shared<CLITrigger>("exit", "Exits the application"))
  {
    registerCommand(m_startTrigger->command());
    registerCommand(m_stopTrigger->command());
    registerCommand(m_captureTrigger->command());
    registerCommand(m_exitTrigger->command());
  }

  void CaptureCLI::init(CaptureController_sptr controller)
  {
    if (m_hasInit || !controller)
      return;

    controller->addStartTrigger(m_startTrigger);
    controller->addStopTrigger(m_stopTrigger);
    controller->addCaptureTrigger(m_captureTrigger);
    controller->addExitTrigger(m_exitTrigger);

    m_hasInit = true;
  }
}
}
