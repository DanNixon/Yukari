/** @file */

#include "CaptureCLI.h"

using namespace Yukari::Triggers;

namespace Yukari
{
namespace CaptureApp
{
  CaptureCLI::CaptureCLI(std::istream &in, std::ostream &out)
      : CLI(in, out)
      , m_hasInit(false)
      , m_captureTrigger(std::make_shared<CLITrigger>("capture", "Captures new frame"))
  {
    registerCommand(m_captureTrigger->command());
  }

  void CaptureCLI::init(CaptureController_sptr controller)
  {
    if (m_hasInit || !controller)
      return;

    controller->addCaptureTrigger(m_captureTrigger);

    m_hasInit = true;
  }
}
}
