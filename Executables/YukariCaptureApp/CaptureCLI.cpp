/** @file */

#include "CaptureCLI.h"

namespace Yukari
{
namespace CaptureApp
{
  CaptureCLI::CaptureCLI(std::istream &in, std::ostream &out)
      : CLI(in, out)
      , m_startTrigger("start", "Start new capture")
      , m_stopTrigger("stop", "Stops current capture")
      , m_captureTrigger("capture", "Captures new frame")
      , m_exitTrigger("exit2", "Exits capture")
  {
    registerCommand(m_startTrigger.command());
    registerCommand(m_stopTrigger.command());
    registerCommand(m_captureTrigger.command());
    registerCommand(m_exitTrigger.command());
  }
}
}
