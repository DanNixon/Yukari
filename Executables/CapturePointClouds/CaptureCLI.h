/** @file */

#pragma once

#include <YukariCLI/CLI.h>
#include <YukariCaptureTriggers/CLITrigger.h>

#include "CaptureController.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureCLI : public CLI::CLI
  {
  public:
    CaptureCLI(std::istream &in, std::ostream &out);

    void init(CaptureController_sptr controller);

  private:
    bool m_hasInit;
    CaptureTriggers::CLITrigger_sptr m_startTrigger;
    CaptureTriggers::CLITrigger_sptr m_stopTrigger;
    CaptureTriggers::CLITrigger_sptr m_captureTrigger;
    CaptureTriggers::CLITrigger_sptr m_exitTrigger;
  };
}
}
