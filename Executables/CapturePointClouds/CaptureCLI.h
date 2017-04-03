/** @file */

#pragma once

#include <YukariCLI/CLI.h>

#include "CLITrigger.h"
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
    CLITrigger_sptr m_startTrigger;
    CLITrigger_sptr m_stopTrigger;
    CLITrigger_sptr m_captureTrigger;
    CLITrigger_sptr m_exitTrigger;
  };
}
}
