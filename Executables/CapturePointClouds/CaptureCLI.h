/** @file */

#pragma once

#include <YukariCLI/CLI.h>
#include <YukariTriggers/CLITrigger.h>

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
    Triggers::CLITrigger_sptr m_captureTrigger;
  };
}
}
