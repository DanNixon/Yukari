/** @file */

#pragma once

#include <YukariCLI/CLI.h>

#include "CLITrigger.h"

namespace Yukari
{
namespace CaptureApp
{
  class CaptureCLI : public CLI::CLI
  {
  public:
    CaptureCLI(std::istream &in, std::ostream &out);

  private:
    CLITrigger m_startTrigger;
    CLITrigger m_stopTrigger;
    CLITrigger m_captureTrigger;
    CLITrigger m_exitTrigger;
  };
}
}
