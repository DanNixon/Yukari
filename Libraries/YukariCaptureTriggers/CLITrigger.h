/** @file */

#pragma once

#include <YukariCLI/Command.h>

#include "ITrigger.h"

namespace Yukari
{
namespace CaptureTriggers
{
  class CLITrigger : public ITrigger
  {
  public:
    CLITrigger(const std::string &cmdName, const std::string &description = "");

    inline CLI::Command_sptr command()
    {
      return m_cliCommand;
    }

    void enable() override;
    void disable() override;

    void trigger();

  private:
    CLI::Command_sptr m_cliCommand;
  };

  typedef std::shared_ptr<CLITrigger> CLITrigger_sptr;
}
}
